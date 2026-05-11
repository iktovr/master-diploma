"""Filter pedestrian infrastructure from an OSM file and export as GeoJSON.

Usage:
    bazel run //maps:filter_osm -- --input city.osm --output pedestrian.geojson
"""

import argparse
import json
import logging
import math
import os
from pathlib import Path

import networkx as nx
import osmium
from pyproj import Proj

logging.basicConfig(
    level=logging.INFO,
    format="%(levelname)s %(message)s",
)
log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# OSM tag sets that define pedestrian infrastructure
# ---------------------------------------------------------------------------

PEDESTRIAN_HIGHWAY_VALUES = frozenset(
    [
        "footway",
        "pedestrian",
        "path",
        "crossing",
        "living_street",
        "track",
    ]
)

# A way is also pedestrian when the foot tag signals explicit access.
FOOT_ACCESS_VALUES = frozenset(["yes", "designated", "permissive"])

DARK_STORE_NAME = "Яндекс.Лавка"

# Building entrance tag values that mark delivery points.
ENTRANCE_VALUES = frozenset(["yes", "main", "staircase", "home"])

# Maximum distance (metres) to connect a point node to the pedestrian network.
MAX_CONNECT_DISTANCE_M = 20.0
MIN_CONNECT_DISTANCE_M = 0.3


def _is_pedestrian_way(tags) -> bool:
    highway = tags.get("highway")
    if highway in PEDESTRIAN_HIGHWAY_VALUES:
        return True
    foot = tags.get("foot")
    if foot in FOOT_ACCESS_VALUES:
        return True
    return False


def _is_entrance(tags) -> bool:
    return tags.get("entrance") in ENTRANCE_VALUES


def _tags_to_dict(tags) -> dict:
    return {tag.k: tag.v for tag in tags}


# ---------------------------------------------------------------------------
# pyosmium handler
# ---------------------------------------------------------------------------


class PedestrianHandler(osmium.SimpleHandler):
    """Collect pedestrian ways, dark-store nodes and building entrance nodes
    from an OSM file in a single pass.

    ``apply_file`` is called with ``locations=True`` so that pyosmium
    automatically resolves node coordinates when processing ways and nodes.
    """

    def __init__(self):
        super().__init__()
        self.way_features: list[dict] = []
        self.store_features: list[dict] = []
        self.entrance_features: list[dict] = []

    def node(self, n):
        # Dark-store (Яндекс.Лавка) nodes → base_point
        if n.tags.get("dark_store") == "yes" and n.tags.get("name") == DARK_STORE_NAME:
            self.store_features.append(
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "Point",
                        "coordinates": [n.location.lon, n.location.lat],
                    },
                    "properties": {
                        "osm_id": n.id,
                        "osm_type": "node",
                        "type": "base_point",
                        **_tags_to_dict(n.tags),
                    },
                }
            )
            return

        # Building entrance nodes → delivery_point
        if _is_entrance(n.tags):
            self.entrance_features.append(
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "Point",
                        "coordinates": [n.location.lon, n.location.lat],
                    },
                    "properties": {
                        "osm_id": n.id,
                        "osm_type": "node",
                        "type": "delivery_point",
                        **_tags_to_dict(n.tags),
                    },
                }
            )

    def way(self, w):
        if not _is_pedestrian_way(w.tags):
            return

        try:
            coords = [[n.lon, n.lat] for n in w.nodes]
        except osmium.InvalidLocationError:
            # Some nodes may be outside the extract — skip the way.
            return

        if len(coords) < 2:
            return

        self.way_features.append(
            {
                "type": "Feature",
                "geometry": {
                    "type": "LineString",
                    "coordinates": coords,
                },
                "properties": {
                    "osm_id": w.id,
                    "osm_type": "way",
                    **_tags_to_dict(w.tags),
                },
            }
        )


# ---------------------------------------------------------------------------
# Connectivity filter — keep only the largest connected component
# ---------------------------------------------------------------------------


def _coord_key(coord: list) -> tuple:
    """Round to ~1 cm precision to merge near-duplicate endpoints."""
    return (round(coord[0], 7), round(coord[1], 7))


def largest_connected_component(features: list[dict]) -> list[dict]:
    """Return only the features belonging to the largest connected component."""
    if not features:
        return features

    G = nx.Graph()

    feature_nodes: list[set] = []
    for feat in features:
        coords = feat["geometry"]["coordinates"]
        keys = [_coord_key(c) for c in coords]
        G.add_nodes_from(keys)
        G.add_edges_from(zip(keys, keys[1:]))
        feature_nodes.append(set(keys))

    components = list(nx.connected_components(G))
    total = len(components)

    log.info("Connectivity check: %d ways, %d connected component(s).", len(features), total)

    if total == 1:
        return features

    components.sort(key=len, reverse=True)

    sizes_ways = [
        sum(1 for ns in feature_nodes if ns & comp)
        for comp in components
    ]
    log.info("Component sizes in ways: %s", sizes_ways)

    largest_nodes = components[0]
    kept = [
        feat
        for feat, ns in zip(features, feature_nodes)
        if ns & largest_nodes
    ]
    discarded = len(features) - len(kept)
    log.info(
        "Keeping largest component (%d ways), discarding %d ways across %d smaller component(s).",
        len(kept),
        discarded,
        total - 1,
    )
    return kept


# ---------------------------------------------------------------------------
# Geometry helpers — UTM projection for metric perpendicular calculations
# ---------------------------------------------------------------------------


def _utm_proj(lon: float, lat: float) -> Proj:
    """Return a UTM Proj object for the zone containing (lon, lat)."""
    zone = int((lon + 180) / 6) + 1
    hemisphere = "north" if lat >= 0 else "south"
    return Proj(proj="utm", zone=zone, ellps="WGS84", hemisphere=hemisphere)


def _to_utm(proj: Proj, lonlat: list) -> tuple[float, float]:
    return proj(lonlat[0], lonlat[1])


def _from_utm(proj: Proj, xy: tuple[float, float]) -> list:
    lon, lat = proj(xy[0], xy[1], inverse=True)
    return [lon, lat]


def _project_point_onto_segment_utm(
    p_xy: tuple, a_xy: tuple, b_xy: tuple
) -> tuple[float, float]:
    """Return the foot of the perpendicular from *p* onto segment *a*–*b* in
    UTM (metric) space and the unclamped parameter t.
    """
    ax, ay = a_xy
    bx, by = b_xy
    px, py = p_xy

    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy

    if seg_len_sq == 0.0:
        return a_xy

    t = ((px - ax) * dx + (py - ay) * dy) / seg_len_sq
    t_clamped = max(0.0, min(1.0, t))
    foot_xy = (ax + t_clamped * dx, ay + t_clamped * dy)
    return foot_xy


def _nearest_segment_projection(
    point: list, way_features: list[dict]
) -> tuple[list, int, int, float] | None:
    """Find the nearest **perpendicular** projection of *point* onto any segment
    of any way, computed in UTM metric space.

    Returns (foot_lonlat, way_index, seg_index, distance_m) or None if no
    valid perpendicular projection exists.
    """
    proj = _utm_proj(point[0], point[1])
    p_xy = _to_utm(proj, point)

    best_dist = math.inf
    best_foot_lonlat = None
    best_way_idx = -1
    best_seg_idx = -1

    for wi, feat in enumerate(way_features):
        coords = feat["geometry"]["coordinates"]
        for si in range(len(coords) - 1):
            a_xy = _to_utm(proj, coords[si])
            b_xy = _to_utm(proj, coords[si + 1])
            foot_xy = _project_point_onto_segment_utm(p_xy, a_xy, b_xy)
            dist = math.hypot(p_xy[0] - foot_xy[0], p_xy[1] - foot_xy[1])
            if dist < best_dist:
                best_dist = dist
                best_foot_lonlat = _from_utm(proj, foot_xy)
                best_way_idx = wi
                best_seg_idx = si

    if best_foot_lonlat is None:
        return None
    return best_foot_lonlat, best_way_idx, best_seg_idx, best_dist


# ---------------------------------------------------------------------------
# Connect point nodes to the nearest edge, splitting it at the projection
# ---------------------------------------------------------------------------


def connect_points_to_network(
    way_features: list[dict],
    point_features: list[dict],
    point_type: str,
    max_distance_m: float = MAX_CONNECT_DISTANCE_M,
    min_distance_m: float = MIN_CONNECT_DISTANCE_M,
) -> tuple[list[dict], list[dict], list[dict]]:
    """Connect each point node to the nearest segment of the pedestrian network.

    For each successfully connected point:
    - The target way is split into two ways at the projection foot.
    - A connector LineString is added from the point to the foot.

    Points whose nearest projection exceeds *max_distance_m* are excluded and
    logged at WARNING level.

    The *way_features* list is modified **in place**: split ways replace the
    original way.

    Returns:
        connected_points  — point features that were successfully connected
        skipped_points    — point features that exceeded the distance threshold
        connectors        — synthetic LineString features (one per connected point)
    """
    if not point_features or not way_features:
        return [], list(point_features), []

    connected_points: list[dict] = []
    skipped_points: list[dict] = []
    connectors: list[dict] = []

    for point in point_features:
        pt_coord = point["geometry"]["coordinates"]
        result = _nearest_segment_projection(pt_coord, way_features)
        if result is None:
            skipped_points.append(point)
            continue

        foot, wi, si, dist = result
        osm_id = point["properties"].get("osm_id", "?")

        if dist > max_distance_m or dist < min_distance_m:
            log.warning(
                "%s osm_id=%s: nearest segment is %.1f m away, skipping.",
                point_type, osm_id, dist,
            )
            skipped_points.append(point)
            continue

        log.info(
            "%s osm_id=%s: projecting [%.6f, %.6f] → foot [%.6f, %.6f] (%.1f m)",
            point_type, osm_id,
            pt_coord[0], pt_coord[1],
            foot[0], foot[1],
            dist,
        )

        # Split the target way at the foot point.
        original = way_features[wi]
        coords = original["geometry"]["coordinates"]
        props = original["properties"]

        part_a_coords = coords[: si + 1] + [foot]
        part_b_coords = [foot] + coords[si + 1 :]

        way_a = {
            "type": "Feature",
            "geometry": {"type": "LineString", "coordinates": part_a_coords},
            "properties": dict(props),
        }
        way_b = {
            "type": "Feature",
            "geometry": {"type": "LineString", "coordinates": part_b_coords},
            "properties": dict(props),
        }

        # Replace the original way with the two halves.
        way_features[wi : wi + 1] = [way_a, way_b]

        connected_points.append(point)
        connectors.append(
            {
                "type": "Feature",
                "geometry": {
                    "type": "LineString",
                    "coordinates": [pt_coord, foot],
                },
                "properties": {
                    "osm_type": "connector",
                    "point_type": point_type,
                    "point_osm_id": osm_id,
                },
            }
        )

    return connected_points, skipped_points, connectors


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Filter pedestrian infrastructure from an OSM file and write GeoJSON."
    )
    parser.add_argument(
        "-i",
        "--input",
        required=True,
        metavar="FILE",
        help="Input OSM file (.osm, .osm.pbf, .osm.bz2, …)",
    )
    parser.add_argument(
        "-o",
        "--output",
        required=True,
        metavar="FILE",
        help="Output GeoJSON file",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    input_path = Path(args.input)
    output_path = Path(args.output)
    if not input_path.is_absolute():
        input_path = (Path(os.getenv("BUILD_WORKING_DIRECTORY", "./")) / input_path).resolve()
    if not output_path.is_absolute():
        output_path = (Path(os.getenv("BUILD_WORKING_DIRECTORY", "./")) / output_path).resolve()

    handler = PedestrianHandler()
    handler.apply_file(input_path, locations=True)
    log.info("Collected %d pedestrian way features.", len(handler.way_features))
    log.info("Collected %d dark store nodes.", len(handler.store_features))
    log.info("Collected %d building entrance nodes.", len(handler.entrance_features))

    # Work on a mutable copy — connect_points_to_network splits ways in place.
    way_features = largest_connected_component(handler.way_features)

    connected_stores, skipped_stores, store_connectors = connect_points_to_network(
        way_features, handler.store_features, "base_point",
    )
    if skipped_stores:
        log.warning("%d store node(s) could not be connected and were excluded.", len(skipped_stores))

    connected_entrances, skipped_entrances, entrance_connectors = connect_points_to_network(
        way_features, handler.entrance_features, "delivery_point",
    )
    if skipped_entrances:
        log.warning(
            "%d entrance node(s) could not be connected and were excluded.", len(skipped_entrances)
        )

    all_features = (
        way_features
        + connected_stores
        + store_connectors
        + connected_entrances
        + entrance_connectors
    )

    feature_collection = {
        "type": "FeatureCollection",
        "features": all_features,
    }

    with open(output_path, "w", encoding="utf-8") as fh:
        json.dump(feature_collection, fh, ensure_ascii=False, indent=2)

    log.info(
        "Written %d features to %s "
        "(%d ways, %d stores, %d store connectors, %d entrances, %d entrance connectors)",
        len(all_features), args.output,
        len(way_features),
        len(connected_stores), len(store_connectors),
        len(connected_entrances), len(entrance_connectors),
    )


if __name__ == "__main__":
    main()
