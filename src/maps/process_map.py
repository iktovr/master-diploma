"""Filter pedestrian infrastructure from an OSM file and export as GeoJSON.

Usage:
    # From a local OSM file:
    bazel run //maps:process_map -- --input city.osm --output pedestrian.geojson

    # Download OSM data via Overpass API for a polygon area:
    bazel run //maps:process_map -- --location area.geojson --output pedestrian.geojson

    # Local file filtered to a polygon:
    bazel run //maps:process_map -- --input city.osm --location area.geojson --output pedestrian.geojson

    # With extra base points from a GeoJSON file:
    bazel run //maps:process_map -- --input city.osm --basepoints stores.geojson --output pedestrian.geojson

The --location file must be a GeoJSON file containing a single Polygon feature.
When --input is omitted and --location is provided, OSM data is automatically
downloaded from the Overpass API (https://overpass-api.de) for the polygon's
bounding box.
"""

import argparse
import json
import logging
import math
import os
import tempfile
import urllib.parse
import urllib.request
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
# Constants
# ---------------------------------------------------------------------------

# --- OSM tag filters ---

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

FOOT_ACCESS_VALUES = frozenset(["yes", "designated", "permissive"])

ENTRANCE_VALUES = frozenset(["yes", "main", "staircase", "home"])

BARRIER_VALUES = frozenset(["wall", "fence", "kerb"])

BASE_POINT_SHOP_VALUES = frozenset(["supermarket"])

BASE_POINT_DISUSED_KEYS = frozenset([
    "disused:shop", "disused:amenity",
    "was:shop", "was:amenity",
    "construction:shop", "construction:amenity",
    "abandoned:shop", "abandoned:amenity",
])

# Higher = preferred when two base points fall within MIN_BASE_POINT_DISTANCE_M.
BASE_POINT_PRIORITY = {
    "dark_store": 3,
    "shop:supermarket": 2,
    "shop:convenience": 1,
}

# Minimum spacing between any two accepted base points (metres).
MIN_BASE_POINT_DISTANCE_M = 150.0

# Minimum spacing between any two accepted delivery points (metres).
# Entrances of the same building often sit a few metres apart; collapse only
# near-duplicates.
MIN_DELIVERY_POINT_DISTANCE_M = 20.0

# Higher = preferred when two delivery points fall within
# MIN_DELIVERY_POINT_DISTANCE_M. Falls back to 0 for unknown values.
DELIVERY_POINT_PRIORITY = {
    "main": 3,
    "yes": 2,
    "staircase": 1,
    "home": 1,
}

# --- Geometry / simplification thresholds (metres) ---

# Maximum perpendicular deviation allowed when simplifying a polyline
# segment via Ramer–Douglas–Peucker.
MAX_SIMPLIFY_DEVIATION_M = 5.0

# Minimum length for a segment to be kept; shorter segments are collapsed
# during RDP and removed (with endpoint merging) after simplification.
MIN_SEGMENT_LENGTH_M = 0.25

# --- Point-to-network connection thresholds (metres) ---

MAX_CONNECT_DISTANCE_M = 20.0
MIN_CONNECT_DISTANCE_M = 0.3

# --- Building-crossing check ---

# Fraction of a connector's own length that may lie inside a building
# before the connection is rejected.
MAX_BUILDING_THROUGH_FRACTION = 0.5

# Connectors shorter than this (metres) are never checked against buildings.
MIN_CONNECTOR_LENGTH_FOR_BUILDING_CHECK_M = 10.0

# --- Narrow-passage detection (metres) ---

# Clearance threshold below which a passage is considered narrow.
NARROW_WIDTH_M = 3.0

# Narrow runs shorter than this are ignored as noise.
NARROW_MIN_LENGTH_M = 1.0

# Narrow runs longer than this are ignored (whole corridor is narrow,
# not a bottleneck worth tagging).
NARROW_MAX_LENGTH_M = 50.0

# --- Visualisation colours (simplestyle-spec #rrggbb) ---

COLOR_NARROW = "#ff8800"
COLOR_BASE_POINT = "#ff0000"
COLOR_DELIVERY_POINT = "#3388ff"


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


def _base_point_subtype(tags) -> str | None:
    """Return base-point subtype for *tags*, or ``None`` if not a base point."""
    if any(k in tags for k in BASE_POINT_DISUSED_KEYS):
        return None
    if tags.get("dark_store") == "yes":
        return "dark_store"
    if tags.get("shop") in BASE_POINT_SHOP_VALUES:
        if tags.get("name") or tags.get("name:ru"):
            return "shop:" + tags.get("shop")
    return None


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
        # Closed polygons for every OSM building way (lon/lat rings).
        self.building_polygons: list[list[list[float]]] = []
        # Open polylines for every OSM barrier way (wall/fence/kerb).
        self.barrier_ways: list[list[list[float]]] = []

    def node(self, n):
        subtype = _base_point_subtype(n.tags)
        if subtype is not None:
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
                        "subtype": subtype,
                        **_tags_to_dict(n.tags),
                    },
                }
            )
            return

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
        if w.tags.get("building"):
            try:
                coords = [[n.lon, n.lat] for n in w.nodes]
            except osmium.InvalidLocationError:
                coords = []
            # A valid closed polygon needs at least 4 nodes (3 unique + repeat).
            if len(coords) >= 4:
                self.building_polygons.append(coords)

        if w.tags.get("barrier") in BARRIER_VALUES:
            try:
                coords = [[n.lon, n.lat] for n in w.nodes]
            except osmium.InvalidLocationError:
                coords = []
            if len(coords) >= 2:
                self.barrier_ways.append(coords)

        if not _is_pedestrian_way(w.tags):
            return

        try:
            coords = [[n.lon, n.lat] for n in w.nodes]
        except osmium.InvalidLocationError:
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
                    "narrow": "no",
                    **_tags_to_dict(w.tags),
                },
            }
        )


# ---------------------------------------------------------------------------
# Geometry helpers — UTM projection for metric calculations
# ---------------------------------------------------------------------------


def _coord_key(coord: list) -> tuple:
    """Round to ~1 cm precision to merge near-duplicate endpoints."""
    return (round(coord[0], 7), round(coord[1], 7))


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
    """Foot of the perpendicular from *p* onto segment *a*–*b* (UTM, clamped)."""
    ax, ay = a_xy
    bx, by = b_xy
    px, py = p_xy

    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy

    if seg_len_sq == 0.0:
        return a_xy

    t = ((px - ax) * dx + (py - ay) * dy) / seg_len_sq
    t_clamped = max(0.0, min(1.0, t))
    return (ax + t_clamped * dx, ay + t_clamped * dy)


def _nearest_segment_projection(
    point: list, way_features: list[dict]
) -> tuple[list, int, int, float] | None:
    """Find the nearest perpendicular projection of *point* onto any segment.

    The returned foot is snapped to the nearest existing way-node if that node
    is within ``MIN_SEGMENT_LENGTH_M`` metres of the raw foot, to avoid
    near-zero-length sub-segments and T-junction violations.

    Returns (foot_lonlat, way_index, seg_index, distance_m) or ``None``.
    """
    proj = _utm_proj(point[0], point[1])
    p_xy = _to_utm(proj, point)

    best_dist = math.inf
    best_foot_xy: tuple[float, float] | None = None
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
                best_foot_xy = foot_xy
                best_way_idx = wi
                best_seg_idx = si

    if best_foot_xy is None:
        return None

    snapped_foot_lonlat: list | None = None
    snap_threshold = MIN_SEGMENT_LENGTH_M
    for feat in way_features:
        for node_lonlat in feat["geometry"]["coordinates"]:
            node_xy = _to_utm(proj, node_lonlat)
            d = math.hypot(best_foot_xy[0] - node_xy[0], best_foot_xy[1] - node_xy[1])
            if d < snap_threshold:
                snap_threshold = d
                snapped_foot_lonlat = node_lonlat

    if snapped_foot_lonlat is not None:
        best_foot_lonlat = snapped_foot_lonlat
    else:
        best_foot_lonlat = _from_utm(proj, best_foot_xy)

    return best_foot_lonlat, best_way_idx, best_seg_idx, best_dist


# ---------------------------------------------------------------------------
# Connectivity filter — keep only the largest connected component
# ---------------------------------------------------------------------------


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
# Planarization — split polylines at every interior touch / crossing
# ---------------------------------------------------------------------------


def _seg_seg_intersection_t(
    a: tuple, b: tuple, c: tuple, d: tuple
) -> tuple[float, float] | None:
    """Parametric intersection of segment *a*–*b* with segment *c*–*d* in 2-D.

    Returns (t, u) ∈ [0, 1]² such that ``a + t·(b-a) == c + u·(d-c)``,
    or ``None`` when the segments are parallel/collinear or do not intersect.
    """
    ax, ay = a
    bx, by = b
    cx, cy = c
    dx, dy = d

    denom = (bx - ax) * (dy - cy) - (by - ay) * (dx - cx)
    if abs(denom) < 1e-10:
        return None

    t = ((cx - ax) * (dy - cy) - (cy - ay) * (dx - cx)) / denom
    u = ((cx - ax) * (by - ay) - (cy - ay) * (bx - ax)) / denom

    if -1e-9 <= t <= 1 + 1e-9 and -1e-9 <= u <= 1 + 1e-9:
        return (max(0.0, min(1.0, t)), max(0.0, min(1.0, u)))
    return None


def planarize_ways(features: list[dict]) -> list[dict]:
    """Split every polyline at points where another polyline touches or
    crosses it, so that resulting LineStrings only meet at endpoints.

    Handles vertex junctions (interior vertex of way *i* is also a vertex of
    way *j*), T-junctions (a node of *j* on a segment of *i*) and proper
    X-crossings.  All coordinate comparisons use ``_coord_key``.
    """
    if not features:
        return features

    in_count = len(features)

    # Use the first dataset coordinate as the single projection origin so all
    # ways share the same metric space (avoids zone-boundary artefacts).
    ref_lon = features[0]["geometry"]["coordinates"][0][0]
    ref_lat = features[0]["geometry"]["coordinates"][0][1]
    proj = _utm_proj(ref_lon, ref_lat)

    utm_coords: list[list[tuple[float, float]]] = [
        [_to_utm(proj, c) for c in feat["geometry"]["coordinates"]]
        for feat in features
    ]

    n = len(features)

    # vertex_owners[key] = set of way indices that have a vertex at that key.
    vertex_owners: dict[tuple, set[int]] = {}
    for i, feat in enumerate(features):
        for c in feat["geometry"]["coordinates"]:
            vertex_owners.setdefault(_coord_key(c), set()).add(i)

    # split_vertex[i] = interior vertex indices of way i that become split
    # points (case A: shared vertex with another way).
    split_vertex: list[set[int]] = [set() for _ in range(n)]

    # split_params[i][seg_idx] = list of t values ∈ (0, 1) (cases B & C).
    split_params: list[list[list[float]]] = [
        [[] for _ in range(len(pts) - 1)]
        for pts in utm_coords
    ]

    # Case A: interior vertex of way i is also a vertex of some OTHER way.
    for i in range(n):
        coords_i = features[i]["geometry"]["coordinates"]
        for vi in range(1, len(coords_i) - 1):
            owners = vertex_owners.get(_coord_key(coords_i[vi]), ())
            if any(o != i for o in owners):
                split_vertex[i].add(vi)

    # Cases B & C: segment-level intersections.
    for i in range(n):
        pts_i = utm_coords[i]
        coords_i = features[i]["geometry"]["coordinates"]
        for j in range(n):
            if i == j:
                continue
            pts_j = utm_coords[j]
            coords_j = features[j]["geometry"]["coordinates"]

            # B: node of way j sitting on a segment of way i.
            for node_lonlat in coords_j:
                node_key = _coord_key(node_lonlat)
                if node_key == _coord_key(coords_i[0]) or node_key == _coord_key(coords_i[-1]):
                    continue
                node_utm = _to_utm(proj, node_lonlat)
                nx_, ny_ = node_utm
                for si in range(len(pts_i) - 1):
                    ax, ay = pts_i[si]
                    bx, by = pts_i[si + 1]
                    dx, dy = bx - ax, by - ay
                    seg_len_sq = dx * dx + dy * dy
                    if seg_len_sq < 1e-12:
                        continue
                    t = ((nx_ - ax) * dx + (ny_ - ay) * dy) / seg_len_sq
                    if t <= 1e-9 or t >= 1 - 1e-9:
                        continue
                    foot_x = ax + t * dx
                    foot_y = ay + t * dy
                    dist = math.hypot(nx_ - foot_x, ny_ - foot_y)
                    if dist < 0.01:
                        split_params[i][si].append(t)

            # C: proper crossing of segments.
            for si in range(len(pts_i) - 1):
                a_utm = pts_i[si]
                b_utm = pts_i[si + 1]
                for sj in range(len(pts_j) - 1):
                    c_utm = pts_j[sj]
                    d_utm = pts_j[sj + 1]
                    isect = _seg_seg_intersection_t(a_utm, b_utm, c_utm, d_utm)
                    if isect is None:
                        continue
                    t_isect, _u_isect = isect
                    if 1e-9 < t_isect < 1 - 1e-9:
                        split_params[i][si].append(t_isect)

    # Emit sub-polylines: for each way walk vertices and emit on every split.
    result: list[dict] = []
    for i, feat in enumerate(features):
        coords = feat["geometry"]["coordinates"]
        props = feat["properties"]
        pts = utm_coords[i]

        param_splits: list[tuple[int, float, list]] = []
        for si, ts in enumerate(split_params[i]):
            for t in ts:
                ax, ay = pts[si]
                bx, by = pts[si + 1]
                ix = ax + t * (bx - ax)
                iy = ay + t * (by - ay)
                lonlat = _from_utm(proj, (ix, iy))
                param_splits.append((si, t, lonlat))
        param_splits.sort(key=lambda x: (x[0], x[1]))

        # Deduplicate parametric splits within 1 cm.
        deduped_param: list[tuple[int, float, list]] = []
        for sp in param_splits:
            if deduped_param:
                prev = deduped_param[-1]
                ax, ay = pts[sp[0]]; bx, by = pts[sp[0] + 1]
                ix = ax + sp[1] * (bx - ax); iy = ay + sp[1] * (by - ay)
                pax, pay = pts[prev[0]]; pbx, pby = pts[prev[0] + 1]
                pix = pax + prev[1] * (pbx - pax); piy = pay + prev[1] * (pby - pay)
                if math.hypot(ix - pix, iy - piy) < 0.01:
                    continue
            deduped_param.append(sp)

        param_by_seg: dict[int, list[tuple[float, list]]] = {}
        for si, t, lonlat in deduped_param:
            param_by_seg.setdefault(si, []).append((t, lonlat))

        sv = split_vertex[i]
        if not sv and not param_by_seg:
            result.append(feat)
            continue

        current: list[list] = [coords[0]]

        for vi in range(1, len(coords)):
            for t, lonlat in param_by_seg.get(vi - 1, []):
                current.append(lonlat)
                if len(current) >= 2:
                    result.append({
                        "type": "Feature",
                        "geometry": {"type": "LineString", "coordinates": current},
                        "properties": dict(props),
                    })
                current = [lonlat]

            current.append(coords[vi])

            if vi in sv and vi < len(coords) - 1:
                if len(current) >= 2:
                    result.append({
                        "type": "Feature",
                        "geometry": {"type": "LineString", "coordinates": current},
                        "properties": dict(props),
                    })
                current = [coords[vi]]

        if len(current) >= 2:
            result.append({
                "type": "Feature",
                "geometry": {"type": "LineString", "coordinates": current},
                "properties": dict(props),
            })

    out_count = len(result)
    log.info(
        "Planarize ways: %d polylines → %d polylines after splitting at junctions.",
        in_count,
        out_count,
    )
    return result


# ---------------------------------------------------------------------------
# Polyline simplification — Ramer–Douglas–Peucker, output: 2-point segments
# ---------------------------------------------------------------------------


def _perp_distance_utm(p: tuple, a: tuple, b: tuple) -> float:
    """Perpendicular distance from *p* to the infinite line through *a*–*b* (UTM)."""
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    seg_len = math.hypot(dx, dy)
    if seg_len == 0.0:
        return math.hypot(px - ax, py - ay)
    return abs(dx * (ay - py) - dy * (ax - px)) / seg_len


def _rdp_split(
    pts: list[tuple[float, float]],
    tolerance_m: float,
    min_length_m: float = MIN_SEGMENT_LENGTH_M,
) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    """Recursively simplify a polyline via Ramer–Douglas–Peucker.

    Returns a list of 2-point (straight) segments such that every output
    deviates from the original by at most *tolerance_m* metres.  Segments
    whose chord length is below *min_length_m* are dropped.
    """
    if len(pts) == 2:
        chord = math.hypot(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1])
        if chord < min_length_m:
            return []
        return [(pts[0], pts[1])]

    a, b = pts[0], pts[-1]

    chord = math.hypot(b[0] - a[0], b[1] - a[1])
    if chord < min_length_m:
        return []

    max_dist = 0.0
    max_idx = 1
    for i in range(1, len(pts) - 1):
        d = _perp_distance_utm(pts[i], a, b)
        if d > max_dist:
            max_dist = d
            max_idx = i

    if max_dist <= tolerance_m:
        return [(a, b)]

    left = _rdp_split(pts[: max_idx + 1], tolerance_m, min_length_m)
    right = _rdp_split(pts[max_idx:], tolerance_m, min_length_m)
    return left + right


def simplify_ways(
    features: list[dict],
    tolerance_m: float = MAX_SIMPLIFY_DEVIATION_M,
) -> list[dict]:
    """Simplify every LineString so that each output feature has exactly
    2 coordinates.

    Straight polylines (within *tolerance_m*) collapse to a single segment;
    bent polylines are split at points of maximum deviation (RDP).
    """
    if not features:
        return features

    result: list[dict] = []
    in_count = len(features)

    for feat in features:
        coords = feat["geometry"]["coordinates"]
        props = feat["properties"]

        if len(coords) < 2:
            continue

        if len(coords) == 2:
            result.append(feat)
            continue

        proj = _utm_proj(coords[0][0], coords[0][1])
        pts_utm = [_to_utm(proj, c) for c in coords]

        # Preserve the polyline's own endpoints exactly (avoid UTM round-trip
        # floating-point noise of ~1e-14).
        orig_lonlat: dict[int, list] = {
            id(pts_utm[0]): coords[0],
            id(pts_utm[-1]): coords[-1],
        }

        segments = _rdp_split(pts_utm, tolerance_m)

        for a_utm, b_utm in segments:
            a_lonlat = orig_lonlat.get(id(a_utm)) or _from_utm(proj, a_utm)
            b_lonlat = orig_lonlat.get(id(b_utm)) or _from_utm(proj, b_utm)
            result.append(
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "LineString",
                        "coordinates": [a_lonlat, b_lonlat],
                    },
                    "properties": dict(props),
                }
            )

    out_count = len(result)
    log.info(
        "Simplify ways: %d polylines → %d 2-point segments (×%.2f, tolerance=%.1f m).",
        in_count,
        out_count,
        out_count / in_count if in_count else 1.0,
        tolerance_m,
    )
    return result


# ---------------------------------------------------------------------------
# Short-segment removal — contract edges shorter than MIN_SEGMENT_LENGTH_M
# ---------------------------------------------------------------------------


def _lonlat_distance_m(a: list, b: list) -> float:
    """Approximate great-circle distance in metres (flat-Earth, < ~10 km)."""
    lat_m = 111_320.0
    lon_m = lat_m * math.cos(math.radians((a[1] + b[1]) / 2))
    return math.hypot((b[0] - a[0]) * lon_m, (b[1] - a[1]) * lat_m)


def remove_short_segments(
    features: list[dict],
    min_length_m: float = MIN_SEGMENT_LENGTH_M,
) -> list[dict]:
    """Contract 2-point segments shorter than *min_length_m* by merging their
    endpoints into the midpoint and updating every other segment that
    referenced either endpoint.  Shortest-first to minimise distortion.
    """
    if not features:
        return features

    coords_list: list[list[list] | None] = [
        list(feat["geometry"]["coordinates"]) for feat in features
    ]
    props_list = [feat["properties"] for feat in features]

    in_count = len(features)
    removed = 0

    changed = True
    while changed:
        changed = False

        best_idx = -1
        best_len = min_length_m

        for i, coords in enumerate(coords_list):
            if coords is None:
                continue
            d = _lonlat_distance_m(coords[0], coords[1])
            if d < best_len:
                best_len = d
                best_idx = i

        if best_idx == -1:
            break

        seg = coords_list[best_idx]
        assert seg is not None
        a = seg[0]
        b = seg[1]
        mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2]

        a_key = _coord_key(a)
        b_key = _coord_key(b)

        for i, coords in enumerate(coords_list):
            if coords is None or i == best_idx:
                continue
            updated = False
            new_coords = []
            for pt in coords:
                k = _coord_key(pt)
                if k == a_key or k == b_key:
                    new_coords.append(mid)
                    updated = True
                else:
                    new_coords.append(pt)
            if updated:
                if _coord_key(new_coords[0]) == _coord_key(new_coords[1]):
                    coords_list[i] = None
                    removed += 1
                else:
                    coords_list[i] = new_coords

        coords_list[best_idx] = None
        removed += 1
        changed = True

    result = [
        {
            "type": "Feature",
            "geometry": {"type": "LineString", "coordinates": coords},
            "properties": dict(props),
        }
        for coords, props in zip(coords_list, props_list)
        if coords is not None
    ]

    out_count = len(result)
    log.info(
        "Remove short segments: %d → %d segments (removed %d shorter than %.2f m).",
        in_count,
        out_count,
        removed,
        min_length_m,
    )
    return result


# ---------------------------------------------------------------------------
# Building-crossing check — reject connectors that traverse most of a building
# ---------------------------------------------------------------------------


def _segment_inside_length_utm(
    a_xy: tuple[float, float],
    b_xy: tuple[float, float],
    ring_xy: list[tuple[float, float]],
) -> float:
    """Length (UTM metres) of segment *a*–*b* lying inside the closed polygon
    *ring_xy* (first == last).  Uses ray-casting on sub-interval midpoints.
    """
    ax, ay = a_xy
    bx, by = b_xy
    seg_len = math.hypot(bx - ax, by - ay)
    if seg_len < 1e-9:
        return 0.0

    t_values: list[float] = [0.0, 1.0]
    n = len(ring_xy)
    for k in range(n - 1):
        cx, cy = ring_xy[k]
        dx, dy = ring_xy[k + 1]
        isect = _seg_seg_intersection_t(a_xy, b_xy, (cx, cy), (dx, dy))
        if isect is not None:
            t, _u = isect
            if 1e-9 < t < 1 - 1e-9:
                t_values.append(t)

    t_values = sorted(set(t_values))

    def _inside(px: float, py: float) -> bool:
        inside = False
        for k in range(n - 1):
            xi, yi = ring_xy[k]
            xj, yj = ring_xy[k + 1]
            if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi + 1e-15) + xi):
                inside = not inside
        return inside

    total = 0.0
    for idx in range(len(t_values) - 1):
        t_mid = (t_values[idx] + t_values[idx + 1]) / 2.0
        mx = ax + t_mid * (bx - ax)
        my = ay + t_mid * (by - ay)
        if _inside(mx, my):
            total += (t_values[idx + 1] - t_values[idx]) * seg_len

    return total


def _connector_blocked_by_building(
    pt_lonlat: list,
    foot_lonlat: list,
    building_polygons: list[list[list[float]]],
    max_through_fraction: float = MAX_BUILDING_THROUGH_FRACTION,
    min_connector_len_m: float = MIN_CONNECTOR_LENGTH_FOR_BUILDING_CHECK_M,
) -> bool:
    """Return True if the connector passes through almost the full length
    of any building.  Short connectors are always allowed.
    """
    if not building_polygons:
        return False

    proj = _utm_proj(pt_lonlat[0], pt_lonlat[1])
    a_xy = _to_utm(proj, pt_lonlat)
    b_xy = _to_utm(proj, foot_lonlat)

    connector_len = math.hypot(b_xy[0] - a_xy[0], b_xy[1] - a_xy[1])

    if connector_len < min_connector_len_m:
        return False

    for ring_lonlat in building_polygons:
        ring_xy: list[tuple[float, float]] = [_to_utm(proj, c) for c in ring_lonlat]
        if ring_xy[0] != ring_xy[-1]:
            ring_xy.append(ring_xy[0])

        # Skip degenerate buildings.
        xs = [p[0] for p in ring_xy]
        ys = [p[1] for p in ring_xy]
        if math.hypot(max(xs) - min(xs), max(ys) - min(ys)) < 1e-3:
            continue

        inside_len = _segment_inside_length_utm(a_xy, b_xy, ring_xy)
        if inside_len / connector_len > max_through_fraction:
            return True

    return False


# ---------------------------------------------------------------------------
# Narrow-passage detection
# ---------------------------------------------------------------------------


def _point_to_segment_distance_utm(
    p_xy: tuple[float, float],
    a_xy: tuple[float, float],
    b_xy: tuple[float, float],
) -> float:
    """Distance from *p* to segment *a*–*b* in UTM metres (clamped to segment)."""
    ax, ay = a_xy
    bx, by = b_xy
    px, py = p_xy
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-20:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / seg_len_sq
    t = max(0.0, min(1.0, t))
    foot_x = ax + t * dx
    foot_y = ay + t * dy
    return math.hypot(px - foot_x, py - foot_y)


def _min_clearance_to_obstacles(
    p_xy: tuple[float, float],
    building_edges_xy: list[tuple[tuple[float, float], tuple[float, float]]],
    barrier_segs_xy: list[tuple[tuple[float, float], tuple[float, float]]],
    cap: float,
) -> float:
    """Minimum distance from *p_xy* to any obstacle edge, capped at *cap*."""
    best = cap
    px, py = p_xy
    for a_xy, b_xy in building_edges_xy:
        ax, ay = a_xy
        bx, by = b_xy
        if min(ax, bx) - best > px or px > max(ax, bx) + best:
            if min(ay, by) - best > py or py > max(ay, by) + best:
                continue
        d = _point_to_segment_distance_utm(p_xy, a_xy, b_xy)
        if d < best:
            best = d
    for a_xy, b_xy in barrier_segs_xy:
        ax, ay = a_xy
        bx, by = b_xy
        if min(ax, bx) - best > px or px > max(ax, bx) + best:
            if min(ay, by) - best > py or py > max(ay, by) + best:
                continue
        d = _point_to_segment_distance_utm(p_xy, a_xy, b_xy)
        if d < best:
            best = d
    return best


def tag_narrow_way_segments(
    way_features: list[dict],
    building_polygons: list[list[list[float]]],
    barrier_ways: list[list[list[float]]],
    narrow_width_m: float = NARROW_WIDTH_M,
    narrow_min_m: float = NARROW_MIN_LENGTH_M,
    narrow_max_m: float = NARROW_MAX_LENGTH_M,
) -> list[dict]:
    """Tag pedestrian ways passing through narrow spots with ``narrow=yes``.

    For each way, measure clearance at every node to the nearest obstacle
    (building edge or barrier segment).  Consecutive nodes below
    *narrow_width_m* form a *narrow run*; the run qualifies when
    ``narrow_min_m <= arc-length <= narrow_max_m``.  Tagging happens before
    planarize/simplify so the tag propagates to derived 2-point segments.
    """
    if not way_features:
        return way_features
    if not building_polygons and not barrier_ways:
        log.info("Narrow detection: no buildings or barriers — skipping.")
        return way_features

    ref_lon = way_features[0]["geometry"]["coordinates"][0][0]
    ref_lat = way_features[0]["geometry"]["coordinates"][0][1]
    proj = _utm_proj(ref_lon, ref_lat)

    building_edges_xy: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for ring in building_polygons:
        ring_xy = [_to_utm(proj, c) for c in ring]
        for k in range(len(ring_xy) - 1):
            building_edges_xy.append((ring_xy[k], ring_xy[k + 1]))

    barrier_segs_xy: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for bw in barrier_ways:
        bw_xy = [_to_utm(proj, c) for c in bw]
        for k in range(len(bw_xy) - 1):
            barrier_segs_xy.append((bw_xy[k], bw_xy[k + 1]))

    log.info(
        "Narrow detection: %d building edges, %d barrier segments, "
        "threshold=%.2f m, min_run=%.2f m, max_run=%.2f m.",
        len(building_edges_xy), len(barrier_segs_xy),
        narrow_width_m, narrow_min_m, narrow_max_m,
    )

    cap = narrow_width_m

    ways_tagged = 0
    runs_total = 0
    runs_kept = 0

    for feat in way_features:
        coords = feat["geometry"]["coordinates"]
        if len(coords) < 2:
            continue

        pts_xy = [_to_utm(proj, c) for c in coords]
        clearances = [
            _min_clearance_to_obstacles(p, building_edges_xy, barrier_segs_xy, cap)
            for p in pts_xy
        ]

        has_qualifying_run = False
        i = 0
        n = len(pts_xy)
        while i < n:
            if clearances[i] < narrow_width_m:
                j = i
                while j + 1 < n and clearances[j + 1] < narrow_width_m:
                    j += 1
                run_len = 0.0
                for k in range(i, j):
                    ax, ay = pts_xy[k]
                    bx, by = pts_xy[k + 1]
                    run_len += math.hypot(bx - ax, by - ay)
                runs_total += 1
                if narrow_min_m <= run_len <= narrow_max_m:
                    has_qualifying_run = True
                    runs_kept += 1
                i = j + 1
            else:
                i += 1

        if has_qualifying_run:
            feat["properties"]["narrow"] = "yes"
            ways_tagged += 1

    log.info(
        "Narrow detection: %d way(s) tagged narrow; %d run(s) found, %d kept "
        "(others outside [%.1f, %.1f] m).",
        ways_tagged, runs_total, runs_kept, narrow_min_m, narrow_max_m,
    )
    return way_features


def thin_narrow_segments(features: list[dict]) -> list[dict]:
    """Thin out clusters of touching ``narrow=yes`` segments so that no two
    narrow segments share an endpoint.

    Greedily unmarks the shortest narrow segment that still touches another
    narrow segment, keeping the longest (most "obvious") narrow spots tagged.
    Geometry is not modified.
    """
    if not features:
        return features

    narrow_idx: list[int] = [
        i for i, f in enumerate(features)
        if f["properties"].get("narrow") == "yes"
        and f["geometry"]["type"] == "LineString"
        and len(f["geometry"]["coordinates"]) == 2
    ]

    if len(narrow_idx) < 2:
        return features

    endpoint_keys: dict[int, tuple[tuple, tuple]] = {}
    lengths: dict[int, float] = {}
    for i in narrow_idx:
        coords = features[i]["geometry"]["coordinates"]
        a_key = _coord_key(coords[0])
        b_key = _coord_key(coords[1])
        endpoint_keys[i] = (a_key, b_key)
        lengths[i] = _lonlat_distance_m(coords[0], coords[1])

    by_endpoint: dict[tuple, set[int]] = {}
    for i in narrow_idx:
        a_key, b_key = endpoint_keys[i]
        by_endpoint.setdefault(a_key, set()).add(i)
        by_endpoint.setdefault(b_key, set()).add(i)

    narrow_neighbours: dict[int, set[int]] = {i: set() for i in narrow_idx}
    for ep, members in by_endpoint.items():
        if len(members) < 2:
            continue
        members_list = list(members)
        for a in members_list:
            for b in members_list:
                if a != b:
                    narrow_neighbours[a].add(b)

    active: set[int] = set(narrow_idx)
    unmarked = 0

    while True:
        worst_idx = -1
        worst_len = math.inf
        for i in active:
            if narrow_neighbours[i] & active:
                if lengths[i] < worst_len:
                    worst_len = lengths[i]
                    worst_idx = i

        if worst_idx == -1:
            break

        active.discard(worst_idx)
        props = features[worst_idx]["properties"]
        if "narrow" in props:
            props["narrow"] = "no"
        unmarked += 1

    kept = len(active)
    log.info(
        "Thin narrow segments: %d narrow → %d isolated narrow (unmarked %d "
        "touching segments, shortest-first).",
        len(narrow_idx), kept, unmarked,
    )
    return features


# ---------------------------------------------------------------------------
# Connect point nodes to the nearest edge
# ---------------------------------------------------------------------------


def connect_points_to_network(
    way_features: list[dict],
    point_features: list[dict],
    point_type: str,
    max_distance_m: float = MAX_CONNECT_DISTANCE_M,
    min_distance_m: float = MIN_CONNECT_DISTANCE_M,
    building_polygons: list[list[list[float]]] | None = None,
) -> tuple[list[dict], list[dict], list[dict]]:
    """Connect each point to the nearest way segment, splitting it at the foot.

    Returns ``(connected_points, skipped_points, connectors)``. *way_features*
    is modified in place: split ways replace the original.
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
            log.debug(
                "%s osm_id=%s: nearest segment is %.1f m away, skipping.",
                point_type, osm_id, dist,
            )
            skipped_points.append(point)
            continue

        log.debug(
            "%s osm_id=%s: projecting [%.6f, %.6f] → foot [%.6f, %.6f] (%.1f m)",
            point_type, osm_id,
            pt_coord[0], pt_coord[1],
            foot[0], foot[1],
            dist,
        )

        # Snap foot to a segment endpoint if it falls too close to one, to
        # avoid creating a degenerate near-zero-length sub-segment.
        original = way_features[wi]
        coords = original["geometry"]["coordinates"]
        props = original["properties"]

        proj_check = _utm_proj(foot[0], foot[1])
        foot_utm = _to_utm(proj_check, foot)
        seg_a_utm = _to_utm(proj_check, coords[si])
        seg_b_utm = _to_utm(proj_check, coords[si + 1])
        dist_to_a = math.hypot(foot_utm[0] - seg_a_utm[0], foot_utm[1] - seg_a_utm[1])
        dist_to_b = math.hypot(foot_utm[0] - seg_b_utm[0], foot_utm[1] - seg_b_utm[1])

        if dist_to_a < min_distance_m:
            foot = coords[si]
        elif dist_to_b < min_distance_m:
            foot = coords[si + 1]

        proj_conn = _utm_proj(pt_coord[0], pt_coord[1])
        pt_utm = _to_utm(proj_conn, pt_coord)
        foot_utm2 = _to_utm(proj_conn, foot)
        conn_len = math.hypot(pt_utm[0] - foot_utm2[0], pt_utm[1] - foot_utm2[1])
        if conn_len < min_distance_m:
            log.debug(
                "%s osm_id=%s: connector length %.3f m < %.3f m after snap, skipping.",
                point_type, osm_id, conn_len, min_distance_m,
            )
            skipped_points.append(point)
            continue

        # Reject delivery_point connectors that pass through a building.
        if point_type == "delivery_point" and building_polygons:
            if _connector_blocked_by_building(pt_coord, foot, building_polygons):
                log.debug(
                    "%s osm_id=%s: connector blocked — passes through most of a building, skipping.",
                    point_type, osm_id,
                )
                skipped_points.append(point)
                continue

        # Only split if the foot is strictly interior to the segment.
        foot_key = _coord_key(foot)
        seg_a_key = _coord_key(coords[si])
        seg_b_key = _coord_key(coords[si + 1])

        if foot_key == seg_a_key or foot_key == seg_b_key:
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
                        "point_osm_id": osm_id,
                    },
                }
            )
            continue

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
                    "point_osm_id": osm_id,
                },
            }
        )

    return connected_points, skipped_points, connectors


# ---------------------------------------------------------------------------
# Final cleanup — eliminate remaining T-junctions / near-touch X-crossings
# ---------------------------------------------------------------------------


def fix_interior_violations(
    features: list[dict],
    tol_m: float = 0.5,
    min_len_m: float = MIN_SEGMENT_LENGTH_M,
    max_iterations: int = 20,
) -> list[dict]:
    """Eliminate residual T-junctions and near-touch X-crossings.

    For each 2-point segment, finds any foreign endpoint within *tol_m*
    perpendicular distance of its interior.  If closer than *min_len_m* to one
    of the segment's endpoints the foreign endpoint is **snapped** to that
    endpoint (all features referencing the old coordinate are updated);
    otherwise the segment is **split** at the foreign endpoint's coordinate.
    Iterates until stable or *max_iterations* is reached.
    """
    def _line_indices() -> list[int]:
        return [
            i for i, f in enumerate(features)
            if f is not None
            and f["geometry"]["type"] == "LineString"
            and len(f["geometry"]["coordinates"]) == 2
        ]

    lidxs = _line_indices()
    if not lidxs:
        return features

    ref_lon, ref_lat = features[lidxs[0]]["geometry"]["coordinates"][0]
    proj = _utm_proj(ref_lon, ref_lat)

    total_snaps = 0
    total_splits = 0

    for _ in range(max_iterations):
        lidxs = _line_indices()
        if not lidxs:
            break

        endpoints: set[tuple] = set()
        for i in lidxs:
            coords = features[i]["geometry"]["coordinates"]
            endpoints.add(tuple(coords[0]))
            endpoints.add(tuple(coords[1]))
        ep_utm: dict[tuple, tuple[float, float]] = {
            ep: _to_utm(proj, list(ep)) for ep in endpoints
        }

        # Pass 1: collect snap pairs.
        snap_pairs: dict[tuple, tuple] = {}

        for i in lidxs:
            coords = features[i]["geometry"]["coordinates"]
            ta, tb = tuple(coords[0]), tuple(coords[1])
            a_xy = ep_utm[ta]; b_xy = ep_utm[tb]
            dx, dy = b_xy[0] - a_xy[0], b_xy[1] - a_xy[1]
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq < 1e-12:
                continue
            seg_len = math.sqrt(seg_len_sq)

            for ep in endpoints:
                if ep == ta or ep == tb or ep in snap_pairs:
                    continue
                ep_xy = ep_utm[ep]
                t = ((ep_xy[0] - a_xy[0]) * dx + (ep_xy[1] - a_xy[1]) * dy) / seg_len_sq
                if t <= 0 or t >= 1:
                    continue
                foot_x = a_xy[0] + t * dx
                foot_y = a_xy[1] + t * dy
                if math.hypot(ep_xy[0] - foot_x, ep_xy[1] - foot_y) > tol_m:
                    continue
                if t * seg_len < min_len_m:
                    snap_pairs[ep] = ta
                elif (1 - t) * seg_len < min_len_m:
                    snap_pairs[ep] = tb

        if snap_pairs:
            # Resolve chains a -> b -> c into a -> c.
            def _resolve(ep: tuple) -> tuple:
                seen = {ep}
                while ep in snap_pairs and snap_pairs[ep] != ep:
                    ep = snap_pairs[ep]
                    if ep in seen:
                        break
                    seen.add(ep)
                return ep

            for k in list(snap_pairs.keys()):
                snap_pairs[k] = _resolve(k)

            for f in features:
                if f is None:
                    continue
                geom = f["geometry"]
                if geom["type"] == "LineString":
                    new_coords = []
                    for c in geom["coordinates"]:
                        tc = tuple(c)
                        new_coords.append(
                            list(snap_pairs[tc]) if tc in snap_pairs else c
                        )
                    if len(new_coords) == 2 and tuple(new_coords[0]) == tuple(new_coords[1]):
                        f.clear()
                        f["__deleted__"] = True
                    else:
                        geom["coordinates"] = new_coords
                elif geom["type"] == "Point":
                    tc = tuple(geom["coordinates"])
                    if tc in snap_pairs:
                        geom["coordinates"] = list(snap_pairs[tc])

            for idx in range(len(features)):
                if features[idx] is not None and features[idx].get("__deleted__"):
                    features[idx] = None  # type: ignore[call-overload]

            total_snaps += len(snap_pairs)
            continue

        # Pass 2: collect split actions.
        split_actions: list[tuple[int, list[tuple[float, list]]]] = []

        for i in lidxs:
            coords = features[i]["geometry"]["coordinates"]
            ta, tb = tuple(coords[0]), tuple(coords[1])
            a_xy = ep_utm[ta]; b_xy = ep_utm[tb]
            dx, dy = b_xy[0] - a_xy[0], b_xy[1] - a_xy[1]
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq < 1e-12:
                continue
            seg_len = math.sqrt(seg_len_sq)

            this_splits: list[tuple[float, list]] = []
            for ep in endpoints:
                if ep == ta or ep == tb:
                    continue
                ep_xy = ep_utm[ep]
                t = ((ep_xy[0] - a_xy[0]) * dx + (ep_xy[1] - a_xy[1]) * dy) / seg_len_sq
                if t <= 0 or t >= 1:
                    continue
                foot_x = a_xy[0] + t * dx
                foot_y = a_xy[1] + t * dy
                if math.hypot(ep_xy[0] - foot_x, ep_xy[1] - foot_y) > tol_m:
                    continue
                # Skip cases that would create a sub-segment shorter than min_len_m.
                if t * seg_len < min_len_m or (1 - t) * seg_len < min_len_m:
                    continue
                this_splits.append((t, list(ep)))

            if this_splits:
                this_splits.sort(key=lambda x: x[0])
                # Deduplicate split points within min_len_m of each other.
                deduped: list[tuple[float, list]] = []
                for tval, lonlat in this_splits:
                    if deduped:
                        prev_t = deduped[-1][0]
                        if abs(tval - prev_t) * seg_len < min_len_m:
                            continue
                    deduped.append((tval, lonlat))
                split_actions.append((i, deduped))

        if not split_actions:
            break

        for i, sp in split_actions:
            if features[i] is None:
                continue
            coords = features[i]["geometry"]["coordinates"]
            a, b = list(coords[0]), list(coords[1])
            props = features[i]["properties"]
            pts = [a] + [s[1] for s in sp] + [b]
            features[i]["geometry"]["coordinates"] = [pts[0], pts[1]]
            total_splits += 1
            for k in range(1, len(pts) - 1):
                features.append({
                    "type": "Feature",
                    "geometry": {
                        "type": "LineString",
                        "coordinates": [pts[k], pts[k + 1]],
                    },
                    "properties": dict(props),
                })
                total_splits += 1

    features = [f for f in features if f is not None]
    log.info(
        "Fix interior violations: %d snap(s), %d split(s).",
        total_snaps, total_splits,
    )
    return features


# ---------------------------------------------------------------------------
# Validation — sanity-check the final GeoJSON feature collection
# ---------------------------------------------------------------------------


def validate_geojson(features: list[dict], min_length_m: float = MIN_SEGMENT_LENGTH_M) -> bool:
    """Validate the feature collection. Returns True if all checks pass.

    Checks: every LineString has exactly 2 coordinates and length >=
    *min_length_m*; no T-junctions or X-crossings; the graph is connected;
    every Point lies on a LineString endpoint; narrow segments are isolated.
    """
    ok = True

    line_features = [f for f in features if f["geometry"]["type"] == "LineString"]
    point_features = [f for f in features if f["geometry"]["type"] == "Point"]

    # --- 1. Every LineString has exactly 2 coordinates ---
    bad_2pt = [
        i for i, f in enumerate(line_features)
        if len(f["geometry"]["coordinates"]) != 2
    ]
    if bad_2pt:
        log.error(
            "VALIDATION FAIL [2-point]: %d segment(s) have != 2 coordinates: indices %s",
            len(bad_2pt), bad_2pt[:10],
        )
        ok = False
    else:
        log.info(
            "VALIDATION OK  [2-point]: all %d segments have exactly 2 coordinates.",
            len(line_features),
        )

    # --- 2. Minimum length ---
    def _dist_m(a: list, b: list) -> float:
        lat_m = 111_320.0
        lon_m = lat_m * math.cos(math.radians((a[1] + b[1]) / 2))
        return math.hypot((b[0] - a[0]) * lon_m, (b[1] - a[1]) * lat_m)

    two_pt = [f for f in line_features if len(f["geometry"]["coordinates"]) == 2]

    short_segs = [
        (i, _dist_m(f["geometry"]["coordinates"][0], f["geometry"]["coordinates"][1]))
        for i, f in enumerate(two_pt)
        if _dist_m(f["geometry"]["coordinates"][0], f["geometry"]["coordinates"][1]) < min_length_m
    ]
    if short_segs:
        log.error(
            "VALIDATION FAIL [min-length]: %d segment(s) shorter than %.2f m: %s",
            len(short_segs), min_length_m,
            [(i, f"{d:.3f}m") for i, d in short_segs[:5]],
        )
        ok = False
    else:
        log.info(
            "VALIDATION OK  [min-length]: all segments >= %.2f m.", min_length_m
        )

    # --- 3. No interior touches / X-crossings ---
    def _point_on_seg(p: tuple, a: tuple, b: tuple, tol: float = 1e-7) -> bool:
        ax, ay = a; bx, by = b; px, py = p
        dx, dy = bx - ax, by - ay
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-20:
            return False
        t = ((px - ax) * dx + (py - ay) * dy) / seg_len_sq
        if t <= 1e-9 or t >= 1 - 1e-9:
            return False
        foot_x = ax + t * dx
        foot_y = ay + t * dy
        return math.hypot(px - foot_x, py - foot_y) < tol

    endpoints: set[tuple] = set()
    for f in two_pt:
        endpoints.add(tuple(f["geometry"]["coordinates"][0]))
        endpoints.add(tuple(f["geometry"]["coordinates"][1]))

    interior_violations = 0
    # (a) T-junctions: foreign endpoint inside another segment.
    for fi in two_pt:
        a = tuple(fi["geometry"]["coordinates"][0])
        b = tuple(fi["geometry"]["coordinates"][1])
        for ep in endpoints:
            if ep == a or ep == b:
                continue
            if _point_on_seg(ep, a, b):
                interior_violations += 1
                if interior_violations <= 3:
                    log.error(
                        "VALIDATION FAIL [no-interior-touch]: endpoint %s lies inside segment %s→%s",
                        ep, a, b,
                    )

    # (b) X-crossings: two segments cross in their interiors.
    crossing_violations = 0
    coords_two_pt = [
        (tuple(f["geometry"]["coordinates"][0]),
         tuple(f["geometry"]["coordinates"][1]),
         f["properties"].get("osm_id"))
        for f in two_pt
    ]
    m = len(coords_two_pt)
    for i in range(m):
        a, b, oid_i = coords_two_pt[i]
        for j in range(i + 1, m):
            c, d, oid_j = coords_two_pt[j]
            if a == c or a == d or b == c or b == d:
                continue
            isect = _seg_seg_intersection_t(a, b, c, d)
            if isect is None:
                continue
            t, u = isect
            if 1e-9 < t < 1 - 1e-9 and 1e-9 < u < 1 - 1e-9:
                crossing_violations += 1
                interior_violations += 1
                if crossing_violations <= 3:
                    log.error(
                        "VALIDATION FAIL [no-interior-touch]: segments cross "
                        "(osm_id %s: %s→%s) × (osm_id %s: %s→%s) at t=%.3f, u=%.3f",
                        oid_i, a, b, oid_j, c, d, t, u,
                    )

    if interior_violations:
        log.error(
            "VALIDATION FAIL [no-interior-touch]: %d violation(s) total "
            "(T-junctions + X-crossings).",
            interior_violations,
        )
        ok = False
    else:
        log.info("VALIDATION OK  [no-interior-touch]: no interior touches or crossings.")

    # --- 4. Graph connectivity — ALL LineString segments (ways + connectors) ---
    if two_pt:
        G = nx.Graph()
        for f in two_pt:
            a_key = _coord_key(f["geometry"]["coordinates"][0])
            b_key = _coord_key(f["geometry"]["coordinates"][1])
            G.add_edge(a_key, b_key)
        n_comp = nx.number_connected_components(G)
        if n_comp != 1:
            log.error(
                "VALIDATION FAIL [connectivity]: LineString graph has %d connected component(s) (expected 1).",
                n_comp,
            )
            ok = False
        else:
            log.info(
                "VALIDATION OK  [connectivity]: graph is connected (%d nodes, %d edges).",
                G.number_of_nodes(), G.number_of_edges(),
            )
    else:
        log.warning("VALIDATION SKIP [connectivity]: no LineString features found.")

    # --- 5. Every Point feature lies on an endpoint of some LineString ---
    if point_features:
        orphan_points = [
            f for f in point_features
            if tuple(f["geometry"]["coordinates"]) not in endpoints
        ]
        if orphan_points:
            log.error(
                "VALIDATION FAIL [point-on-endpoint]: %d point feature(s) not on any LineString endpoint: osm_ids=%s",
                len(orphan_points),
                [f["properties"].get("osm_id") for f in orphan_points[:5]],
            )
            ok = False
        else:
            log.info(
                "VALIDATION OK  [point-on-endpoint]: all %d point features lie on a LineString endpoint.",
                len(point_features),
            )
    else:
        log.info("VALIDATION SKIP [point-on-endpoint]: no point features.")

    # --- 6. Narrow segments are isolated (no two share an endpoint) ---
    narrow_segs = [
        f for f in two_pt
        if f["properties"].get("narrow") == "yes"
    ]
    if narrow_segs:
        ep_to_narrow: dict[tuple, list[int]] = {}
        for idx, f in enumerate(narrow_segs):
            a_key = _coord_key(f["geometry"]["coordinates"][0])
            b_key = _coord_key(f["geometry"]["coordinates"][1])
            ep_to_narrow.setdefault(a_key, []).append(idx)
            ep_to_narrow.setdefault(b_key, []).append(idx)
        bad_endpoints = [
            (k, ids) for k, ids in ep_to_narrow.items() if len(ids) >= 2
        ]
        if bad_endpoints:
            log.error(
                "VALIDATION FAIL [narrow-isolated]: %d endpoint(s) shared by "
                "2+ narrow segments; first few: %s",
                len(bad_endpoints),
                [(k, ids) for k, ids in bad_endpoints[:3]],
            )
            ok = False
        else:
            log.info(
                "VALIDATION OK  [narrow-isolated]: all %d narrow segments are "
                "isolated (no shared endpoints).",
                len(narrow_segs),
            )
    else:
        log.info("VALIDATION SKIP [narrow-isolated]: no narrow segments.")

    if ok:
        log.info("VALIDATION PASSED — all checks OK.")
    else:
        log.error("VALIDATION FAILED — see errors above.")

    return ok


# ---------------------------------------------------------------------------
# Feature colouring — sets the ``stroke`` property used by GeoJSON viewers
# ---------------------------------------------------------------------------


def colorize_features(features: list[dict]) -> list[dict]:
    """Assign simplestyle colours to features in place; returns the list."""
    for feat in features:
        props = feat["properties"]
        geom_type = feat["geometry"]["type"]

        if geom_type == "Point":
            ptype = props.get("type")
            color = None
            if ptype == "base_point":
                color = COLOR_BASE_POINT
            elif ptype == "delivery_point":
                color = COLOR_DELIVERY_POINT
            if color is not None:
                props["marker-color"] = color
        elif geom_type == "LineString":
            color = None
            if props.get("narrow") == "yes":
                color = COLOR_NARROW
            if color is not None:
                props["stroke"] = color
    return features


# ---------------------------------------------------------------------------
# Location polygon helpers
# ---------------------------------------------------------------------------


def load_location_polygon(path: Path) -> list[list[float]]:
    """Return the exterior ring of the first Polygon in a GeoJSON file.

    Accepts a bare Polygon, a Feature, or a FeatureCollection.
    """
    with open(path, encoding="utf-8") as fh:
        data = json.load(fh)

    geom = None
    if data.get("type") == "Polygon":
        geom = data
    elif data.get("type") == "Feature":
        geom = data.get("geometry")
    elif data.get("type") == "FeatureCollection":
        features = data.get("features", [])
        if features:
            geom = features[0].get("geometry")

    if geom is None or geom.get("type") != "Polygon":
        raise ValueError(
            f"Expected a GeoJSON Polygon (or Feature/FeatureCollection containing one) "
            f"in {path}, got: {geom.get('type') if geom else 'nothing'}"
        )

    ring: list[list[float]] = geom["coordinates"][0]
    log.info(
        "Loaded location polygon from %s (%d vertices, bbox %.6f,%.6f – %.6f,%.6f).",
        path,
        len(ring),
        min(c[0] for c in ring),
        min(c[1] for c in ring),
        max(c[0] for c in ring),
        max(c[1] for c in ring),
    )
    return ring


def _dedupe_points(
    features: list[dict],
    min_distance_m: float,
    priority_of,
    label: str,
) -> list[dict]:
    """Greedily drop points closer than *min_distance_m* to a kept one.

    Sort by (priority desc, osm_id asc); ties broken by smaller osm_id for
    determinism. *priority_of* maps a feature to an int (higher = preferred).
    """
    if len(features) < 2:
        return features

    def _key(f: dict) -> tuple[int, int]:
        oid = f["properties"].get("osm_id")
        oid_int = oid if isinstance(oid, int) else 0
        return (-priority_of(f), oid_int)

    ordered = sorted(features, key=_key)
    kept: list[dict] = []
    for feat in ordered:
        lon, lat = feat["geometry"]["coordinates"]
        too_close = False
        for k in kept:
            klon, klat = k["geometry"]["coordinates"]
            if _lonlat_distance_m([lon, lat], [klon, klat]) < min_distance_m:
                too_close = True
                break
        if not too_close:
            kept.append(feat)

    dropped = len(features) - len(kept)
    log.info(
        "Dedupe %s: %d → %d (dropped %d within %.0f m).",
        label, len(features), len(kept), dropped, min_distance_m,
    )
    return kept


def dedupe_base_points(
    features: list[dict],
    min_distance_m: float = MIN_BASE_POINT_DISTANCE_M,
) -> list[dict]:
    """Greedily drop base points closer than *min_distance_m* to a kept one."""
    return _dedupe_points(
        features,
        min_distance_m,
        lambda f: BASE_POINT_PRIORITY.get(f["properties"].get("subtype", ""), 0),
        "base points",
    )


def dedupe_delivery_points(
    features: list[dict],
    min_distance_m: float = MIN_DELIVERY_POINT_DISTANCE_M,
) -> list[dict]:
    """Greedily drop delivery points closer than *min_distance_m* to a kept one."""
    return _dedupe_points(
        features,
        min_distance_m,
        lambda f: DELIVERY_POINT_PRIORITY.get(f["properties"].get("entrance", ""), 0),
        "delivery points",
    )


def load_basepoints_geojson(path: Path) -> list[dict]:
    """Load Point features from a GeoJSON file as extra base points.

    Each returned feature has ``properties.type`` defaulted to ``"base_point"``.
    """
    with open(path, encoding="utf-8") as fh:
        data = json.load(fh)

    if data.get("type") == "FeatureCollection":
        raw_features = data.get("features", [])
    elif data.get("type") == "Feature":
        raw_features = [data]
    elif data.get("type") == "Point":
        raw_features = [{"type": "Feature", "geometry": data, "properties": {}}]
    else:
        raw_features = []

    features: list[dict] = []
    for feat in raw_features:
        geom = feat.get("geometry") or {}
        if geom.get("type") != "Point":
            continue
        props = dict(feat.get("properties") or {})
        props.setdefault("type", "base_point")
        features.append({
            "type": "Feature",
            "geometry": geom,
            "properties": props,
        })

    if not features:
        raise ValueError(f"No Point features found in {path}")

    log.info("Loaded %d extra base point(s) from %s.", len(features), path)
    return features


def download_osm_for_polygon(ring: list[list[float]]) -> Path:
    """Download OSM data for the bbox of *ring* via Overpass; return tmp file path."""
    lons = [c[0] for c in ring]
    lats = [c[1] for c in ring]
    min_lon, max_lon = min(lons), max(lons)
    min_lat, max_lat = min(lats), max(lats)

    bbox = f"{min_lat},{min_lon},{max_lat},{max_lon}"
    query = (
        f"[out:xml][timeout:180];\n"
        f"(\n"
        f"  node({bbox});\n"
        f"  way({bbox});\n"
        f"  relation({bbox});\n"
        f");\n"
        f"out body;\n"
        f">;\n"
        f"out skel qt;\n"
    )

    overpass_url = "https://overpass-api.de/api/interpreter"
    log.info(
        "Downloading OSM data from Overpass API for bbox %.6f,%.6f – %.6f,%.6f …",
        min_lon, min_lat, max_lon, max_lat,
    )

    data = urllib.parse.urlencode({"data": query}).encode()
    req = urllib.request.Request(overpass_url, data=data, method="POST")
    req.add_header("Content-Type", "application/x-www-form-urlencoded")
    req.add_header("User-Agent", "process_map/1.0 (pedestrian-infrastructure-filter; https://github.com/iktovr/master-diploma)")

    with urllib.request.urlopen(req, timeout=200) as resp:
        xml_bytes = resp.read()

    log.info("Downloaded %.1f KB of OSM data.", len(xml_bytes) / 1024)

    tmp = tempfile.NamedTemporaryFile(suffix=".osm", delete=False)
    tmp.write(xml_bytes)
    tmp.close()
    return Path(tmp.name)


# ---------------------------------------------------------------------------
# Polygon containment filter
# ---------------------------------------------------------------------------


def _point_in_polygon(lon: float, lat: float, ring: list[list[float]]) -> bool:
    """Ray-casting point-in-polygon test in lon/lat space."""
    inside = False
    n = len(ring)
    j = n - 1
    for i in range(n):
        xi, yi = ring[i][0], ring[i][1]
        xj, yj = ring[j][0], ring[j][1]
        if ((yi > lat) != (yj > lat)) and (
            lon < (xj - xi) * (lat - yi) / (yj - yi + 1e-15) + xi
        ):
            inside = not inside
        j = i
    return inside


def filter_features_by_polygon(
    way_features: list[dict],
    store_features: list[dict],
    entrance_features: list[dict],
    building_polygons: list[list[list[float]]],
    barrier_ways: list[list[list[float]]],
    ring: list[list[float]],
) -> tuple[
    list[dict], list[dict], list[dict],
    list[list[list[float]]], list[list[list[float]]],
]:
    """Keep only features whose every coordinate lies inside *ring*."""
    def _all_coords_inside(coords: list[list[float]]) -> bool:
        return all(_point_in_polygon(c[0], c[1], ring) for c in coords)

    filtered_ways = [f for f in way_features if _all_coords_inside(f["geometry"]["coordinates"])]
    filtered_stores = [
        f for f in store_features
        if _point_in_polygon(f["geometry"]["coordinates"][0], f["geometry"]["coordinates"][1], ring)
    ]
    filtered_entrances = [
        f for f in entrance_features
        if _point_in_polygon(f["geometry"]["coordinates"][0], f["geometry"]["coordinates"][1], ring)
    ]
    filtered_buildings = [bp for bp in building_polygons if _all_coords_inside(bp)]
    filtered_barriers = [bw for bw in barrier_ways if _all_coords_inside(bw)]

    log.info(
        "Polygon filter: ways %d→%d, stores %d→%d, entrances %d→%d, "
        "buildings %d→%d, barriers %d→%d.",
        len(way_features), len(filtered_ways),
        len(store_features), len(filtered_stores),
        len(entrance_features), len(filtered_entrances),
        len(building_polygons), len(filtered_buildings),
        len(barrier_ways), len(filtered_barriers),
    )
    return (
        filtered_ways, filtered_stores, filtered_entrances,
        filtered_buildings, filtered_barriers,
    )


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
        required=False,
        default=None,
        metavar="FILE",
        help=(
            "Input OSM file (.osm, .osm.pbf, .osm.bz2, …). "
            "Optional when --location is provided; OSM data will be downloaded "
            "automatically via the Overpass API in that case."
        ),
    )
    parser.add_argument(
        "-o",
        "--output",
        required=False,
        default=None,
        metavar="FILE",
        help=(
            "Output GeoJSON file. If omitted, the rest of the processing "
            "pipeline is skipped (useful together with --output-map to only "
            "download the raw OSM data)."
        ),
    )
    parser.add_argument(
        "--output-map",
        required=False,
        default=None,
        metavar="FILE",
        help=(
            "Path to write the downloaded raw OSM file to. Only used when "
            "OSM data is downloaded via the Overpass API (i.e. when --input "
            "is not provided); ignored otherwise."
        ),
    )
    parser.add_argument(
        "-l",
        "--location",
        required=False,
        default=None,
        metavar="FILE",
        help=(
            "GeoJSON file containing a single Polygon. "
            "Only map features whose geometry lies entirely inside this polygon "
            "are kept. When --input is omitted, OSM data for the polygon's "
            "bounding box is downloaded automatically via the Overpass API."
        ),
    )
    parser.add_argument(
        "-b",
        "--basepoints",
        required=False,
        default=None,
        metavar="FILE",
        help=(
            "GeoJSON file containing Point features to use as additional base "
            "points (dark stores). These are merged with any base points found "
            "in the OSM data before connecting them to the pedestrian network."
        ),
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    # Resolve working directory for relative paths (Bazel sets BUILD_WORKING_DIRECTORY).
    work_dir = Path(os.getenv("BUILD_WORKING_DIRECTORY", "./"))

    def _resolve(p: str) -> Path:
        path = Path(p)
        if not path.is_absolute():
            path = (work_dir / path).resolve()
        return path

    output_path = _resolve(args.output) if args.output else None
    output_map_path = _resolve(args.output_map) if args.output_map else None

    if not args.input and not args.location:
        import sys
        print(
            "error: at least one of --input or --location must be provided.\n"
            "  Use --input to specify a local OSM file.\n"
            "  Use --location to provide a polygon GeoJSON; OSM data will be\n"
            "  downloaded automatically when --input is omitted.",
            file=sys.stderr,
        )
        sys.exit(2)

    if output_path is None and output_map_path is None:
        import sys
        print(
            "error: at least one of --output or --output-map must be provided.",
            file=sys.stderr,
        )
        sys.exit(2)

    location_ring: list[list[float]] | None = None
    if args.location:
        location_path = _resolve(args.location)
        location_ring = load_location_polygon(location_path)

    tmp_osm_path: Path | None = None
    if args.input:
        input_path = _resolve(args.input)
        if output_map_path is not None:
            log.info("--output-map ignored because --input was provided.")
            output_map_path = None
    else:
        assert location_ring is not None
        downloaded_path = download_osm_for_polygon(location_ring)
        if output_map_path is not None:
            output_map_path.parent.mkdir(parents=True, exist_ok=True)
            downloaded_path.replace(output_map_path)
            input_path = output_map_path
            log.info("Saved downloaded OSM data to %s.", output_map_path)
        else:
            tmp_osm_path = downloaded_path
            input_path = tmp_osm_path

    if output_path is None:
        log.info("--output not provided; skipping processing pipeline.")
        return

    try:
        handler = PedestrianHandler()
        handler.apply_file(str(input_path), locations=True)
        log.info("Collected %d pedestrian way features.", len(handler.way_features))
        log.info("Collected %d dark store nodes.", len(handler.store_features))
        log.info("Collected %d building entrance nodes.", len(handler.entrance_features))
        log.info("Collected %d building polygons.", len(handler.building_polygons))
        log.info("Collected %d barrier ways.", len(handler.barrier_ways))

        way_features = handler.way_features
        store_features = handler.store_features
        entrance_features = handler.entrance_features
        building_polygons = handler.building_polygons
        barrier_ways = handler.barrier_ways

        if location_ring is not None:
            (
                way_features, store_features, entrance_features,
                building_polygons, barrier_ways,
            ) = filter_features_by_polygon(
                way_features, store_features, entrance_features,
                building_polygons, barrier_ways, location_ring,
            )

        if args.basepoints:
            extra_bp_path = _resolve(args.basepoints)
            extra_store_features = load_basepoints_geojson(extra_bp_path)
            if location_ring is not None:
                before = len(extra_store_features)
                extra_store_features = [
                    f for f in extra_store_features
                    if _point_in_polygon(
                        f["geometry"]["coordinates"][0],
                        f["geometry"]["coordinates"][1],
                        location_ring,
                    )
                ]
                log.info(
                    "Polygon filter: extra base points %d→%d.",
                    before, len(extra_store_features),
                )
            osm_count = len(store_features)
            store_features = store_features + extra_store_features
            log.info(
                "Total base points after merging: %d (OSM) + %d (file) = %d.",
                osm_count, len(extra_store_features), len(store_features),
            )

        store_features = dedupe_base_points(store_features)
        entrance_features = dedupe_delivery_points(entrance_features)

        # Tag narrow passages before any splitting; the tag propagates through
        # the planarize/simplify passes via dict(props).
        way_features = tag_narrow_way_segments(
            way_features, building_polygons, barrier_ways,
        )

        way_features = largest_connected_component(way_features)
        way_features = planarize_ways(way_features)
        way_features = simplify_ways(way_features)
        way_features = remove_short_segments(way_features)

        connected_stores, skipped_stores, store_connectors = connect_points_to_network(
            way_features, store_features, "base_point",
        )
        if skipped_stores:
            log.warning("%d store node(s) could not be connected and were excluded.", len(skipped_stores))

        connected_entrances, skipped_entrances, entrance_connectors = connect_points_to_network(
            way_features, entrance_features, "delivery_point",
            building_polygons=building_polygons,
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

        # Resolve T-junctions / near-touch X-crossings introduced by later passes.
        all_features = fix_interior_violations(all_features)

        # Thin out chains of touching narrow segments. Runs last because earlier
        # splits inherit the narrow tag and can create new narrow-narrow touches.
        all_features = thin_narrow_segments(all_features)

        narrow_count = sum(
            1 for f in all_features
            if f["geometry"]["type"] == "LineString"
            and f["properties"].get("narrow") == "yes"
        )
        total_lines = sum(
            1 for f in all_features if f["geometry"]["type"] == "LineString"
        )
        log.info(
            "Final narrow segments: %d / %d (%.1f%%).",
            narrow_count, total_lines,
            100.0 * narrow_count / total_lines if total_lines else 0.0,
        )

        colorize_features(all_features)

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

        validate_geojson(all_features)

    finally:
        if tmp_osm_path is not None and tmp_osm_path.exists():
            tmp_osm_path.unlink()
            log.info("Removed temporary OSM file %s.", tmp_osm_path)


if __name__ == "__main__":
    main()
