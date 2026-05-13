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

# Fraction of the connector's own length that may lie inside a building before
# the connection is rejected.  A connector whose path is more than this
# fraction inside any building is considered to go "through almost the full
# building" and is skipped.  A connector that only clips a corner (small
# inside-fraction) is still allowed.
MAX_BUILDING_THROUGH_FRACTION = 0.5

# Connectors shorter than this (metres) are never checked against buildings —
# they are assumed to be legitimate short connections (e.g. entrance right
# next to the footway).
MIN_CONNECTOR_LENGTH_FOR_BUILDING_CHECK_M = 10.0

# Maximum distance (metres) to connect a point node to the pedestrian network.
MAX_CONNECT_DISTANCE_M = 20.0
MIN_CONNECT_DISTANCE_M = 0.3

# Maximum perpendicular deviation (metres) allowed when simplifying a polyline
# segment via Ramer–Douglas–Peucker.  Segments that deviate more than this are
# split rather than collapsed.
MAX_SIMPLIFY_DEVIATION_M = 5.0

# Minimum length (metres) for a segment to be kept.  Segments shorter than
# this are collapsed during RDP and removed (with endpoint merging) after
# simplification.
MIN_SEGMENT_LENGTH_M = 0.25

# Barrier tag values to extract as obstacle line segments.
BARRIER_VALUES = frozenset(["wall", "fence", "kerb"])

# A passage is considered narrow when the clearance to the nearest
# building edge or barrier is below this threshold (metres).
NARROW_WIDTH_M = 3.0

# Narrow runs shorter than this are ignored (noise / measurement artefact).
NARROW_MIN_LENGTH_M = 1.0

# Narrow runs longer than this are also ignored (the whole corridor is
# narrow — not a bottleneck worth tagging).
NARROW_MAX_LENGTH_M = 50.0


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
        # Closed polygons for every OSM building way (lon/lat rings).
        self.building_polygons: list[list[list[float]]] = []
        # Open polylines for every OSM barrier way (wall/fence/kerb).
        self.barrier_ways: list[list[list[float]]] = []

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
        # Collect building polygons for connector-crossing checks.
        if w.tags.get("building"):
            try:
                coords = [[n.lon, n.lat] for n in w.nodes]
            except osmium.InvalidLocationError:
                coords = []
            # A valid closed polygon needs at least 4 nodes (3 unique + repeat).
            if len(coords) >= 4:
                self.building_polygons.append(coords)

        # Collect barrier polylines (walls, fences, kerbs) for narrow-passage
        # detection. A barrier way may be open or closed; we treat it as a
        # polyline either way.
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
                    "narrow": "no",
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
# Planarization — split polylines at every interior touch / crossing
# ---------------------------------------------------------------------------
# After this step every pair of LineStrings either:
#   • does not intersect at all, or
#   • shares exactly one endpoint.
# This is a prerequisite for the RDP simplification that follows, which
# produces 2-point segments: without planarization a long straight OSM way
# that is merely *touched* by another way in its interior would be collapsed
# to a single segment that skips the junction node.


def _seg_seg_intersection_t(
    a: tuple, b: tuple, c: tuple, d: tuple
) -> tuple[float, float] | None:
    """Parametric intersection of segment *a*–*b* with segment *c*–*d*.

    Works in any 2-D coordinate system (UTM metres here).

    Returns (t, u) where
        t ∈ [0, 1]  is the parameter along *a*–*b*
        u ∈ [0, 1]  is the parameter along *c*–*d*
    such that  a + t*(b-a) == c + u*(d-c).

    Returns None when the segments are parallel / collinear or do not
    intersect within [0, 1] × [0, 1].
    """
    ax, ay = a
    bx, by = b
    cx, cy = c
    dx, dy = d

    denom = (bx - ax) * (dy - cy) - (by - ay) * (dx - cx)
    if abs(denom) < 1e-10:
        return None  # parallel or collinear

    t = ((cx - ax) * (dy - cy) - (cy - ay) * (dx - cx)) / denom
    u = ((cx - ax) * (by - ay) - (cy - ay) * (bx - ax)) / denom

    if -1e-9 <= t <= 1 + 1e-9 and -1e-9 <= u <= 1 + 1e-9:
        return (max(0.0, min(1.0, t)), max(0.0, min(1.0, u)))
    return None


def planarize_ways(features: list[dict]) -> list[dict]:
    """Split every polyline at every point where another polyline touches or
    crosses it, so that the resulting set of LineStrings only meets at
    endpoints.

    Two kinds of junctions are handled:

    A. **Vertex junction** — an interior vertex of way *i* is also an endpoint
       of some other way *j*.  Way *i* must be split at that vertex so that
       the junction becomes an endpoint of both resulting sub-polylines.
       (This is the most common case in OSM: two ways share a node that is
       interior to one of them.)

    B. **T-junction** — a node of way *j* lies on a *segment* of way *i*
       (not at an existing vertex).  Way *i* is split at the projection foot.

    C. **X-crossing** — two segments from different ways properly cross.
       Both ways are split at the crossing point.

    All coordinate comparisons use the rounded key from ``_coord_key`` so that
    near-duplicate coordinates (< 1 cm apart) are treated as identical.
    """
    if not features:
        return features

    in_count = len(features)

    # Pre-project every way to UTM once.  We use the first coordinate of the
    # *entire dataset* as the single projection origin so that all ways share
    # the same metric space (avoids zone-boundary artefacts for small cities).
    ref_lon = features[0]["geometry"]["coordinates"][0][0]
    ref_lat = features[0]["geometry"]["coordinates"][0][1]
    proj = _utm_proj(ref_lon, ref_lat)

    # utm_coords[i] = list of (x, y) UTM tuples for way i
    utm_coords: list[list[tuple[float, float]]] = [
        [_to_utm(proj, c) for c in feat["geometry"]["coordinates"]]
        for feat in features
    ]

    n = len(features)

    # --- Build global vertex-ownership map ---
    # For each coord_key, collect the set of way indices that have a vertex
    # at that key (anywhere — endpoint or interior).  A shared vertex is one
    # that appears in 2+ different ways.
    vertex_owners: dict[tuple, set[int]] = {}
    for i, feat in enumerate(features):
        for c in feat["geometry"]["coordinates"]:
            vertex_owners.setdefault(_coord_key(c), set()).add(i)

    # split_vertex[i] = set of interior vertex indices of way i that must
    # become split points (case A).
    split_vertex: list[set[int]] = [set() for _ in range(n)]

    # split_params[i][seg_idx] = list of t values ∈ (0, 1) for cases B & C.
    split_params: list[list[list[float]]] = [
        [[] for _ in range(len(pts) - 1)]
        for pts in utm_coords
    ]

    # --- Case A: interior vertex of way i is also a vertex of some OTHER way ---
    # This handles both:
    #   • shared interior vertex (vertex interior to both ways) — formerly
    #     missed: such a vertex was not an endpoint of any way, so the old
    #     check failed, RDP later straightened it away, and the resulting
    #     chords could cross.
    #   • interior vertex that is an endpoint of another way (original case A).
    for i in range(n):
        coords_i = features[i]["geometry"]["coordinates"]
        # Only interior vertices (indices 1 .. len-2).
        for vi in range(1, len(coords_i) - 1):
            owners = vertex_owners.get(_coord_key(coords_i[vi]), ())
            # Shared with any way other than i?
            if any(o != i for o in owners):
                split_vertex[i].add(vi)

    # --- Cases B & C: segment-level intersections ---
    for i in range(n):
        pts_i = utm_coords[i]
        coords_i = features[i]["geometry"]["coordinates"]
        for j in range(n):
            if i == j:
                continue
            pts_j = utm_coords[j]
            coords_j = features[j]["geometry"]["coordinates"]

            # --- B: node of way j sitting on a segment of way i ---
            for node_lonlat in coords_j:
                node_key = _coord_key(node_lonlat)
                # Skip if it is already an endpoint of way i (no split needed).
                if node_key == _coord_key(coords_i[0]) or node_key == _coord_key(coords_i[-1]):
                    continue
                # Skip if it is already an interior vertex of way i — handled
                # by case A above.
                node_utm = _to_utm(proj, node_lonlat)
                nx_, ny_ = node_utm
                for si in range(len(pts_i) - 1):
                    # Skip segment if its start vertex is already a split point
                    # (the node would be at t≈0 of the next segment).
                    ax, ay = pts_i[si]
                    bx, by = pts_i[si + 1]
                    dx, dy = bx - ax, by - ay
                    seg_len_sq = dx * dx + dy * dy
                    if seg_len_sq < 1e-12:
                        continue
                    t = ((nx_ - ax) * dx + (ny_ - ay) * dy) / seg_len_sq
                    if t <= 1e-9 or t >= 1 - 1e-9:
                        continue
                    # Perpendicular distance must be < 1 cm.
                    foot_x = ax + t * dx
                    foot_y = ay + t * dy
                    dist = math.hypot(nx_ - foot_x, ny_ - foot_y)
                    if dist < 0.01:
                        split_params[i][si].append(t)

            # --- C: proper crossing of segments ---
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

    # Build output: for each way walk its vertices and emit a new sub-polyline
    # every time we reach a split vertex (case A) or a parametric split point
    # (cases B/C).
    result: list[dict] = []
    for i, feat in enumerate(features):
        coords = feat["geometry"]["coordinates"]
        props = feat["properties"]
        pts = utm_coords[i]

        # Collect parametric split points as (coord_idx_of_segment_start, t, lonlat).
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

        # Convert parametric splits to a dict keyed by segment index for fast
        # lookup during the walk.
        param_by_seg: dict[int, list[tuple[float, list]]] = {}
        for si, t, lonlat in deduped_param:
            param_by_seg.setdefault(si, []).append((t, lonlat))

        sv = split_vertex[i]
        no_splits = not sv and not param_by_seg
        if no_splits:
            result.append(feat)
            continue

        # Walk the coordinate list.
        current: list[list] = [coords[0]]

        for vi in range(1, len(coords)):
            # First emit any parametric split points on the segment (vi-1)→vi.
            for t, lonlat in param_by_seg.get(vi - 1, []):
                current.append(lonlat)
                if len(current) >= 2:
                    result.append({
                        "type": "Feature",
                        "geometry": {"type": "LineString", "coordinates": current},
                        "properties": dict(props),
                    })
                current = [lonlat]

            # Now add vertex vi itself.
            current.append(coords[vi])

            # If vi is a split vertex (and not the last vertex), close the
            # current sub-polyline here and start a new one.
            if vi in sv and vi < len(coords) - 1:
                if len(current) >= 2:
                    result.append({
                        "type": "Feature",
                        "geometry": {"type": "LineString", "coordinates": current},
                        "properties": dict(props),
                    })
                current = [coords[vi]]

        # Flush the last sub-polyline.
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
    """Perpendicular distance from point *p* to the infinite line through *a*–*b*
    in UTM (metric) space.  Returns 0 when *a* == *b*.
    """
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    seg_len = math.hypot(dx, dy)
    if seg_len == 0.0:
        return math.hypot(px - ax, py - ay)
    # Signed area of the triangle / base length = perpendicular height.
    return abs(dx * (ay - py) - dy * (ax - px)) / seg_len


def _rdp_split(
    pts: list[tuple[float, float]],
    tolerance_m: float,
    min_length_m: float = MIN_SEGMENT_LENGTH_M,
) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    """Recursively simplify a polyline using Ramer–Douglas–Peucker.

    Returns a list of 2-point pairs (each pair is a straight segment) such
    that every output segment deviates from the original geometry by at most
    *tolerance_m* metres.  The input must have at least 2 points.

    Segments whose chord length is less than *min_length_m* are dropped
    entirely (not emitted) so that degenerate micro-segments are eliminated
    before they reach the output.
    """
    if len(pts) == 2:
        chord = math.hypot(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1])
        if chord < min_length_m:
            return []  # too short — drop
        return [(pts[0], pts[1])]

    a, b = pts[0], pts[-1]

    # If the chord itself is shorter than the minimum, drop the whole sub-polyline.
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
        # The whole polyline is within tolerance — collapse to one segment.
        return [(a, b)]

    # Split at the farthest point and recurse on both halves.
    left = _rdp_split(pts[: max_idx + 1], tolerance_m, min_length_m)
    right = _rdp_split(pts[max_idx:], tolerance_m, min_length_m)
    return left + right


def simplify_ways(
    features: list[dict],
    tolerance_m: float = MAX_SIMPLIFY_DEVIATION_M,
) -> list[dict]:
    """Simplify every LineString way so that each output feature has exactly
    2 coordinates (a straight segment).

    Polylines that are already straight (or nearly so within *tolerance_m*)
    are collapsed to a single 2-point segment.  Polylines that bend more than
    *tolerance_m* are split into multiple 2-point segments at the points of
    maximum deviation (Ramer–Douglas–Peucker).

    The total number of output features is always ≥ the number of input
    features (splitting only, never merging across ways).

    Args:
        features:    List of GeoJSON LineString Feature dicts.
        tolerance_m: Maximum allowed perpendicular deviation in metres.

    Returns:
        A new list of GeoJSON LineString Feature dicts, each with exactly
        2 coordinates.
    """
    if not features:
        return features

    result: list[dict] = []
    in_count = len(features)

    for feat in features:
        coords = feat["geometry"]["coordinates"]
        props = feat["properties"]

        if len(coords) < 2:
            # Degenerate — skip.
            continue

        if len(coords) == 2:
            # Already a 2-point segment — pass through unchanged.
            result.append(feat)
            continue

        # Project to UTM using the first coordinate as origin.
        proj = _utm_proj(coords[0][0], coords[0][1])
        pts_utm = [_to_utm(proj, c) for c in coords]

        # Map UTM point (by identity) back to the original lon/lat so that
        # the polyline's own endpoints are never corrupted by the UTM
        # round-trip (which can introduce ~1e-14 floating-point noise).
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
    """Approximate great-circle distance in metres between two lon/lat points.

    Uses a flat-Earth approximation valid for distances < ~10 km.
    """
    lat_m = 111_320.0  # metres per degree of latitude
    lon_m = lat_m * math.cos(math.radians((a[1] + b[1]) / 2))
    return math.hypot((b[0] - a[0]) * lon_m, (b[1] - a[1]) * lat_m)


def remove_short_segments(
    features: list[dict],
    min_length_m: float = MIN_SEGMENT_LENGTH_M,
) -> list[dict]:
    """Remove 2-point LineString segments shorter than *min_length_m* metres
    while preserving graph connectivity.

    Each short segment is contracted: its two endpoints are merged into a
    single representative point (the midpoint).  Every other segment that
    referenced either of the two original endpoints is updated to use the
    merged point instead.  This is repeated until no short segments remain.

    The contraction order does not matter for correctness; we process
    shortest-first to minimise geometric distortion.
    """
    if not features:
        return features

    # Work with mutable coordinate lists (None = segment has been removed).
    coords_list: list[list[list] | None] = [
        list(feat["geometry"]["coordinates"]) for feat in features
    ]
    props_list = [feat["properties"] for feat in features]

    in_count = len(features)
    removed = 0

    changed = True
    while changed:
        changed = False

        # Find the shortest segment below the threshold.
        best_idx = -1
        best_len = min_length_m  # only consider segments strictly shorter

        for i, coords in enumerate(coords_list):
            if coords is None:
                continue
            d = _lonlat_distance_m(coords[0], coords[1])
            if d < best_len:
                best_len = d
                best_idx = i

        if best_idx == -1:
            break  # nothing left to remove

        # Contract: merge the two endpoints into their midpoint.
        seg = coords_list[best_idx]
        assert seg is not None  # guaranteed by the search loop above
        a = seg[0]
        b = seg[1]
        mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2]

        a_key = _coord_key(a)
        b_key = _coord_key(b)

        # Update all other segments that reference either endpoint.
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
                # Drop degenerate segments (both endpoints merged to same point).
                if _coord_key(new_coords[0]) == _coord_key(new_coords[1]):
                    coords_list[i] = None
                    removed += 1
                else:
                    coords_list[i] = new_coords

        # Remove the contracted segment.
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

    The returned foot coordinate is snapped to the nearest existing way-node
    if that node is within ``MIN_SEGMENT_LENGTH_M`` metres of the raw foot.
    This prevents creating split points that are nearly coincident with
    existing nodes (which would produce degenerate sub-segments and
    T-junction violations).

    Returns (foot_lonlat, way_index, seg_index, distance_m) or None if no
    valid perpendicular projection exists.
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

    # Snap the foot to the nearest existing way-node if it is within
    # MIN_SEGMENT_LENGTH_M.  This avoids creating near-zero-length sub-segments
    # and T-junction violations caused by UTM round-trip float noise.
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
# Building-crossing check — reject connectors that traverse most of a building
# ---------------------------------------------------------------------------


def _segment_inside_length_utm(
    a_xy: tuple[float, float],
    b_xy: tuple[float, float],
    ring_xy: list[tuple[float, float]],
) -> float:
    """Return the total length (metres, UTM) of the segment *a*–*b* that lies
    **inside** the closed polygon defined by *ring_xy* (a list of UTM (x, y)
    tuples forming a closed ring, i.e. first == last).

    Algorithm
    ---------
    1. Collect all t ∈ (0, 1) where the segment crosses a polygon edge.
    2. Sort the t values and evaluate the midpoint of each sub-interval.
    3. Use a ray-casting point-in-polygon test to decide whether each
       midpoint is inside the polygon.
    4. Sum the lengths of the inside sub-intervals.
    """
    ax, ay = a_xy
    bx, by = b_xy
    seg_len = math.hypot(bx - ax, by - ay)
    if seg_len < 1e-9:
        return 0.0

    # Collect crossing t-values with polygon edges.
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

    # Ray-casting PIP test (horizontal ray, UTM space).
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
    """Return True if the connector segment *pt_lonlat* → *foot_lonlat* passes
    through *almost the full length* of any building polygon.

    Short connectors (< *min_connector_len_m* metres) are never blocked —
    they are assumed to be legitimate short connections (entrance right next
    to the footway).

    For longer connectors, "almost the full length" means the portion of the
    connector that lies inside the building exceeds *max_through_fraction* ×
    the connector's own length.  A connector that only clips a corner of a
    building (small inside-fraction relative to the connector length) returns
    False.
    """
    if not building_polygons:
        return False

    proj = _utm_proj(pt_lonlat[0], pt_lonlat[1])
    a_xy = _to_utm(proj, pt_lonlat)
    b_xy = _to_utm(proj, foot_lonlat)

    connector_len = math.hypot(b_xy[0] - a_xy[0], b_xy[1] - a_xy[1])

    # Short connectors are always allowed — no building check needed.
    if connector_len < min_connector_len_m:
        return False

    for ring_lonlat in building_polygons:
        # Convert ring to UTM.
        ring_xy: list[tuple[float, float]] = [_to_utm(proj, c) for c in ring_lonlat]
        # Ensure the ring is closed.
        if ring_xy[0] != ring_xy[-1]:
            ring_xy.append(ring_xy[0])

        # Skip degenerate buildings.
        xs = [p[0] for p in ring_xy]
        ys = [p[1] for p in ring_xy]
        if math.hypot(max(xs) - min(xs), max(ys) - min(ys)) < 1e-3:
            continue

        inside_len = _segment_inside_length_utm(a_xy, b_xy, ring_xy)
        # Reject if the connector spends more than max_through_fraction of its
        # own length inside this building.
        if inside_len / connector_len > max_through_fraction:
            return True

    return False


# ---------------------------------------------------------------------------
# Narrow-passage detection — tag pedestrian ways that pass through tight
# spots between buildings / barriers (walls, fences, kerbs)
# ---------------------------------------------------------------------------


def _point_to_segment_distance_utm(
    p_xy: tuple[float, float],
    a_xy: tuple[float, float],
    b_xy: tuple[float, float],
) -> float:
    """Perpendicular distance from point *p* to segment *a*–*b* in UTM metres.

    If the perpendicular foot falls outside the segment, returns the distance
    to the nearer endpoint.
    """
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
    """Return the minimum distance from *p_xy* to any obstacle edge.

    Obstacles are building polygon edges and barrier polyline segments.
    The search is short-circuited once a distance below *cap* is observed
    further reduced — this is a simple optimisation; the function still has
    to scan every edge in the worst case, but in practice obstacles are
    sparse and most points are far from all of them.
    """
    best = cap
    px, py = p_xy
    for a_xy, b_xy in building_edges_xy:
        # Cheap bbox reject — skip far edges quickly.
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
    """Tag pedestrian ways that pass through narrow spots with ``narrow=yes``.

    For each pedestrian way we measure the clearance at every node to the
    nearest obstacle (building polygon edge or barrier polyline segment).
    Consecutive nodes whose clearance is below *narrow_width_m* form a
    *narrow run*.  The run's arc-length must satisfy
    ``narrow_min_m <= length <= narrow_max_m`` to qualify; otherwise it is
    ignored (too-short = noise; too-long = the whole corridor is narrow and
    not a meaningful bottleneck).

    A way that contains at least one qualifying narrow run is tagged with
    ``properties["narrow"] = "yes"``.  Because the tag is set on the
    original (multi-node) way feature **before** planarization and
    simplification, it propagates automatically to every derived 2-point
    segment via ``dict(props)`` in the downstream passes.

    The way geometries themselves are not modified — only the ``properties``
    dict is updated in place.  Returns the same list (for chaining).
    """
    if not way_features:
        return way_features
    if not building_polygons and not barrier_ways:
        log.info("Narrow detection: no buildings or barriers — skipping.")
        return way_features

    # Use a single UTM projection origin for the whole dataset.
    ref_lon = way_features[0]["geometry"]["coordinates"][0][0]
    ref_lat = way_features[0]["geometry"]["coordinates"][0][1]
    proj = _utm_proj(ref_lon, ref_lat)

    # Pre-project building polygon edges to UTM (one edge per polygon side).
    building_edges_xy: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for ring in building_polygons:
        ring_xy = [_to_utm(proj, c) for c in ring]
        for k in range(len(ring_xy) - 1):
            building_edges_xy.append((ring_xy[k], ring_xy[k + 1]))

    # Pre-project barrier polyline segments to UTM.
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

    # The clearance cap used for the per-node distance search: any clearance
    # >= narrow_width_m is irrelevant (we only care whether it is below the
    # threshold), so we can early-out at narrow_width_m.  We add a tiny
    # margin so that points exactly at the threshold are treated as "wide".
    cap = narrow_width_m

    ways_tagged = 0
    runs_total = 0
    runs_kept = 0

    for feat in way_features:
        coords = feat["geometry"]["coordinates"]
        if len(coords) < 2:
            continue

        pts_xy = [_to_utm(proj, c) for c in coords]
        # Clearance at each node (capped at narrow_width_m for speed).
        clearances = [
            _min_clearance_to_obstacles(p, building_edges_xy, barrier_segs_xy, cap)
            for p in pts_xy
        ]

        # Walk node-by-node, tracking runs of consecutive narrow nodes.
        # A "narrow run" is a maximal contiguous index range [i0, i1] where
        # every clearance[i] < narrow_width_m.  Its arc-length is the sum of
        # the segment lengths between consecutive nodes in [i0, i1].
        has_qualifying_run = False
        i = 0
        n = len(pts_xy)
        while i < n:
            if clearances[i] < narrow_width_m:
                j = i
                while j + 1 < n and clearances[j + 1] < narrow_width_m:
                    j += 1
                # Arc-length of the run i..j.
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
    """Thin out clusters of touching ``narrow=yes`` segments.

    After [`tag_narrow_way_segments`](src/maps/process_map.py:1) and the
    planarize/simplify passes there may be long chains or clusters where
    many short 2-point segments are all tagged ``narrow=yes`` and touch
    each other at endpoints.  For downstream routing / visualisation we
    only want *isolated* narrow segments: between any two segments that
    keep the ``narrow`` tag, at least one non-narrow segment must lie in
    between.

    Algorithm
    ---------
    1. Build the touch-graph of narrow segments: two narrow segments are
       neighbours iff they share an endpoint (``_coord_key``).
    2. Repeatedly remove the shortest segment that still has at least one
       narrow neighbour — i.e. unmark it (delete its ``narrow`` property).
       Removing the shortest first keeps the longest, most "obvious"
       narrow spots tagged.
    3. Stop when no narrow segment touches any other narrow segment.

    The original feature list is returned with ``properties["narrow"]``
    removed from the unmarked features.  Geometry is not modified.
    """
    if not features:
        return features

    # Index of every narrow segment in the input list.
    narrow_idx: list[int] = [
        i for i, f in enumerate(features)
        if f["properties"].get("narrow") == "yes"
        and f["geometry"]["type"] == "LineString"
        and len(f["geometry"]["coordinates"]) == 2
    ]

    if len(narrow_idx) < 2:
        return features

    # Pre-compute endpoint keys and lengths for narrow segments.
    endpoint_keys: dict[int, tuple[tuple, tuple]] = {}
    lengths: dict[int, float] = {}
    for i in narrow_idx:
        coords = features[i]["geometry"]["coordinates"]
        a_key = _coord_key(coords[0])
        b_key = _coord_key(coords[1])
        endpoint_keys[i] = (a_key, b_key)
        lengths[i] = _lonlat_distance_m(coords[0], coords[1])

    # endpoint -> set of narrow segment indices touching that endpoint.
    by_endpoint: dict[tuple, set[int]] = {}
    for i in narrow_idx:
        a_key, b_key = endpoint_keys[i]
        by_endpoint.setdefault(a_key, set()).add(i)
        by_endpoint.setdefault(b_key, set()).add(i)

    # narrow_neighbours[i] = set of narrow segments j != i that share an endpoint with i.
    narrow_neighbours: dict[int, set[int]] = {i: set() for i in narrow_idx}
    for ep, members in by_endpoint.items():
        if len(members) < 2:
            continue
        members_list = list(members)
        for a in members_list:
            for b in members_list:
                if a != b:
                    narrow_neighbours[a].add(b)

    # Greedy removal: at each step pick the shortest narrow segment that
    # still has at least one narrow neighbour and unmark it.
    #
    # We use a simple priority list: scan for the minimum each iteration.
    # The narrow set is typically small (a few hundred at most) so an
    # O(k^2) scan is fine here.
    active: set[int] = set(narrow_idx)
    unmarked = 0

    while True:
        # Find shortest segment in `active` that still has an active neighbour.
        worst_idx = -1
        worst_len = math.inf
        for i in active:
            if narrow_neighbours[i] & active:
                if lengths[i] < worst_len:
                    worst_len = lengths[i]
                    worst_idx = i

        if worst_idx == -1:
            break  # no two active narrow segments touch — done

        # Unmark the shortest conflicting segment.
        active.discard(worst_idx)
        # Remove the ``narrow`` property from the feature.
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
# Connect point nodes to the nearest edge, splitting it at the projection
# ---------------------------------------------------------------------------


def connect_points_to_network(
    way_features: list[dict],
    point_features: list[dict],
    point_type: str,
    max_distance_m: float = MAX_CONNECT_DISTANCE_M,
    min_distance_m: float = MIN_CONNECT_DISTANCE_M,
    building_polygons: list[list[list[float]]] | None = None,
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

        # Check if the foot is too close to either endpoint of the segment.
        # If so, snap to that endpoint to avoid creating a degenerate
        # near-zero-length sub-segment after the split.
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
            # Snap foot to the start of the segment — no split needed.
            foot = coords[si]
        elif dist_to_b < min_distance_m:
            # Snap foot to the end of the segment — no split needed.
            foot = coords[si + 1]

        # Re-check connector length after possible snap.
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

        # For delivery_point: reject the connector if it passes through almost
        # the full extent of any building polygon (i.e. it goes through the
        # building rather than just clipping a corner).
        if point_type == "delivery_point" and building_polygons:
            if _connector_blocked_by_building(pt_coord, foot, building_polygons):
                log.debug(
                    "%s osm_id=%s: connector blocked — passes through most of a building, skipping.",
                    point_type, osm_id,
                )
                skipped_points.append(point)
                continue

        # Only split if the foot is strictly interior to the segment
        # (not snapped to an existing endpoint).
        foot_key = _coord_key(foot)
        seg_a_key = _coord_key(coords[si])
        seg_b_key = _coord_key(coords[si + 1])

        if foot_key == seg_a_key or foot_key == seg_b_key:
            # Foot is at an existing node — no split, just add connector.
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
    """Eliminate T-junctions and near-touch X-crossings remaining after the
    main pipeline.

    Operates on 2-point LineString features (ways + connectors).  For each
    segment, finds any foreign endpoint that lies on it (within *tol_m*
    perpendicular distance, strictly between the segment's endpoints):

    - If the foreign endpoint is closer than *min_len_m* to one of the
      segment's own endpoints, it is **snapped** to that endpoint.  All
      LineString and Point features referencing the snapped lon/lat are
      updated so the merged node is a single shared coordinate.  This
      prevents creating sub-segments shorter than *min_len_m*.

    - Otherwise, the segment is **split** at the foreign endpoint's exact
      coordinate, producing two new 2-point segments that both have the
      foreign coordinate as an endpoint.  Properties are inherited.

    The procedure iterates until no further violations are detected or
    *max_iterations* is reached.  Degenerate segments (both endpoints
    coincident after snapping) are removed.

    All near-touch X-crossings observed in practice degenerate into
    T-junctions (one segment's endpoint sits ~ε metres off the other
    segment's line), so this single mechanism resolves both validator
    failure modes.
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

        # Collect endpoint lon/lat tuples and their UTM coords.
        endpoints: set[tuple] = set()
        for i in lidxs:
            coords = features[i]["geometry"]["coordinates"]
            endpoints.add(tuple(coords[0]))
            endpoints.add(tuple(coords[1]))
        ep_utm: dict[tuple, tuple[float, float]] = {
            ep: _to_utm(proj, list(ep)) for ep in endpoints
        }

        # Pass 1: collect snap pairs.  An endpoint ep is snapped to one of
        # segment AB's own endpoints when ep is within tol_m perpendicular
        # of AB and within min_len_m of A or B along AB.
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
            # Resolve chains: a -> b, b -> c  becomes  a -> c.
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

            # Apply snaps to every feature (LineString + Point).
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
                        # Degenerate after snap — drop the feature.
                        f.clear()
                        f["__deleted__"] = True
                    else:
                        geom["coordinates"] = new_coords
                elif geom["type"] == "Point":
                    tc = tuple(geom["coordinates"])
                    if tc in snap_pairs:
                        geom["coordinates"] = list(snap_pairs[tc])

            # Materialise deletions.
            for idx in range(len(features)):
                if features[idx] is not None and features[idx].get("__deleted__"):
                    features[idx] = None

            total_snaps += len(snap_pairs)
            # Re-iterate after a snap pass before considering splits.
            continue

        # Pass 2: collect split actions.  Foreign endpoints strictly inside
        # AB (with both sides >= min_len_m) trigger a split at the
        # foreign endpoint's exact lon/lat.
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
                # Skip too-close cases (would create short sub-segment); these
                # weren't snapped above only because the perpendicular distance
                # exceeded tol_m on the previous pass — extremely rare; safer
                # to leave them than to create a short segment.
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
    """Validate the final feature collection against five invariants.

    Checks (applied to LineString features unless noted):

    1. **2-point segments** — every LineString has exactly 2 coordinates.
    2. **Minimum length** — every segment is at least *min_length_m* metres long.
    3. **No interior touches / crossings** — no endpoint of any segment lies
       strictly inside another segment (T-junctions or X-crossings).
    4. **Graph connectivity** — the graph formed by ALL LineString segments is
       connected (ways + connectors together).
    5. **Point-on-endpoint** — every Point feature (base_point / delivery_point)
       has its coordinate equal to an endpoint of some LineString.

    Returns True if all checks pass, False otherwise.  All failures are logged
    at ERROR level so they appear in the normal log stream.
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
    # Two failure modes are checked:
    #   (a) T-junction: an endpoint of one segment lies strictly inside
    #       another segment.
    #   (b) X-crossing: two segments cross each other strictly in their
    #       interiors (neither share an endpoint at the crossing point).
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
    # (a) T-junctions
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

    # (b) X-crossings — two segments crossing strictly in their interiors.
    # Uses ``_seg_seg_intersection_t`` which returns the parametric (t, u) of
    # the intersection on segments AB and CD respectively.  We flag the pair
    # when both parameters are strictly inside (0, 1) — that excludes shared
    # endpoints, which are legal.
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
            # Skip if the two segments share an endpoint — that's legal.
            if a == c or a == d or b == c or b == d:
                continue
            isect = _seg_seg_intersection_t(a, b, c, d)
            if isect is None:
                continue
            t, u = isect
            # Strictly interior on BOTH segments.
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

# Colour palette in #rrggbb hex format (recognised by geojson.io,
# Mapbox simplestyle-spec, etc.).
COLOR_NARROW = "#ff8800"        # orange — narrow pedestrian segments
COLOR_BASE_POINT = "#ff0000"    # red    — base points (dark stores)
COLOR_DELIVERY_POINT = "#3388ff"  # green  — building-entrance delivery points


def colorize_features(features: list[dict]) -> list[dict]:
    """Set the ``stroke`` (and, for Points, ``marker-color``) property on
    every feature according to its type / tags:

    - ``narrow=yes`` LineStrings → orange
    - ``base_point`` Points       → red
    - ``delivery_point`` Points   → green
    - connector LineStrings       → grey
    - other LineStrings           → blue

    Points additionally receive ``marker-color`` so simplestyle-aware
    viewers render the marker in the same colour.

    The features list is modified in place and also returned for chaining.
    """
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
    """Load a GeoJSON file and return the exterior ring of the first Polygon.

    The file may be a GeoJSON ``Feature`` with a ``Polygon`` geometry, a bare
    ``Polygon`` geometry object, or a ``FeatureCollection`` whose first feature
    has a ``Polygon`` geometry.

    Returns a list of ``[lon, lat]`` pairs representing the exterior ring
    (``coordinates[0]`` of the polygon).

    Raises ``ValueError`` if no Polygon geometry can be found.
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


def load_basepoints_geojson(path: Path) -> list[dict]:
    """Load Point features from a GeoJSON file to use as extra base points.

    Accepts a GeoJSON FeatureCollection of Point features, a single Feature
    with a Point geometry, or a bare Point geometry object.

    Each loaded point is returned as a GeoJSON Feature dict with
    ``properties.type`` set to ``"base_point"``.  Any existing properties on
    the feature are preserved; ``type`` is only added when absent.

    Raises ``ValueError`` if the file contains no Point features.
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
    """Download OSM data for the bounding box of *ring* via the Overpass API.

    Sends a POST request to ``https://overpass-api.de/api/interpreter`` with
    an Overpass QL query that fetches all nodes, ways and relations inside the
    bounding box derived from *ring*.  The response XML is written to a
    temporary file whose path is returned.

    The caller is responsible for deleting the file when it is no longer needed.
    """
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
    """Ray-casting point-in-polygon test in lon/lat space.

    *ring* is a list of ``[lon, lat]`` pairs forming a closed polygon ring
    (first point == last point is not required but accepted).

    Returns ``True`` if the point ``(lon, lat)`` is strictly inside the ring.
    """
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
    """Keep only features whose geometry lies entirely inside *ring*.

    Rules:
    - **LineString ways**: kept only if *every* coordinate is inside the polygon.
    - **Point features** (stores, entrances): kept only if the point is inside.
    - **Building polygons**: kept only if *every* vertex is inside the polygon.
    - **Barrier ways**: kept only if *every* vertex is inside the polygon.

    Returns filtered ``(way_features, store_features, entrance_features,
    building_polygons, barrier_ways)``.
    """
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
        required=True,
        metavar="FILE",
        help="Output GeoJSON file",
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

    output_path = _resolve(args.output)

    # Validate argument combination.
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

    # Load location polygon (if provided).
    location_ring: list[list[float]] | None = None
    if args.location:
        location_path = _resolve(args.location)
        location_ring = load_location_polygon(location_path)

    # Resolve or download the input OSM file.
    tmp_osm_path: Path | None = None
    if args.input:
        input_path = _resolve(args.input)
    else:
        # --location is guaranteed to be set here (validated above).
        assert location_ring is not None
        tmp_osm_path = download_osm_for_polygon(location_ring)
        input_path = tmp_osm_path

    try:
        handler = PedestrianHandler()
        handler.apply_file(str(input_path), locations=True)
        log.info("Collected %d pedestrian way features.", len(handler.way_features))
        log.info("Collected %d dark store nodes.", len(handler.store_features))
        log.info("Collected %d building entrance nodes.", len(handler.entrance_features))
        log.info("Collected %d building polygons.", len(handler.building_polygons))
        log.info("Collected %d barrier ways.", len(handler.barrier_ways))

        # Apply polygon filter before any further processing.
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

        # Load and merge extra base points from --basepoints file (if provided).
        if args.basepoints:
            extra_bp_path = _resolve(args.basepoints)
            extra_store_features = load_basepoints_geojson(extra_bp_path)
            # Apply polygon filter to extra points if a location polygon is active.
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

        # Tag narrow passages on the original (multi-node) ways before any
        # splitting.  The ``narrow=yes`` tag propagates to every derived
        # 2-point segment through the planarize/simplify passes (both copy
        # properties via ``dict(props)``).
        way_features = tag_narrow_way_segments(
            way_features, building_polygons, barrier_ways,
        )

        # Keep only the largest connected component, planarize (split at every
        # interior junction / crossing), then simplify every polyline to a set of
        # 2-point straight segments (Ramer–Douglas–Peucker).
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

        # Resolve any remaining T-junctions / near-touch X-crossings introduced
        # by simplify_ways, remove_short_segments, and connect_points_to_network
        # (these passes modify geometry after planarize_ways).
        all_features = fix_interior_violations(all_features)

        # Thin out chains of touching narrow segments so that no two
        # ``narrow=yes`` segments share an endpoint — between any pair of
        # narrow segments at least one non-narrow segment must remain.
        # This runs AFTER point connection and fix_interior_violations,
        # because both can split ways and the resulting halves inherit
        # ``narrow`` from their parent, potentially creating new
        # narrow-narrow touches.
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

        # Assign per-feature colours (stroke / marker-color) for visualisation.
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
