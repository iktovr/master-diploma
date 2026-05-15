#!/usr/bin/env bash
# Regenerate small/medium/large *_map.geojson via //maps:process_map.
# Usage: regenerate_all.sh [path/to/map.osm]
# When map.osm is omitted, OSM data is fetched from Overpass for each region.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(dirname "${SCRIPT_DIR}")"

INPUT_ARGS=()
if [[ $# -ge 1 ]]; then
    INPUT_ARGS=(--input "$1")
fi

cd "${SRC_DIR}"
for size in small medium large; do
    echo "=== Regenerating ${size}_map.geojson ==="
    bazel run //maps:process_map -- \
        "${INPUT_ARGS[@]}" \
        --location "maps/${size}_location.geojson" \
        --output "maps/${size}_map.geojson"
done
