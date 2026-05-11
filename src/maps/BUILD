load("@rules_shell//shell:sh_binary.bzl", "sh_binary")

# filter_osm uses the system python3 with packages installed via apt.
# The wrapper script forwards all CLI arguments to filter_osm.py.
sh_binary(
    name = "filter_osm",
    srcs = ["filter_osm_wrapper.sh"],
    data = ["filter_osm.py"],
    visibility = ["//visibility:public"],
)
