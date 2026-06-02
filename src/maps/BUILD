load("@rules_shell//shell:sh_binary.bzl", "sh_binary")

# process_map uses the system python3 with packages installed via apt.
# The wrapper script forwards all CLI arguments to process_map.py.
sh_binary(
    name = "process_map",
    srcs = ["process_map_wrapper.sh"],
    data = ["process_map.py"],
    visibility = ["//visibility:public"],
)
