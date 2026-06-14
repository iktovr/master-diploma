load("@pypi//:requirements.bzl", "requirement")
load("@rules_cc//cc:defs.bzl", "cc_binary")
load("@rules_python//python:defs.bzl", "py_binary")

cc_binary(
    name = "demo",
    srcs = [
        "main.cpp",
    ],
    deps = [
        "//lib:agent",
        "//lib:ccbs_router",
        "//lib:graph",
        "//lib:logging",
        "//lib:router",
        "//lib:simulation",
        "//lib:statistics",
        "//lib:visualizer",
        "@cli11",
    ],
)

cc_binary(
    name = "draw_locations",
    srcs = [
        "draw_locations.cpp",
    ],
    deps = [
        "//lib:geometry",
        "//lib:graph",
        "//lib:visualizer",
        "@cli11",
        "@nlohmann_json//:singleheader-json",
        "@opencv//:core",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
)

py_binary(
    name = "run_sweep",
    srcs = ["run_sweep.py"],
    deps = [
        requirement("pandas"),
        requirement("tabulate"),
    ],
)
