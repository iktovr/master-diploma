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

py_binary(
    name = "run_sweep",
    srcs = ["run_sweep.py"],
)
