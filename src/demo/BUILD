load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "demo",
    srcs = [
        "main.cpp",
    ],
    deps = [
        "//lib:agent",
        "//lib:graph",
        "//lib:simulation",
        "//lib:visualizer",
        "@cli11",
    ],
)
