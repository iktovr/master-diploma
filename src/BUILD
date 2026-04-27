load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")

cc_library(
    name="agent",
    srcs = [
        "agent.cpp"
    ],
    hdrs = [
        "agent.h"
    ]
)

cc_library(
    name="simulation",
    srcs = [
        "simulation.cpp"
    ],
    hdrs = [
        "simulation.h"
    ],
    deps = [
        ":agent"
    ]
)

cc_library(
    name="visualizer",
    srcs = [
        "visualizer.cpp"
    ],
    hdrs = [
        "visualizer.h"
    ],
    deps = [
        "@opencv//:imgproc",
        "@opencv//:imgcodecs",
        ":agent"
    ]
)

cc_binary(
    name = "main",
    srcs = [
        "main.cpp"
    ],
    deps = [
        ":simulation",
        ":agent"
    ]
)