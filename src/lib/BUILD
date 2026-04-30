load("@rules_cc//cc:cc_library.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

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
        "@opencv//:core",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
        ":agent",
    ]
)
