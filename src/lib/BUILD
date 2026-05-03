load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "agent",
    srcs = [
        "agent.cpp",
    ],
    hdrs = [
        "agent.h",
    ],
    deps = [
        ":geometry",
    ],
)

cc_library(
    name = "graph",
    srcs = [
        "graph.cpp",
    ],
    hdrs = [
        "graph.h",
    ],
    deps = [
        ":geometry",
    ],
)

cc_library(
    name = "simulation",
    srcs = [
        "simulation.cpp",
    ],
    hdrs = [
        "simulation.h",
    ],
    deps = [
        ":agent",
    ],
)

cc_library(
    name = "visualizer",
    srcs = [
        "visualizer.cpp",
    ],
    hdrs = [
        "visualizer.h",
    ],
    deps = [
        ":agent",
        ":geometry",
        "@opencv//:core",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
)

cc_library(
    name = "geometry",
    srcs = [
        "geometry.cpp",
    ],
    hdrs = [
        "geometry.h",
    ],
    deps = [
        "@boost.geometry",
    ],
)
