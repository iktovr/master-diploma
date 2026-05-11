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
        "@nlohmann_json//:singleheader-json",
        "@proj",
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
        ":dispatch",
        ":graph",
        ":semaphore",
        ":visualizer",
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
        ":graph",
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

cc_library(
    name = "dispatch",
    srcs = [
        "dispatch.cpp",
    ],
    hdrs = [
        "dispatch.h",
    ],
    deps = [
        ":agent",
        ":graph",
    ],
)

cc_library(
    name = "semaphore",
    srcs = [
        "semaphore.cpp",
    ],
    hdrs = [
        "semaphore.h",
    ],
    deps = [
        ":agent",
        ":geometry",
        ":graph",
    ],
)
