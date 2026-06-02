load("@rules_cc//cc:defs.bzl", "cc_test")

cc_test(
    name = "test_graph",
    size = "small",
    srcs = [
        "test_graph.cpp",
    ],
    data = [
        "data/graph.geojson",
        "data/graph.txt",
    ],
    deps = [
        "//lib:graph",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_semaphore",
    size = "small",
    srcs = [
        "test_semaphore.cpp",
    ],
    deps = [
        "//lib:agent",
        "//lib:geometry",
        "//lib:graph",
        "//lib:semaphore",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_agent",
    size = "small",
    srcs = [
        "test_agent.cpp",
    ],
    deps = [
        "//lib:agent",
        "//lib:geometry",
        "//lib:statistics",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_statistics",
    size = "small",
    srcs = [
        "test_statistics.cpp",
    ],
    deps = [
        "//lib:statistics",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_metrics_report",
    size = "small",
    srcs = [
        "test_metrics_report.cpp",
    ],
    deps = [
        "//lib:statistics",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_dispatch",
    size = "small",
    srcs = [
        "test_dispatch.cpp",
    ],
    deps = [
        "//lib:agent",
        "//lib:dispatch",
        "//lib:geometry",
        "//lib:graph",
        "//lib:router",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_simulation",
    size = "small",
    srcs = [
        "test_simulation.cpp",
    ],
    deps = [
        "//lib:agent",
        "//lib:graph",
        "//lib:router",
        "//lib:simulation",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_resolver",
    size = "small",
    srcs = [
        "test_resolver.cpp",
    ],
    deps = [
        "//lib:agent",
        "//lib:geometry",
        "//lib:graph",
        "//lib:resolver",
        "//lib:router",
        "//lib:simulation",
        "//lib:statistics",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_router",
    size = "small",
    srcs = [
        "test_router.cpp",
    ],
    deps = [
        "//lib:graph",
        "//lib:router",
        "//lib:statistics",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_ccbs_router",
    size = "small",
    srcs = [
        "test_ccbs_router.cpp",
    ],
    deps = [
        "//lib:agent",
        "//lib:ccbs_router",
        "//lib:graph",
        "//lib:statistics",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)
