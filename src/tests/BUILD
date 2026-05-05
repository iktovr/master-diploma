load("@rules_cc//cc:defs.bzl", "cc_test")

cc_test(
    name = "test_graph",
    srcs = [
        "test_graph.cpp",
    ],
    deps = [
        "//lib:graph",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
    data = [
        "data/graph.txt"
    ],
    size = "small",
)
