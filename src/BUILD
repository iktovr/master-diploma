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

cc_binary(
    name = "main",
    srcs = [
        "main.cpp"
    ],
    deps = [
        ":agent"
    ]
)