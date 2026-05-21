load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ccbs",
    srcs = [
        "cbs.cpp",
        "config.cpp",
        "heuristic.cpp",
        "map.cpp",
        "sipp.cpp",
        "task.cpp",
        "tinyxml2.cpp",
        "xml_logger.cpp",
    ],
    hdrs = [
        "cbs.h",
        "config.h",
        "const.h",
        "heuristic.h",
        "map.h",
        "sipp.h",
        "structs.h",
        "task.h",
        "tinyxml2.h",
        "xml_logger.h",
    ],
    copts = [
        "-std=c++11",
        "-O2",
        "-Wall",
        "-Wextra",
    ],
    include_prefix = "third_party/ccbs",
    strip_include_prefix = "/third_party/ccbs",
    deps = [
        ":simplex",
        "@boost.multi_index",
    ],
)

cc_library(
    name = "simplex",
    srcs = [
        "simplex/columnset.cpp",
        "simplex/constraint.cpp",
        "simplex/matrix.cpp",
        "simplex/objectivefunction.cpp",
        "simplex/simplex.cpp",
        "simplex/variable.cpp",
    ],
    hdrs = [
        "simplex/columnset.h",
        "simplex/constraint.h",
        "simplex/datamismatchexception.h",
        "simplex/divisionbyzeroexception.h",
        "simplex/indexoutofboundexception.h",
        "simplex/matrix.h",
        "simplex/matrixissingularexception.h",
        "simplex/matrixnotsquareexception.h",
        "simplex/memoryreachedoutexception.h",
        "simplex/notavectorexception.h",
        "simplex/objectivefunction.h",
        "simplex/pilal.h",
        "simplex/pilalexceptions.h",
        "simplex/simplex.h",
        "simplex/simplexexceptions.h",
        "simplex/sizemismatchexception.h",
        "simplex/variable.h",
    ],
    copts = [
        "-std=c++11",
        "-O2",
        "-Wall",
        "-Wextra",
    ],
)

cc_binary(
    name = "ccbs_main",
    srcs = ["main.cpp"],
    copts = [
        "-std=c++11",
        "-O2",
        "-Wall",
        "-Wextra",
    ],
    deps = [":ccbs"],
)
