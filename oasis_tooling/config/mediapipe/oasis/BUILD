load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "oasis_mediapipe_backend",
    srcs = ["oasis_mediapipe_backend.cc"],
    linkshared = 1,
    linkstatic = 1,
    visibility = ["//visibility:public"],
    deps = [
        "//mediapipe/calculators/core:pass_through_calculator",
        "//mediapipe/framework:calculator_cc_proto",
        "//mediapipe/framework:calculator_graph",
        "//mediapipe/framework/port:parse_text_proto",
        "//mediapipe/framework/port:status",
        "//mediapipe/tasks/c/vision/pose_landmarker:pose_landmarker_lib",
    ],
)
