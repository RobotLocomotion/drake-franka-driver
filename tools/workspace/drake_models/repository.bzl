# -*- mode: python -*-
# vi: set ft=python :

load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "68e906556069eab9896f9006b6bf17a32b8b0f1d",
        sha256 = "bad9de839fc61447cecaf2abc8f8df7afe77e5ab4fbdeb28d9cfec0a8338b1fa",
        build_file = "@drake//tools/workspace/drake_models:package.BUILD.bazel",
        mirrors = mirrors,
    )