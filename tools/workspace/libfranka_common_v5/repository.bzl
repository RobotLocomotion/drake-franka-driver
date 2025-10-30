# -*- mode: python -*-
# vi: set ft=python :

load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def libfranka_common_v5_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "frankaemika/libfranka-common",
        commit = "cd38d0ec300b7e6864407d85d1e88e2fba31ccd5",
        sha256 = "0696297f235dc63c014232acde602aa9ad59bf8630d5b9901b32ac79f4d2dee2",  # noqa
        build_file = "//tools/workspace/libfranka_common_v5:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
