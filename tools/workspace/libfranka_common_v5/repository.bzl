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
        commit = "ea26b89aa302b1456c825fb622b414bb3b1013ce",
        sha256 = "2f8c3dc6a63ab4bcb8ab770b4f39105a4a4fd64bdcd2c98755b7c51754be1ead",  # noqa
        build_file = "//tools/workspace/libfranka_common_v5:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
