# -*- mode: python -*-
# vi: set ft=python :

load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def libfranka_v4_repository(
        name,
        mirrors = None):
    """
    Provides libfranka v4 (0.9.0) for FE3 robots with firmware v4.x.

    This is the legacy version for older Panda robots. Use libfranka_v5
    for newer FR3 robots instead.

    Commit: c452ba20397cde846fe2e48d0be94b522ef88dac (v0.9.0 release)
    """
    github_archive(
        name = name,
        repository = "frankarobotics/libfranka",
        commit = "c452ba20397cde846fe2e48d0be94b522ef88dac",
        sha256 = "f0616d01ef09aa1d5948d385a6cfe18c514bc6e0921e4fac97ada7262b8722e4",  # noqa
        build_file = "//tools/workspace/libfranka_v4:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
