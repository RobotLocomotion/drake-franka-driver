# -*- mode: python -*-
# vi: set ft=python :

load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def libfranka_v4_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "frankaemika/libfranka",
        commit = "c452ba20397cde846fe2e48d0be94b522ef88dac",
        sha256 = "f0616d01ef09aa1d5948d385a6cfe18c514bc6e0921e4fac97ada7262b8722e4",  # noqa
        build_file = "//tools/workspace/libfranka_v4:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
