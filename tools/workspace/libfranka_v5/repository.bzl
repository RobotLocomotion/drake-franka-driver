# -*- mode: python -*-
# vi: set ft=python :

load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def libfranka_v5_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "frankaemika/libfranka",
        commit = "4f9e3cc666e42d267f1ab566869c4f4c552e5b57",
        sha256 = "ba546b58ff211c7cc41cda2a2067ba200983e49304f5ddccf9dbbb55ebfef7a4",  # noqa
        build_file = "//tools/workspace/libfranka_v5:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
