# -*- mode: python -*-
# vi: set ft=python :

load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def libfranka_common_v4_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "frankaemika/libfranka-common",
        commit = "e6aa0fc210d93fe618bfd8956829a264d5476ba8",
        sha256 = "721fffe11c5dec6735e191c2f08e58de3828d05ea25c3901c1940cd30a226509",  # noqa
        build_file = "//tools/workspace/libfranka_common_v4:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
