# -*- mode: python -*-
# vi: set ft=python :

load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def libfranka_v5_repository(
        name,
        mirrors = None):
    """
    Provides libfranka v5 (~0.15.0) for FR3 robots with firmware v5.x.

    This version includes:
    - Support for FR3 robot hardware
    - Pinocchio integration for dynamics
    - Modern API with improved real-time control

    Commit: faaefaa0ff17a812b8a9bc4ac1ed7c353c46064d (post-0.15.0)
    """
    github_archive(
        name = name,
        repository = "frankarobotics/libfranka",
        commit = "faaefaa0ff17a812b8a9bc4ac1ed7c353c46064d",
        sha256 = "d733299b6a88762405cb56fce605d3f98ab2d78ecd6464ee1b255017e6ed903b",  # noqa
        build_file = "//tools/workspace/libfranka_v5:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
