# -*- python -*-

workspace(name = "drake_franka_driver")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

(DRAKE_COMMIT, DRAKE_CHECKSUM) = (
    "v1.21.0",
    "6571295843aff8e11620340739bf5eab7a25130f8f06667b2d3e6df85567509a",
)

DRAKE_STRIP_PREFIX = "drake-1.21.0"
# If using commit vs. a tag, uncomment below.
# DRAKE_STRIP_PREFIX = "drake-v{}".format(DRAKE_COMMIT)

# Before changing the COMMIT, temporarily uncomment the next line so that Bazel
# displays the suggested new value for the CHECKSUM.
# DRAKE_CHECKSUM = "0" * 64

# Or to build against a local checkout of Drake, at the bash prompt set an
# environment variable before building:
#  export FRANKA_LOCAL_DRAKE_PATH=/home/user/stuff/drake

# Load an environment variable.
load("//:environ.bzl", "environ_repository")

environ_repository(
    name = "environ",
    vars = ["FRANKA_LOCAL_DRAKE_PATH"],
)

load("@environ//:environ.bzl", "FRANKA_LOCAL_DRAKE_PATH")

# This declares the `@drake` repository as an http_archive from github,
# iff FRANKA_LOCAL_DRAKE_PATH is unset.  When it is set, this declares a
# `@drake_ignored` package which is never referenced, and thus is ignored.
http_archive(
    name = "drake" if not FRANKA_LOCAL_DRAKE_PATH else "drake_ignored",
    sha256 = DRAKE_CHECKSUM,
    strip_prefix = DRAKE_STRIP_PREFIX,
    urls = [x.format(DRAKE_COMMIT) for x in [
        "https://github.com/RobotLocomotion/drake/archive/{}.tar.gz",
    ]],
)

# This declares the `@drake` repository as a local directory,
# iff FRANKA_LOCAL_DRAKE_PATH is set.  When it is unset, this declares a
# `@drake_ignored` package which is never referenced, and thus is ignored.
local_repository(
    name = "drake" if FRANKA_LOCAL_DRAKE_PATH else "drake_ignored",
    path = FRANKA_LOCAL_DRAKE_PATH,
)

print("Using FRANKA_LOCAL_DRAKE_PATH={}".format(FRANKA_LOCAL_DRAKE_PATH)) if FRANKA_LOCAL_DRAKE_PATH else None  # noqa

load("@drake//tools/workspace:default.bzl", "add_default_workspace")
add_default_workspace()


load("@drake//tools/workspace:mirrors.bzl", "DEFAULT_MIRRORS")
load("//tools/workspace/libfranka_v4:repository.bzl", "libfranka_v4_repository")
load("//tools/workspace/libfranka_v5:repository.bzl", "libfranka_v5_repository")
load("//tools/workspace/libfranka_common_v4:repository.bzl", "libfranka_common_v4_repository")  
load("//tools/workspace/libfranka_common_v5:repository.bzl", "libfranka_common_v5_repository")
load("//tools/workspace/poco:repository.bzl", "poco_repository")

libfranka_v4_repository(name = "libfranka_v4", mirrors = DEFAULT_MIRRORS)
libfranka_v5_repository(name = "libfranka_v5", mirrors = DEFAULT_MIRRORS)
libfranka_common_v4_repository(name = "libfranka_common_v4", mirrors = DEFAULT_MIRRORS)
libfranka_common_v5_repository(name = "libfranka_common_v5", mirrors = DEFAULT_MIRRORS)
poco_repository(name = "poco")