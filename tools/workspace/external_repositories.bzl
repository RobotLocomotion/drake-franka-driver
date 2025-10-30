# -*- mode: python -*-
# vi: set ft=python :

"""Module extension for external dependencies."""

load("@drake//tools/workspace:mirrors.bzl", "DEFAULT_MIRRORS")
load("//tools/workspace/drake_models:repository.bzl", "drake_models_repository")
load("//tools/workspace/libfranka_v4:repository.bzl", "libfranka_v4_repository")
load("//tools/workspace/libfranka_v5:repository.bzl", "libfranka_v5_repository")
load("//tools/workspace/libfranka_common_v4:repository.bzl", "libfranka_common_v4_repository")
load("//tools/workspace/libfranka_common_v5:repository.bzl", "libfranka_common_v5_repository")

def _external_repositories_impl(ctx):
    """Implementation of the external_repositories module extension."""
    drake_models_repository(name = "drake_models", mirrors = DEFAULT_MIRRORS)
    libfranka_v4_repository(name = "libfranka_v4", mirrors = DEFAULT_MIRRORS)
    libfranka_v5_repository(name = "libfranka_v5", mirrors = DEFAULT_MIRRORS)
    libfranka_common_v4_repository(name = "libfranka_common_v4", mirrors = DEFAULT_MIRRORS)
    libfranka_common_v5_repository(name = "libfranka_common_v5", mirrors = DEFAULT_MIRRORS)

external_repositories = module_extension(
    implementation = _external_repositories_impl,
)