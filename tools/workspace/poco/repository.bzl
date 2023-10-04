# -*- mode: python -*-
# vi: set ft=python :

def poco_repository(
        name):
    native.new_local_repository(
        name = name,
        path = "/usr",
        build_file = "//tools/workspace/poco:package.BUILD.bazel",
    )
