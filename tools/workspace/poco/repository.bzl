# -*- mode: python -*-
# vi: set ft=python :

def _poco_repository_impl(repository_ctx):
    # Symlink the contents of /usr directly into the repository root
    repository_ctx.symlink("/usr/include", "include")
    repository_ctx.symlink("/usr/lib", "lib")
    repository_ctx.symlink(
        Label("//tools/workspace/poco:package.BUILD.bazel"),
        "BUILD.bazel",
    )

poco_repository = repository_rule(
    implementation = _poco_repository_impl,
    local = True,
)
