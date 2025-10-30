load("@rules_python//python:py_test.bzl", "py_test")

def _bazel_lint():
    files = native.glob(["*.bazel", "*.bzl"], allow_empty = True)
    if len(files) == 0:
        return
    buildifier = "@buildifier_prebuilt//:buildifier"
    locations = ["$(locations %s)" % f for f in files]
    py_test(
        name = "buildifier_lint",
        srcs = ["//tools/lint:buildifier.py"],
        main = "buildifier.py",
        data = files + [buildifier],
        args = ["$(location %s)" % buildifier, "-mode=check"] + locations,
        tags = ["buildifier_lint", "lint"],
    )

def add_lint_tests():
    _bazel_lint()
