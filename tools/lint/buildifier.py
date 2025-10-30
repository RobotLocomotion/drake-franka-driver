#!/usr/bin/env python3
"""Wrapper script for buildifier that provides helpful error messages."""

import subprocess
import sys

def main():
    # Find the actual buildifier binary (passed as first argument by Bazel)
    if len(sys.argv) < 2:
        print("Error: buildifier binary path not provided", file=sys.stderr)
        return 1

    buildifier_bin = sys.argv[1]
    args = sys.argv[2:]

    # Run buildifier
    result = subprocess.run([buildifier_bin] + args, capture_output=False)

    if result.returncode != 0:
        # Print helpful message on failure
        print("\n" + "="*80, file=sys.stderr)
        print("BUILDIFIER FORMATTING FAILED", file=sys.stderr)
        print("="*80, file=sys.stderr)
        print("\nYour Bazel files need to be reformatted.", file=sys.stderr)
        print("\nTo fix this, run:", file=sys.stderr)
        print("  bazel run @buildifier_prebuilt//:buildifier -- -r $(pwd)", file=sys.stderr)
        print("\nOr for a specific file:", file=sys.stderr)
        print("  bazel run @buildifier_prebuilt//:buildifier -- path/to/file.bazel", file=sys.stderr)
        print("\n" + "="*80 + "\n", file=sys.stderr)

    return result.returncode

if __name__ == "__main__":
    sys.exit(main())