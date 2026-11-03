#!/usr/bin/env python3
"""Apply HexaShell's changes to the RoboCasa submodule (third_party/robocasa): the patch file
scripts/patches/robocasa.patch, then the text replacement of REPLACEMENTS.  THIRD_PARTY_VERSIONS.md
lists them.

    python3 scripts/patch_third_party.py [--check]

Idempotent: a change already in place is skipped.  One that fits neither the original nor the patched
text stops the script, since the submodule is then not at the commit the change was written for.
--check only reports, and exits 1 when something is not applied yet.

scripts/patches/robodojo-layout-ids.patch is not applied here: it changes a RoboDojo checkout (see
its first lines).
"""

import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
MARKER = "[roboshell-determinism]"

#: patch file under scripts/patches -> the submodule it applies to
PATCH_FILES = (("robocasa.patch", "third_party/robocasa"),)

REPLACEMENTS = (
    (
        "third_party/robocasa/robocasa/models/fixtures/counter.py",
        "                # remove duplicates as needed\n"
        "                valid_geoms = list(set(valid_geoms))\n",
        "                # remove duplicates as needed\n"
        f"                # {MARKER} was list(set(valid_geoms)): a set of XML elements iterates in memory\n"
        "                # address order, which made the sampled counter regions differ between resets with the same seed\n"
        "                valid_geoms = list(dict.fromkeys(valid_geoms))\n",
    ),
)


def git_apply(directory, patch, *flags):
    return subprocess.run(["git", "apply", *flags, str(patch)], cwd=directory, capture_output=True).returncode == 0


def main():
    check = "--check" in sys.argv[1:]
    missing = 0
    for name, relative in PATCH_FILES:
        patch, directory = ROOT / "scripts" / "patches" / name, ROOT / relative
        if not directory.is_dir() or not any(directory.iterdir()):
            sys.exit(f"{relative} is empty: run git submodule update --init")
        if git_apply(directory, patch, "--reverse", "--check"):
            print(f"patched   {name}")
            continue
        if not git_apply(directory, patch, "--check"):
            sys.exit(f"{name}: does not apply to {relative}; is the submodule at the expected commit?")
        if check:
            print(f"unpatched {name}")
            missing += 1
            continue
        git_apply(directory, patch)
        print(f"applied   {name}")
    for relative, old, new in REPLACEMENTS:
        path = ROOT / relative
        text = path.read_text()
        if new in text:
            print(f"patched   {relative}")
            continue
        if text.count(old) != 1:
            sys.exit(f"{relative}: neither the original nor the patched text found; is this the expected version?")
        if check:
            print(f"unpatched {relative}")
            missing += 1
            continue
        path.write_text(text.replace(old, new))
        print(f"applied   {relative}")
    sys.exit(1 if missing else 0)


if __name__ == "__main__":
    main()
