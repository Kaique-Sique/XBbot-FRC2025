"""Open one pull request per outdated WPILib vendordep.

Reads JSON files in ./vendordeps, compares each version with the version
published at its jsonUrl, and opens a PR for newer versions in the same FRC
season. Vendordeps without jsonUrl are intentionally skipped.

Environment:
  GH_TOKEN     GitHub token used by gh (required)
  BASE_BRANCH  PR target branch (default: main)
  FRC_YEAR     Only accept this FRC season (default: no season filter)
"""

# Copyright (c) 2026 Megazord 7563
# All rights reserved.
#
# This file is part of the XBbot FRC robot code.
# See the LICENSE file in the project root for license terms.
import json
import os
import re
import subprocess
import sys

import requests
from packaging import version

VENDORDEP_DIR = "./vendordeps"
BASE_BRANCH = os.environ.get("BASE_BRANCH", "main")
FRC_YEAR = os.environ.get("FRC_YEAR")


def run(*cmd, check=True, capture=False):
    return subprocess.run(cmd, check=check, text=True, capture_output=capture)


def slug(value):
    """Make a value safe for a branch name."""
    return re.sub(r"[^A-Za-z0-9._-]+", "-", str(value)).strip("-")


def parse_version(value):
    try:
        return version.parse(value)
    except (version.InvalidVersion, TypeError):
        return None


def pull_request_exists(branch):
    """Return whether a PR was already created for this update branch."""
    prs = run(
        "gh", "pr", "list", "--head", branch, "--state", "all",
        "--json", "number", capture=True
    ).stdout
    return bool(json.loads(prs))


def remote_branch_exists(branch):
    remote = run(
        "git", "ls-remote", "--heads", "origin", branch, capture=True
    ).stdout
    return bool(remote.strip())


def process(filename):
    file_path = os.path.join(VENDORDEP_DIR, filename)
    with open(file_path, encoding="utf-8") as source:
        local = json.load(source)

    json_url = local.get("jsonUrl")
    if not json_url:
        print(f"{filename}: no jsonUrl; skipped")
        return

    response = requests.get(json_url, timeout=30)
    response.raise_for_status()
    remote = response.json()

    name = remote.get("name") or local.get("name") or filename
    remote_year = str(remote.get("frcYear", ""))
    if FRC_YEAR and remote_year and remote_year != FRC_YEAR:
        print(f"{name}: FRC year {remote_year} != {FRC_YEAR}; skipped")
        return

    local_version = parse_version(local.get("version"))
    remote_version = parse_version(remote.get("version"))
    if local_version is None or remote_version is None:
        raise ValueError(f"Unrecognized version format for {name}")
    if remote_version <= local_version:
        print(f"{name}: up to date ({local['version']})")
        return

    old_version, new_version = local["version"], remote["version"]
    branch = f"update-{slug(name)}-{slug(new_version)}"
    if pull_request_exists(branch):
        print(f"{name}: PR for {branch} already exists; skipped")
        return

    # Keep upstream filenames inside vendordeps.
    new_filename = os.path.basename(remote.get("fileName") or filename)
    new_path = os.path.join(VENDORDEP_DIR, new_filename)

    if remote_branch_exists(branch):
        # A previous run may have pushed the branch but failed before opening
        # its PR. Reuse that branch instead of treating it as fully handled.
        print(f"{name}: reusing existing branch {branch} to create its PR")
        run("git", "fetch", "origin", branch)
        run("git", "checkout", "-B", branch, f"origin/{branch}")
    else:
        run("git", "checkout", BASE_BRANCH)
        run("git", "checkout", "-b", branch)

        content = response.text
        if not content.endswith("\\n"):
            content += "\\n"
        with open(new_path, "w", encoding="utf-8") as destination:
            destination.write(content)

        if new_path != file_path:
            print(f"Renaming {filename} -> {new_filename}")
            os.remove(file_path)

        message = f"Bump {name} from {old_version} to {new_version}"
        print(message)
        run("git", "add", "-A", VENDORDEP_DIR)
        run("git", "commit", "-m", message)
        run("git", "push", "-u", "origin", branch)

    message = f"Bump {name} from {old_version} to {new_version}"
    run(
        "gh", "pr", "create",
        "--title", message,
        "--base", BASE_BRANCH,
        "--head", branch,
        "--body",
        (
            f"### Automated update for {name}\\n"
            f"- **Old version:** {old_version}\\n"
            f"- **New version:** {new_version}\\n"
            f"- **Source:** {json_url}\\n\\n"
            "Run the build and test on a robot or in simulation before merging."
        ),
    )


def main():
    failures = 0
    for filename in sorted(os.listdir(VENDORDEP_DIR)):
        if not filename.endswith(".json"):
            continue
        try:
            process(filename)
        except Exception as error:
            failures += 1
            print(f"ERROR in {filename}: {error}", file=sys.stderr)
        finally:
            run("git", "checkout", "-f", BASE_BRANCH, check=False)

    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
