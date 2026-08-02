"""Aggregate first-version evidence and fail closed for draft RC creation."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Iterable

REQUIRED_CRITERIA = (
    "rust_tests",
    "workspace_clippy",
    "blender_headless",
    "reference_matrix",
    "equal_time_matrix",
    "repeatability",
    "native_smoke",
    "package_structure",
    "native_dependencies",
    "wheel_contract",
    "documentation",
)
SUPPORTED_PLATFORMS = ("linux-x86_64", "windows-x86_64", "macos-arm64", "macos-x86_64")


def aggregate_acceptance(
    reports: Iterable[dict[str, Any]],
    *,
    commit: str,
    github_sha: str | None,
    create_draft: bool,
) -> dict[str, Any]:
    reports = list(reports)
    if not reports:
        raise ValueError("at least one acceptance report is required")
    records = []
    platforms = set()
    for report in reports:
        platform = report.get("platform")
        if platform not in SUPPORTED_PLATFORMS:
            raise ValueError(f"unsupported or missing report platform: {platform!r}")
        if platform in platforms:
            raise ValueError(f"duplicate acceptance report platform: {platform}")
        if report.get("commit") != commit:
            raise ValueError("acceptance report commit does not match requested commit")
        if report.get("failures"):
            raise ValueError(f"acceptance report failed on {platform}")
        criteria = report.get("criteria")
        if not isinstance(criteria, dict) or set(criteria) != set(REQUIRED_CRITERIA):
            raise ValueError(f"acceptance criteria are incomplete on {platform}")
        if not all(value is True for value in criteria.values()):
            raise ValueError(f"acceptance criteria are not all passing on {platform}")
        platforms.add(platform)
        for criterion in REQUIRED_CRITERIA:
            records.append(
                {
                    "criterion": criterion,
                    "source": report.get("source", f"acceptance-{platform}.json"),
                    "observed": report.get("observed", {}).get(criterion),
                    "allowed": report.get("allowed", {}).get(criterion),
                    "commit": commit,
                    "platform": platform,
                    "pass": True,
                }
            )
    draft_allowed = (
        create_draft
        and github_sha == commit
        and set(SUPPORTED_PLATFORMS).issubset(platforms)
    )
    if create_draft and not set(SUPPORTED_PLATFORMS).issubset(platforms):
        raise ValueError("complete platform evidence is required for draft creation")
    return {
        "schema_version": 1,
        "commit": commit,
        "platforms": sorted(platforms),
        "criteria": records,
        "success": True,
        "draft_allowed": draft_allowed,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--commit", required=True)
    parser.add_argument("--github-sha", required=True)
    parser.add_argument("--create-draft", action="store_true")
    parser.add_argument("--report", action="append", required=True, help="platform=report.json")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    reports = []
    for item in args.report:
        platform, _, filename = item.partition("=")
        if not platform or not filename:
            raise SystemExit("--report must be platform=path")
        report = json.loads(Path(filename).read_text(encoding="utf-8"))
        report["platform"] = platform
        reports.append(report)
    artifact = aggregate_acceptance(
        reports,
        commit=args.commit,
        github_sha=args.github_sha,
        create_draft=args.create_draft,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(artifact, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if artifact["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
