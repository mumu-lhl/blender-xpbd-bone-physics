"""Build the fail-closed, exact-commit V1 acceptance artifact."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Iterable

from scripts.ci.aggregate_acceptance import (
    REQUIRED_CRITERIA,
    SUPPORTED_PLATFORMS,
    aggregate_acceptance,
)


def _require_commit(report: dict[str, Any], commit: str, label: str) -> None:
    if report.get("commit") != commit:
        raise ValueError(f"{label} commit does not match requested commit")


def _validate_reference(reference: dict[str, Any], commit: str) -> None:
    if not isinstance(reference, dict):
        raise ValueError("reference report is required")
    _require_commit(reference, commit, "reference report")
    if reference.get("backend") != "native":
        raise ValueError("reference report must use the native backend")
    if reference.get("run_count") != 60 or len(reference.get("runs", ())) != 60:
        raise ValueError("reference report is incomplete")
    if reference.get("failures"):
        raise ValueError("reference report has failures")


def _validate_platform(report: dict[str, Any], commit: str) -> None:
    platform = report.get("platform")
    if platform not in SUPPORTED_PLATFORMS:
        raise ValueError(f"unsupported or missing report platform: {platform!r}")
    _require_commit(report, commit, f"{platform} report")
    if report.get("status") != "ok":
        raise ValueError(f"package report failed on {platform}")
    criteria = report.get("criteria")
    if not isinstance(criteria, dict) or set(criteria) != set(REQUIRED_CRITERIA):
        raise ValueError(f"acceptance criteria are incomplete on {platform}")
    if not all(value is True for value in criteria.values()):
        raise ValueError(f"acceptance criteria are not all passing on {platform}")
    if report.get("verification", {}).get("native_count") != 1:
        raise ValueError(f"package native module evidence is incomplete on {platform}")
    dependencies = report.get("dependencies", {})
    if dependencies.get("status") != "ok" or dependencies.get("unexpected"):
        raise ValueError(f"package dependency evidence is incomplete on {platform}")
    if report.get("load_test", {}).get("status") != "ok":
        raise ValueError(f"package load evidence is incomplete on {platform}")


def derive_platform_reports(
    platform_reports: Iterable[dict[str, Any]],
    reference: dict[str, Any],
    ci: dict[str, Any],
    *,
    commit: str,
) -> list[dict[str, Any]]:
    """Attach the global CI/reference criteria to raw package reports."""
    _validate_reference(reference, commit)
    required_ci = {"rust_tests", "workspace_clippy", "blender_headless", "documentation"}
    if set(ci) != required_ci or not all(value is True for value in ci.values()):
        raise ValueError("CI acceptance criteria are incomplete")
    reference_ok = {
        "reference_matrix": reference["run_count"] == 60,
        "equal_time_matrix": all(
            run.get("metrics", {}).get("equal_time_position_error", 0.0) <= 0.00005
            and run.get("metrics", {}).get("equal_time_angle_error", 0.0) <= 0.001
            for run in reference["runs"]
        ),
        "repeatability": all(
            run.get("metrics", {}).get("repeat_position_error", 0.0) <= 0.000001
            and run.get("metrics", {}).get("repeat_angle_error", 0.0) <= 0.00001
            for run in reference["runs"]
        ),
    }
    result = []
    for raw in platform_reports:
        platform = raw.get("platform")
        _require_commit(raw, commit, f"{platform} report")
        if raw.get("status") != "ok":
            raise ValueError(f"package report failed on {platform}")
        package_ok = {
            "native_smoke": raw.get("load_test", {}).get("status") == "ok",
            "package_structure": raw.get("verification", {}).get("native_count") == 1,
            "native_dependencies": (
                raw.get("dependencies", {}).get("status") == "ok"
                and not raw.get("dependencies", {}).get("unexpected")
            ),
            "wheel_contract": raw.get("wheel_build", {}).get("status") == "ok",
        }
        criteria = {
            "rust_tests": ci["rust_tests"],
            "workspace_clippy": ci["workspace_clippy"],
            "blender_headless": ci["blender_headless"],
            **reference_ok,
            **package_ok,
            "documentation": ci["documentation"],
        }
        normalized = dict(raw)
        normalized["criteria"] = criteria
        normalized["observed"] = {
            "reference_run_count": reference["run_count"],
            "package_status": raw["status"],
            "native_dependencies": raw["dependencies"].get("dependencies", []),
        }
        normalized["allowed"] = {
            "reference_run_count": 60,
            "native_dependencies": (),
        }
        result.append(normalized)
    return result


def build_v1_acceptance(
    platform_reports: Iterable[dict[str, Any]],
    reference: dict[str, Any] | None,
    *,
    commit: str,
    github_sha: str | None,
    create_draft: bool = False,
) -> dict[str, Any]:
    """Validate all evidence and return the canonical acceptance artifact."""
    if not reference:
        raise ValueError("reference report is required")
    _validate_reference(reference, commit)
    reports = list(platform_reports)
    for report in reports:
        _validate_platform(report, commit)
    platforms = [report.get("platform") for report in reports]
    if len(platforms) != len(set(platforms)):
        raise ValueError("duplicate acceptance report platform")
    if set(platforms) != set(SUPPORTED_PLATFORMS):
        raise ValueError("complete platform evidence is required")
    return aggregate_acceptance(
        reports,
        commit=commit,
        github_sha=github_sha,
        create_draft=create_draft,
    ) | {
        "reference": {
            "source": "native-reference.json",
            "backend": reference["backend"],
            "run_count": reference["run_count"],
            "failure_count": len(reference["failures"]),
            "commit": commit,
        }
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--commit", required=True)
    parser.add_argument("--github-sha", required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--report", action="append", required=True, help="platform=report.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--failure-snapshot", type=Path)
    parser.add_argument("--ci-report", type=Path)
    parser.add_argument("--create-draft", action="store_true")
    args = parser.parse_args()
    try:
        reports = []
        for item in args.report:
            platform, separator, filename = item.partition("=")
            if not separator:
                raise ValueError("--report must be platform=path")
            report = json.loads(Path(filename).read_text(encoding="utf-8"))
            report["platform"] = platform
            reports.append(report)
        reference = json.loads(args.reference.read_text(encoding="utf-8"))
        ci = None
        if args.ci_report is not None:
            ci = json.loads(args.ci_report.read_text(encoding="utf-8"))
            reports = derive_platform_reports(reports, reference, ci, commit=args.commit)
        artifact = build_v1_acceptance(
            reports,
            reference,
            commit=args.commit,
            github_sha=args.github_sha,
            create_draft=args.create_draft,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(artifact, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return 0
    except Exception as error:
        snapshot = {
            "commit": args.commit,
            "github_sha": args.github_sha,
            "status": "failed",
            "error": f"{type(error).__name__}: {error}",
        }
        destination = args.failure_snapshot or args.output.with_name("failure-snapshot.json")
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(snapshot, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
