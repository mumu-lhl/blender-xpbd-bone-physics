import unittest

from scripts.ci.aggregate_acceptance import REQUIRED_CRITERIA, SUPPORTED_PLATFORMS, aggregate_acceptance
from scripts.ci.build_v1_acceptance import build_v1_acceptance


def report(platform, commit="abc123", failures=None):
    return {
        "platform": platform,
        "commit": commit,
        "failures": [] if failures is None else failures,
        "criteria": {criterion: True for criterion in REQUIRED_CRITERIA},
        "observed": {},
        "allowed": {},
    }


class ReleaseGateTests(unittest.TestCase):
    def test_missing_platform_evidence_is_rejected_for_draft(self):
        with self.assertRaisesRegex(ValueError, "platform"):
            aggregate_acceptance([report("linux-x86_64")], commit="abc123", github_sha="abc123", create_draft=True)

    def test_mismatched_commit_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "commit"):
            aggregate_acceptance([report("linux-x86_64", "old")], commit="abc123", github_sha="abc123", create_draft=False)

    def test_failed_numerical_report_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "failed"):
            aggregate_acceptance([report("linux-x86_64", failures=[{"id": "ccd-1"}])], commit="abc123", github_sha="abc123", create_draft=False)

    def test_complete_matching_matrix_allows_draft_only_with_explicit_flag(self):
        reports = [report(platform) for platform in SUPPORTED_PLATFORMS]
        artifact = aggregate_acceptance(reports, commit="abc123", github_sha="abc123", create_draft=True)
        self.assertTrue(artifact["draft_allowed"])
        artifact = aggregate_acceptance(reports, commit="abc123", github_sha="other", create_draft=True)
        self.assertFalse(artifact["draft_allowed"])

    def test_builder_rejects_missing_reference_report(self):
        with self.assertRaisesRegex(ValueError, "reference"):
            build_v1_acceptance([], None, commit="abc123", github_sha="abc123")

    def test_builder_rejects_reference_commit_mismatch(self):
        reference = {"commit": "old", "backend": "native", "run_count": 60, "failures": [], "runs": [{}] * 60}
        with self.assertRaisesRegex(ValueError, "commit"):
            build_v1_acceptance([], reference, commit="abc123", github_sha="abc123")

    def test_builder_rejects_missing_platform_criteria(self):
        reference = {"commit": "abc123", "backend": "native", "run_count": 60, "failures": [], "runs": [{}] * 60}
        report = {"platform": "linux-x86_64", "commit": "abc123", "status": "ok"}
        with self.assertRaisesRegex(ValueError, "criteria"):
            build_v1_acceptance([report], reference, commit="abc123", github_sha="abc123")

    def test_builder_emits_all_criteria_for_complete_matrix(self):
        reference = {"commit": "abc123", "backend": "native", "run_count": 60, "failures": [], "runs": [{}] * 60}
        reports = []
        for platform in SUPPORTED_PLATFORMS:
            reports.append(
                {
                    "platform": platform,
                    "commit": "abc123",
                    "status": "ok",
                    "package": {"native_count": 1},
                    "verification": {"native_count": 1},
                    "dependencies": {"status": "ok", "unexpected": []},
                    "load_test": {"status": "ok"},
                    "criteria": {criterion: True for criterion in REQUIRED_CRITERIA},
                }
            )
        artifact = build_v1_acceptance(reports, reference, commit="abc123", github_sha="abc123")
        self.assertTrue(artifact["success"])
        self.assertEqual(len(artifact["criteria"]), len(REQUIRED_CRITERIA) * len(SUPPORTED_PLATFORMS))

    def test_builder_rejects_duplicate_platform_evidence(self):
        reference = {"commit": "abc123", "backend": "native", "run_count": 60, "failures": [], "runs": [{}] * 60}
        report = {
            "platform": "linux-x86_64",
            "commit": "abc123",
            "status": "ok",
            "verification": {"native_count": 1},
            "dependencies": {"status": "ok", "unexpected": []},
            "load_test": {"status": "ok"},
            "criteria": {criterion: True for criterion in REQUIRED_CRITERIA},
        }
        with self.assertRaisesRegex(ValueError, "duplicate"):
            build_v1_acceptance([report, dict(report)], reference, commit="abc123", github_sha="abc123")


if __name__ == "__main__":
    unittest.main()
