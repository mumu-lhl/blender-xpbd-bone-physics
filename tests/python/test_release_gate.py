import unittest

from scripts.ci.aggregate_acceptance import REQUIRED_CRITERIA, SUPPORTED_PLATFORMS, aggregate_acceptance


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


if __name__ == "__main__":
    unittest.main()
