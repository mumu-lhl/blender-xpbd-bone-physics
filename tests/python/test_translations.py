import unittest

from blender_extension.bone_physics.authoring import ValidationIssue, ValidationSeverity
from blender_extension.bone_physics.translations import (
    critical_message_ids,
    format_issue,
    translated_message_ids,
)


class TranslationTests(unittest.TestCase):
    def test_every_critical_message_has_simplified_chinese_translation(self):
        missing = critical_message_ids() - translated_message_ids("zh_HANS")

        self.assertEqual(missing, set())

    def test_json_and_enum_identifiers_are_not_translation_keys(self):
        translated = translated_message_ids("zh_HANS")

        self.assertNotIn("gravity_scale", translated)
        self.assertNotIn("SWING_TWIST", translated)

    def test_validation_issue_uses_stable_code_and_named_arguments(self):
        issue = ValidationIssue(
            "chain.overlap",
            ValidationSeverity.ERROR,
            ("hair-a", "hair-b"),
            (("bone_uuid", "shared-bone"),),
        )

        self.assertEqual(
            format_issue(issue),
            "Chains hair-a and hair-b overlap at bone shared-bone.",
        )


if __name__ == "__main__":
    unittest.main()
