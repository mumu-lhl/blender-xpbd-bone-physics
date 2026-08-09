import unittest
import ast
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest import mock

from blender_extension.bone_physics.authoring import ValidationIssue, ValidationSeverity
from blender_extension.bone_physics.translations import (
    critical_message_ids,
    format_issue,
    critical_operator_labels,
    translated_message_ids,
)


class TranslationTests(unittest.TestCase):
    def test_chain_operator_imports_exception_reporter(self):
        source = (
            Path(__file__).parents[2]
            / "blender_extension"
            / "bone_physics"
            / "operators"
            / "system.py"
        ).read_text(encoding="utf-8")
        tree = ast.parse(source)
        imported = {
            alias.name
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom)
            and node.level == 2
            and node.module == "translations"
            for alias in node.names
        }
        self.assertIn("report_exception", imported)

    def test_override_remove_operator_is_declared(self):
        source = (
            Path(__file__).parents[2]
            / "blender_extension"
            / "bone_physics"
            / "operators"
            / "system.py"
        ).read_text(encoding="utf-8")
        tree = ast.parse(source)
        operator_ids = {
            assignment.value.value
            for node in ast.walk(tree)
            if isinstance(node, ast.ClassDef)
            and node.name == "BONE_PHYSICS_OT_override_remove"
            for assignment in node.body
            if isinstance(assignment, ast.Assign)
            and any(
                isinstance(target, ast.Name) and target.id == "bl_idname"
                for target in assignment.targets
            )
            and isinstance(assignment.value, ast.Constant)
        }
        self.assertEqual(operator_ids, {"bone_physics.override_remove"})

    def test_translate_supports_blender_positional_context_api(self):
        calls = []

        def pgettext_iface(message, context):
            calls.append((message, context))
            return "translated"

        fake_bpy = SimpleNamespace(
            app=SimpleNamespace(
                translations=SimpleNamespace(pgettext_iface=pgettext_iface)
            )
        )
        from blender_extension.bone_physics import translations

        with mock.patch.dict(sys.modules, {"bpy": fake_bpy}):
            self.assertEqual(translations._translate("Bone Physics"), "translated")
        self.assertEqual(calls, [("Bone Physics", "*")])

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

    def test_critical_operator_labels_declared_in_source_are_translated(self):
        operators = Path(__file__).parents[2] / "blender_extension" / "bone_physics" / "operators"
        labels = set()
        for path in operators.glob("*.py"):
            for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
                if isinstance(node, ast.Assign) and any(
                    isinstance(target, ast.Name) and target.id == "bl_label" for target in node.targets
                ) and isinstance(node.value, ast.Constant) and isinstance(node.value.value, str):
                    labels.add(node.value.value)

        self.assertEqual(critical_operator_labels() - labels, set())
        self.assertTrue(critical_operator_labels() <= translated_message_ids("zh_HANS"))

    def test_operator_report_message_ids_are_translated(self):
        operators = Path(__file__).parents[2] / "blender_extension" / "bone_physics" / "operators"
        message_ids = set()
        for path in operators.glob("*.py"):
            for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
                if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
                    continue
                if node.func.id not in {"report", "report_exception"} or len(node.args) < 3:
                    continue
                message = node.args[2]
                if isinstance(message, ast.Constant) and isinstance(message.value, str):
                    message_ids.add(message.value)

        self.assertTrue(message_ids <= translated_message_ids("zh_HANS"), message_ids - translated_message_ids("zh_HANS"))

if __name__ == "__main__":
    unittest.main()
