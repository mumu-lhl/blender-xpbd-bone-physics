"""Blender Simplified Chinese translation smoke; run by the bounded Blender harness."""

import bpy

import bone_physics


preferences = bpy.context.preferences.view
previous_language = preferences.language
previous_translate_interface = preferences.use_translate_interface
previous_translate_tooltips = preferences.use_translate_tooltips

try:
    bone_physics.register()
    preferences.language = "zh_HANS"
    preferences.use_translate_interface = True
    preferences.use_translate_tooltips = True

    assert bpy.app.translations.pgettext_iface("Bone Physics") == "骨骼物理"
    assert bpy.app.translations.pgettext_iface("Validate and Rebuild") == "验证并重建"
    assert bpy.app.translations.pgettext_tip("Deletes this preset permanently.") == "永久删除此预设。"
finally:
    preferences.language = previous_language
    preferences.use_translate_interface = previous_translate_interface
    preferences.use_translate_tooltips = previous_translate_tooltips
    bone_physics.unregister()

print('{"status": "ok", "test": "ui_translations"}')
