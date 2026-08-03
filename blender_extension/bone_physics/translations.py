"""Stable, Blender-routable UI messages for Bone Physics.

Internal identifiers (RNA fields, enum values, stable IDs, and JSON keys) are
deliberately absent from this module.  Validation produces an issue code plus
arguments; this boundary turns that data into a localized display message.
"""

from __future__ import annotations

from typing import Any


CONTEXT = "*"

_ISSUE_MESSAGES = {
    "uuid.duplicate": "Duplicate {entity}: {value}.",
    "resource.id.invalid": "Collision resource ID is invalid.",
    "resource.stable_id.invalid": "Collision resource {entity0} has an invalid stable ID.",
    "stable_id.invalid": "Stable ID {entity0} is invalid.",
    "stable_id.duplicate": "Duplicate stable ID: {value}.",
    "chain.path.invalid": "Chain {entity0} has an invalid bone path.",
    "chain.overlap": "Chains {entity0} and {entity1} overlap at bone {bone_uuid}.",
    "target.missing": "{entity0} references missing target {entity1}.",
    "constraint.cycle.all_rigid": "System {entity0} contains an all-rigid constraint cycle.",
    "constraint.redundant": "System {entity0} contains a redundant constraint.",
    "constraint.overconstrained": "System {entity0} may be overconstrained.",
    "collision.layers.exhausted": "Collision layers exceed the limit of {limit} systems.",
    "collision.external_resource.missing": "System {entity0} has no collision resource.",
    "resource.missing": "{entity0} references missing collision resource {entity1}.",
    "collision.root.unprotected": "Chain {entity0} root is not protected from collision.",
    "collision.initial_penetration": "System {entity0} starts with a collision penetration.",
    "collision.candidates.excessive": "System {entity0} has {count} self-collision candidates.",
    "collision.weight.suspicious": "System {entity0} has suspicious collision weight on bone {bone_uuid}.",
    "collision.collection.cycle": "Collision resource collections contain a cycle.",
    "parameter.scalar.nonfinite": "{field} must be finite.",
    "parameter.scalar.invalid": "{field} is invalid.",
    "parameter.scalar.negative": "{field} must not be negative.",
    "parameter.range.inverted": "{field} lower limit must not exceed its upper limit.",
    "parameter.scalar.zero": "{field} must not be zero.",
    "parameter.range.exceeds_limit": "{field} exceeds its allowed limit.",
    "collision.radius.suspicious": "Capsule radius {radius} may be too large.",
}

_UI_MESSAGES = {
    "Bone Physics",
    "Parameters",
    "Runtime",
    "Structural Constraints",
    "Collision Resources",
    "Diagnostics",
    "Validate and Rebuild",
    "Reset Physics Bones",
    "Enable Physics",
    "Pause or Resume Physics",
    "Isolate Armature Physics",
    "Initialize Physics Configuration",
    "Add Physics System",
    "Duplicate Physics System",
    "Remove Physics System",
    "Add Chains from Selected Bones",
    "Add Structural Constraints",
    "Update Structural Constraints",
    "Remove Structural Constraints",
    "Add Collision Resource",
    "Duplicate Collision Resource",
    "Remove Collision Resource",
    "Assign Collision Resource",
    "Bind Collision Resource",
    "Enable Collision Resource",
    "Mirror Collision Resource",
    "Bone Parameter Presets",
    "Exact Armature Presets",
    "Delete Preset",
    "Delete",
    "Overwrite",
    "Overwrite Preset",
    "Copy Preset",
    "Apply Bone Parameter Preset",
    "Save Bone Parameter Preset",
    "Delete Bone Parameter Preset",
    "Copy Bone Parameter Preset",
    "Rename Bone Parameter Preset",
    "Save Exact Armature Preset",
    "Apply Exact Armature Preset",
    "Delete Exact Armature Preset",
    "Apply Endpoint Value",
    "Add Curve Point",
    "Add Selected Bone Override",
    "Remove Curve Point",
    "Select Diagnostic Target",
    "Deletes this preset permanently.",
    "Controls will appear in this workspace.",
    "No physics systems. Add one to begin.",
    "Enter Pose Mode to add chains from selected bones.",
    "No current diagnostics.",
    "Select at least one physics system",
    "Add a physics system before adding chains",
    "Add a physics system before adding an override",
    "Resource is referenced by: {references}",
    "Preset operation failed: {error}",
    "Structural constraint operation failed: {error}",
    "Physics operation failed: {error}",
    "Physics operation failed",
    "Select one branched chain head at a time",
    "Active armature changed while choosing a chain tail",
    "Select a tail for branched chain {head}",
    "Curve point positions must be strictly increasing",
    "Diagnostic target no longer exists",
    "Diagnostic target is hidden; it was not made visible",
    "Diagnostic target is not selectable",
    "Diagnostic bone requires an armature target",
    "Diagnostic bone no longer exists",
}

_CRITICAL_OPERATOR_LABELS = frozenset({
    "Apply Endpoint Value", "Add Curve Point", "Add Selected Bone Override",
    "Remove Curve Point", "Select Diagnostic Target",
})

_ZH_HANS = {
    "Bone Physics": "骨骼物理", "Parameters": "参数", "Runtime": "运行时", "Structural Constraints": "结构约束",
    "Collision Resources": "碰撞资源", "Diagnostics": "诊断", "Validate and Rebuild": "验证并重建",
    "Reset Physics Bones": "重置物理骨骼", "Enable Physics": "启用物理", "Pause or Resume Physics": "暂停或恢复物理",
    "Isolate Armature Physics": "隔离骨架物理", "Initialize Physics Configuration": "初始化物理配置",
    "Add Physics System": "添加物理系统", "Duplicate Physics System": "复制物理系统", "Remove Physics System": "移除物理系统",
    "Add Chains from Selected Bones": "从选中骨骼添加链", "Add Structural Constraints": "添加结构约束",
    "Update Structural Constraints": "更新结构约束", "Remove Structural Constraints": "移除结构约束",
    "Add Collision Resource": "添加碰撞资源", "Duplicate Collision Resource": "复制碰撞资源", "Remove Collision Resource": "移除碰撞资源",
    "Assign Collision Resource": "分配碰撞资源", "Bind Collision Resource": "绑定碰撞资源", "Enable Collision Resource": "启用碰撞资源",
    "Mirror Collision Resource": "镜像碰撞资源", "Bone Parameter Presets": "骨骼参数预设", "Exact Armature Presets": "完整骨架预设",
    "Delete Preset": "删除预设", "Delete": "删除", "Overwrite": "覆盖", "Overwrite Preset": "覆盖预设", "Copy Preset": "复制预设",
    "Apply Bone Parameter Preset": "应用骨骼参数预设", "Save Bone Parameter Preset": "保存骨骼参数预设",
    "Delete Bone Parameter Preset": "删除骨骼参数预设", "Copy Bone Parameter Preset": "复制骨骼参数预设",
    "Rename Bone Parameter Preset": "重命名骨骼参数预设", "Save Exact Armature Preset": "保存完整骨架预设",
    "Apply Exact Armature Preset": "应用完整骨架预设", "Delete Exact Armature Preset": "删除完整骨架预设",
    "Apply Endpoint Value": "应用端点值", "Add Curve Point": "添加曲线点", "Add Selected Bone Override": "添加选中骨骼覆盖",
    "Remove Curve Point": "移除曲线点", "Select Diagnostic Target": "选择诊断目标",
    "Deletes this preset permanently.": "永久删除此预设。", "Controls will appear in this workspace.": "控件将显示在此工作区中。",
    "No physics systems. Add one to begin.": "没有物理系统。请添加一个以开始。", "Enter Pose Mode to add chains from selected bones.": "进入姿态模式以从选中骨骼添加链。",
    "No current diagnostics.": "当前没有诊断信息。", "Select at least one physics system": "至少选择一个物理系统。",
    "Add a physics system before adding chains": "添加链之前请先添加物理系统。", "Add a physics system before adding an override": "添加覆盖前请先添加物理系统。",
    "Resource is referenced by: {references}": "资源被以下对象引用：{references}",
    "Preset operation failed: {error}": "预设操作失败：{error}",
    "Structural constraint operation failed: {error}": "结构约束操作失败：{error}",
    "Physics operation failed: {error}": "物理操作失败：{error}",
    "Physics operation failed": "物理操作失败",
    "Select one branched chain head at a time": "一次只能选择一个分支链头。",
    "Active armature changed while choosing a chain tail": "选择链尾时活动骨架已更改。",
    "Select a tail for branched chain {head}": "为分支链 {head} 选择一个尾端。",
    "Curve point positions must be strictly increasing": "曲线点位置必须严格递增。",
    "Diagnostic target no longer exists": "诊断目标已不存在。",
    "Diagnostic target is hidden; it was not made visible": "诊断目标被隐藏，未显示。",
    "Diagnostic target is not selectable": "诊断目标不可选择。",
    "Diagnostic bone requires an armature target": "诊断骨骼需要骨架目标。",
    "Diagnostic bone no longer exists": "诊断骨骼已不存在。",
    "Duplicate {entity}: {value}.": "重复的{entity}：{value}。", "Collision resource ID is invalid.": "碰撞资源 ID 无效。",
    "Collision resource {entity0} has an invalid stable ID.": "碰撞资源 {entity0} 的稳定 ID 无效。", "Stable ID {entity0} is invalid.": "稳定 ID {entity0} 无效。",
    "Duplicate stable ID: {value}.": "重复的稳定 ID：{value}。", "Chain {entity0} has an invalid bone path.": "链 {entity0} 的骨骼路径无效。",
    "Chains {entity0} and {entity1} overlap at bone {bone_uuid}.": "链 {entity0} 和 {entity1} 在骨骼 {bone_uuid} 处重叠。",
    "{entity0} references missing target {entity1}.": "{entity0} 引用了缺失目标 {entity1}。", "System {entity0} contains an all-rigid constraint cycle.": "系统 {entity0} 包含全刚性约束环。",
    "System {entity0} contains a redundant constraint.": "系统 {entity0} 包含冗余约束。", "System {entity0} may be overconstrained.": "系统 {entity0} 可能存在过度约束。",
    "Collision layers exceed the limit of {limit} systems.": "碰撞层超过 {limit} 个系统的限制。", "System {entity0} has no collision resource.": "系统 {entity0} 没有碰撞资源。",
    "{entity0} references missing collision resource {entity1}.": "{entity0} 引用了缺失碰撞资源 {entity1}。", "Chain {entity0} root is not protected from collision.": "链 {entity0} 的根部未受碰撞保护。",
    "System {entity0} starts with a collision penetration.": "系统 {entity0} 初始存在碰撞穿透。", "System {entity0} has {count} self-collision candidates.": "系统 {entity0} 有 {count} 个自碰撞候选。",
    "System {entity0} has suspicious collision weight on bone {bone_uuid}.": "系统 {entity0} 的骨骼 {bone_uuid} 碰撞权重可疑。", "Collision resource collections contain a cycle.": "碰撞资源集合包含循环。",
    "{field} must be finite.": "{field} 必须是有限值。", "{field} is invalid.": "{field} 无效。", "{field} must not be negative.": "{field} 不能为负。",
    "{field} lower limit must not exceed its upper limit.": "{field} 的下限不能超过上限。", "{field} must not be zero.": "{field} 不能为零。",
    "{field} exceeds its allowed limit.": "{field} 超出允许限制。", "Capsule radius {radius} may be too large.": "胶囊半径 {radius} 可能过大。",
}

CATALOG = {"zh_HANS": {(CONTEXT, message): _ZH_HANS[message] for message in _UI_MESSAGES | set(_ISSUE_MESSAGES.values())}}


def critical_message_ids() -> set[str]:
    """Return user-visible messages that must be translated before release."""
    return set(_UI_MESSAGES) | set(_ISSUE_MESSAGES.values())


def translated_message_ids(locale: str) -> set[str]:
    return {message for _context, message in CATALOG.get(locale, {})}


def critical_operator_labels() -> frozenset[str]:
    """Return source-declared operator labels that are release-critical."""
    return _CRITICAL_OPERATOR_LABELS


def _translate(message: str, *, tooltip: bool = False) -> str:
    try:
        import bpy
        method = bpy.app.translations.pgettext_tip if tooltip else bpy.app.translations.pgettext_iface
        return method(message, context=CONTEXT)
    except (AttributeError, ImportError):
        return message


def format_message(message: str, /, **arguments: object) -> str:
    """Translate an English stable message ID, then safely substitute values."""
    try:
        return _translate(message).format(**arguments)
    except (KeyError, ValueError):
        return _translate(message)


def format_issue(issue: object) -> str:
    """Format a validation issue without translating its stable code or IDs."""
    code = str(getattr(issue, "code", "authoring.validation"))
    arguments = dict(getattr(issue, "arguments", ()))
    entity_ids = tuple(str(value) for value in getattr(issue, "entity_ids", ()))
    arguments.update({f"entity{index}": value for index, value in enumerate(entity_ids)})
    arguments.setdefault("entity", entity_ids[0] if entity_ids else "item")
    arguments.setdefault("value", entity_ids[1] if len(entity_ids) > 1 else arguments["entity"])
    arguments.setdefault("field", "value")
    return format_message(_ISSUE_MESSAGES.get(code, "Validation issue: {code}"), code=code, **arguments)


def report(operator: Any, levels: set[str], message: str, /, **arguments: object) -> None:
    """Route operator reports through the same stable message catalog."""
    operator.report(levels, format_message(message, **arguments))


def report_exception(operator: Any, levels: set[str], message: str, error: Exception) -> None:
    """Preserve exception details while routing the surrounding UI message."""
    report(operator, levels, message, error=str(error))


def register_translations() -> None:
    import bpy
    bpy.app.translations.register(__name__, CATALOG)


def unregister_translations() -> None:
    import bpy
    bpy.app.translations.unregister(__name__)
