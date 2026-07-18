# -*- coding: utf-8 -*-

"""Preview-only semantic library for the unified GUI (M63-B).

Maps a recognized symbol label / candidate to a richer, human-readable *visible
task* description so the dashboard can show what a stroke would mean, while
staying strictly preview-only. This module NEVER dispatches a task, NEVER
publishes ``/colmag/task_command`` or ``/colmag/confirmed_label``, and NEVER
touches Gazebo or FR3. It only produces display text.

Design:

  * ``VISIBLE_TASKS`` is a data-driven catalog keyed by a neutral task id. Each
    entry carries a title, English + Chinese descriptions, a ``safety_class`` and
    a ``source`` tag. Extend it by adding rows; no code changes required.
  * ``symbol_semantics`` (label -> task/intent) stays the single source of truth
    for label meaning; this library reuses its ``task`` field to pick a visible
    task, then falls back to a label-keyed table for symbols it does not map.
  * Only a documented subset is marked dispatchable for a *future* gazebo-enabled
    mode (M63-C). In M63-B nothing is dispatchable; the flag is display metadata.

Safety classes:

    SAFE     - benign preview shapes / motions
    REVIEW   - manipulation that needs a human check before any future dispatch
    CRITICAL - stop / clear / compliance: must be handled explicitly later
"""

from colmag_ros.symbol_semantics import get_symbol_semantics

SAFE = "SAFE"
REVIEW = "REVIEW"
CRITICAL = "CRITICAL"


def _task(title, description, description_zh, safety_class,
          source="semantic_library", dispatchable_future=False):
    return {
        "title": title,
        "description": description,
        "description_zh": description_zh,
        "safety_class": safety_class,
        "source": source,
        "dispatchable_future": bool(dispatchable_future),
    }


# Data-driven visible-task catalog. Neutral task ids; no person names.
VISIBLE_TASKS = {
    "DRAW_HEXAGON": _task(
        "Draw Hexagon", "Trace a hexagon outline.", "绘制六边形轮廓。",
        SAFE, dispatchable_future=True),
    "DRAW_STAR": _task(
        "Draw Star", "Trace a five-point star.", "绘制五角星。", SAFE),
    "TRACE_LETTER": _task(
        "Trace Letter", "Trace the recognized letter shape.", "描摹识别到的字母。",
        SAFE),
    "FOLLOW_PATH": _task(
        "Follow Path", "Follow the drawn free path.", "沿绘制的自由路径运动。", SAFE),
    "WAVE": _task(
        "Wave", "Perform a friendly wave motion.", "做一个挥手动作。", SAFE),
    "HOME_PREVIEW": _task(
        "Home (preview)", "Preview returning to the home pose.", "预览回到初始位姿。",
        SAFE),
    "PICK_PLACE": _task(
        "Pick & Place", "Pick an object and place it at the target.",
        "抓取物体并放置到目标处。", REVIEW),
    "SORT_OBJECTS": _task(
        "Sort Objects", "Sort objects by class into bins.", "按类别将物体分拣入槽。",
        REVIEW),
    "STACK_BLOCKS": _task(
        "Stack Blocks", "Stack blocks into a tower.", "将积木堆叠成塔。", REVIEW),
    "OPEN_DRAWER": _task(
        "Open Drawer", "Open a drawer handle.", "打开抽屉把手。", REVIEW),
    "PRESS_BUTTON": _task(
        "Press Button", "Press a physical button.", "按下一个物理按钮。", REVIEW),
    "INSPECT_OBJECT": _task(
        "Inspect Object", "Move the camera to inspect an object.",
        "移动相机以检查物体。", REVIEW),
    "AVOID_OBSTACLE": _task(
        "Avoid Obstacle", "Plan a motion that avoids an obstacle.",
        "规划避开障碍物的运动。", REVIEW),
    "COMPLIANT_TOUCH": _task(
        "Compliant Touch", "Switch to a compliant, force-limited touch.",
        "切换到柔顺、限力的接触模式。", CRITICAL),
    "STOP": _task(
        "Stop", "Stop all motion immediately.", "立即停止所有运动。", CRITICAL),
    "CLEAR": _task(
        "Clear", "Clear the current segment and candidates.",
        "清除当前轨迹段与候选。", CRITICAL),
    "UNKNOWN": _task(
        "Unknown", "No known visible task for this symbol.",
        "该符号没有已知的可见任务。", REVIEW),
}

# Bridge symbol_semantics task tags -> visible task ids.
_SEMANTIC_TASK_TO_VISIBLE = {
    "HEXAGON_TRAJECTORY": "DRAW_HEXAGON",
    "PICK_PLACE": "PICK_PLACE",
    "COMPLIANT_CONTROL": "COMPLIANT_TOUCH",
    "AVOID_OBSTACLE": "AVOID_OBSTACLE",
    "STOP": "STOP",
    "MOVE_LEFT": "FOLLOW_PATH",
    "MOVE_RIGHT": "FOLLOW_PATH",
    "MOVE_UP": "FOLLOW_PATH",
    "MOVE_DOWN": "FOLLOW_PATH",
    "NEUTRAL": "UNKNOWN",
}

# Direct label -> visible task, for symbols not covered by symbol_semantics.
_LABEL_TO_VISIBLE = {
    "0": "DRAW_HEXAGON",
    "5": "DRAW_STAR",
    "8": "STACK_BLOCKS",
    "H": "HOME_PREVIEW",
    "W": "WAVE",
    "P": "PRESS_BUTTON",
    "O": "OPEN_DRAWER",
    "I": "INSPECT_OBJECT",
}


def visible_task_for_label(label):
    """Return ``(task_id, task_dict)`` for a canonical symbol label.

    Uses ``symbol_semantics`` first (single source of truth), then a direct
    label table, then ``UNKNOWN``. Never raises.
    """
    key = str(label or "").strip().upper()
    if not key:
        return "UNKNOWN", VISIBLE_TASKS["UNKNOWN"]

    semantics = get_symbol_semantics(key) or get_symbol_semantics(str(label))
    if semantics:
        task_id = _SEMANTIC_TASK_TO_VISIBLE.get(semantics.get("task"))
        if task_id and task_id in VISIBLE_TASKS:
            return task_id, VISIBLE_TASKS[task_id]

    task_id = _LABEL_TO_VISIBLE.get(key)
    if task_id and task_id in VISIBLE_TASKS:
        return task_id, VISIBLE_TASKS[task_id]

    return "UNKNOWN", VISIBLE_TASKS["UNKNOWN"]


def describe_candidate(candidate):
    """Return a preview-only semantic dict for a recognizer candidate.

    ``candidate`` is a ``{label, confidence, ...}`` dict. The result carries the
    label, visible-task id/title/description(+zh), safety class, source and
    confidence. Purely descriptive; the caller only displays it.
    """
    label = ""
    confidence = None
    if isinstance(candidate, dict):
        label = str(candidate.get("label", "") or "")
        conf = candidate.get("confidence")
        confidence = float(conf) if isinstance(conf, (int, float)) else None
    task_id, task = visible_task_for_label(label)
    return {
        "label": label,
        "task_id": task_id,
        "title": task["title"],
        "description": task["description"],
        "description_zh": task["description_zh"],
        "safety_class": task["safety_class"],
        "source": task["source"],
        "dispatchable_future": task["dispatchable_future"],
        "confidence": confidence,
    }


def preview_line(candidate):
    """One-line GUI string for a candidate's semantic meaning (preview only)."""
    d = describe_candidate(candidate)
    label = d["label"] or "?"
    if d["confidence"] is not None:
        return "%s -> %s [%s] (conf %.2f) — preview only" % (
            label, d["title"], d["safety_class"], d["confidence"])
    return "%s -> %s [%s] — preview only" % (label, d["title"], d["safety_class"])
