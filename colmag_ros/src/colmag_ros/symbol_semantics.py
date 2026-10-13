# -*- coding: utf-8 -*-

"""
Symbol semantics vocabulary for OCR and trajectory recognition.
Maps canonical labels to display names, tasks, and command intents.

For the 8 C4 labels the task must match execution_semantics.C4_LABEL_TO_TASK,
which is what the dispatcher executes. Display only; nothing here dispatches.
"""

SYMBOL_SEMANTICS = {
    "1": {
        "label": "1",
        "display_name": "Move Left",
        "task": "MOVE_LEFT",
        "command_intent": "run_move_left",
        "description": "Move the robot to the left.",
        "description_zh": "控制机械臂向左移动。",
        "safety_critical": False,
    },
    "2": {
        "label": "2",
        "display_name": "Hexagon Trajectory",
        "task": "HEXAGON_TRAJECTORY",
        "command_intent": "run_hexagon",
        "description": "Run the hexagon trajectory demo after confirmation.",
        "description_zh": "确认后执行六边形轨迹演示。",
        "safety_critical": False,
    },
    "3": {
        "label": "3",
        "display_name": "Move Right",
        "task": "MOVE_RIGHT",
        "command_intent": "run_move_right",
        "description": "Move the robot to the right.",
        "description_zh": "控制机械臂向右移动。",
        "safety_critical": False,
    },
    "6": {
        "label": "6",
        "display_name": "Move Down",
        "task": "MOVE_DOWN",
        "command_intent": "run_move_down",
        "description": "Move the robot down.",
        "description_zh": "控制机械臂向下移动。",
        "safety_critical": False,
    },
    "V": {
        "label": "V",
        "display_name": "Hover Approach",
        "task": "HOVER_APPROACH",
        "command_intent": "run_hover_approach",
        "description": "Non-contact hover/approach preview in Gazebo.",
        "description_zh": "在 Gazebo 中预览非接触的悬停接近动作。",
        "safety_critical": False,
    },
    "O": {
        "label": "O",
        "display_name": "Small Orbit",
        "task": "ORBIT_SMALL",
        "command_intent": "run_orbit_small",
        "description": "Small non-contact orbit preview in Gazebo.",
        "description_zh": "在 Gazebo 中预览小范围非接触环绕动作。",
        "safety_critical": False,
    },
    "A": {
        "label": "A",
        "display_name": "Home or Ready",
        "task": "HOME_OR_READY",
        "command_intent": "run_home_or_ready",
        "description": "Move toward a neutral ready pose in Gazebo.",
        "description_zh": "在 Gazebo 中移动到中性准备位姿。",
        "safety_critical": False,
    },
    "C": {
        "label": "C",
        "display_name": "Soft Descend Preview",
        "task": "SOFT_DESCEND_PREVIEW",
        "command_intent": "run_soft_descend_preview",
        "description": "Non-contact soft-descend preview; no compliance or contact control.",
        "description_zh": "非接触的缓降预览；不做柔顺或接触控制。",
        "safety_critical": False,
    },
    "S": {
        "label": "S",
        "display_name": "Stop",
        "task": "STOP",
        "command_intent": "run_stop",
        "description": "Stop current movement.",
        "description_zh": "停止当前所有运动。",
        "safety_critical": True,
    },
    "X": {
        "label": "X",
        "display_name": "Stop or Cancel",
        "task": "STOP_OR_CANCEL",
        "command_intent": "run_stop_or_cancel",
        "description": "Cancel the current task; no motion.",
        "description_zh": "取消当前任务，不产生运动。",
        "safety_critical": False,
    },
    "N": {
        "label": "N",
        "display_name": "Neutral / Reserved",
        "task": "NEUTRAL",
        "command_intent": "no_op",
        "description": "Reserved neutral symbol.",
        "description_zh": "保留的无操作符号。",
        "safety_critical": False,
    }
}

def get_symbol_semantics(label):
    """
    Get the semantics dict for a given canonical label.
    Returns None if the label is unknown.
    """
    return SYMBOL_SEMANTICS.get(label)

def enrich_candidate_with_semantics(candidate):
    """
    Enrich a candidate dictionary with semantic fields.
    Mutates the candidate in place and returns it.
    """
    label = candidate.get("label")
    semantics = get_symbol_semantics(label)
    if semantics:
        for k, v in semantics.items():
            if k != "label" and k not in candidate:
                candidate[k] = v
    return candidate
