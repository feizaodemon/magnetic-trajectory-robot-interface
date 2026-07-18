# -*- coding: utf-8 -*-

"""
Symbol semantics vocabulary for OCR and trajectory recognition.
Maps canonical labels to display names, tasks, and command intents.
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
        "display_name": "Move Up",
        "task": "MOVE_UP",
        "command_intent": "run_move_up",
        "description": "Move the robot up.",
        "description_zh": "控制机械臂向上移动。",
        "safety_critical": False,
    },
    "A": {
        "label": "A",
        "display_name": "Pick & Place",
        "task": "PICK_PLACE",
        "command_intent": "run_pick_place",
        "description": "Execute the pick and place sequence.",
        "description_zh": "执行抓取与放置任务。",
        "safety_critical": True,
    },
    "C": {
        "label": "C",
        "display_name": "Compliant Control",
        "task": "COMPLIANT_CONTROL",
        "command_intent": "run_compliant_control",
        "description": "Switch to compliant control mode.",
        "description_zh": "切换到柔顺控制模式。",
        "safety_critical": True,
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
        "display_name": "Avoid Obstacle",
        "task": "AVOID_OBSTACLE",
        "command_intent": "run_avoid_obstacle",
        "description": "Clear or avoid obstacle.",
        "description_zh": "清除或避开障碍物。",
        "safety_critical": True,
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

def normalize_symbol_label(raw_text, allowed_labels=None):
    """
    Normalize raw OCR text to a canonical symbol label.
    """
    text = str(raw_text).strip().upper()
    if not text:
        return ""

    if allowed_labels is None:
        allowed_labels = SYMBOL_SEMANTICS.keys()

    if text in allowed_labels:
        return text

    confusions = {
        "I": "1",
        "L": "1",
        "|": "1",
        "Z": "2",
        "O": "0",
    }

    if text in confusions and confusions[text] in allowed_labels:
        return confusions[text]

    if len(text) <= 3:
        for char in text:
            if char in allowed_labels:
                return char
            if char in confusions and confusions[char] in allowed_labels:
                return confusions[char]

    return ""
