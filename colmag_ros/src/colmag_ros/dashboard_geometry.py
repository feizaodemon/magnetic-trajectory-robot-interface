"""Package-level geometry and hit-test helpers for the trajectory dashboard."""


def clamp(value, low, high):
    return max(low, min(high, value))


def map_trajectory_point(
    x,
    y,
    bounds,
    width,
    height,
    padding,
    flip_x=False,
    flip_y=False,
    swap_xy=False,
    clamp=True,
):
    """Map a real-board (x, y) sample to canvas pixels for trajectory preview."""
    if swap_xy:
        x, y = y, x
    x_min, x_max, y_min, y_max = bounds
    x_range = x_max - x_min
    y_range = y_max - y_min
    if x_range <= 0 or y_range <= 0:
        return width / 2.0, height / 2.0
    draw_w = max(1.0, width - 2.0 * padding)
    draw_h = max(1.0, height - 2.0 * padding)
    scale = min(draw_w / x_range, draw_h / y_range)
    x_mid = (x_min + x_max) / 2.0
    y_mid = (y_min + y_max) / 2.0
    cx = width / 2.0 + (x - x_mid) * scale
    cy = height / 2.0 - (y - y_mid) * scale
    if flip_x:
        cx = width - cx
    if flip_y:
        cy = height - cy
    if clamp:
        cx = max(padding, min(width - padding, cx))
        cy = max(padding, min(height - padding, cy))
    return cx, cy


def is_point_inside_rect(x, y, rect):
    return (
        rect["x1"] <= x <= rect["x2"]
        and rect["y1"] <= y <= rect["y2"]
    )


def get_virtual_button_under_cursor(canvas_x, canvas_y, button_rects):
    for rect in button_rects:
        if is_point_inside_rect(canvas_x, canvas_y, rect):
            return rect["id"]
    return ""


def resolve_preview_button_hit(x, y, hit_zones):
    """Return the preview button id under a stage-local pointer position."""
    for zone in hit_zones or []:
        try:
            dx = float(x) - float(zone["cx"])
            dy = float(y) - float(zone["cy"])
            radius = float(zone["radius"])
        except (KeyError, TypeError, ValueError):
            continue
        if dx * dx + dy * dy <= radius * radius:
            return str(zone.get("id", "")).upper()
    return ""


def derive_orientation_clutch_mode(
    interaction_engaged,
    inside_drawing_zone=False,
    hovered_button="",
):
    """Derive the compact operator mode without owning interaction state."""
    if not interaction_engaged:
        return "NAVIGATE"
    if inside_drawing_zone:
        return "DRAW"
    button_id = str(hovered_button or "").strip().upper()
    if button_id in ("B", "C", "A", "X"):
        return "HOVER — %s" % button_id
    return "IDLE"


def normalize_board_control_layout(
    value,
    external_layout="external_toolbar",
    canvas_reachable_layout="canvas_reachable",
):
    layout = str(value or external_layout).strip().lower()
    if layout in ("canvas", "canvas_reachable", "inline", "inline_controls"):
        return canvas_reachable_layout
    return external_layout


def compute_dashboard_panel_widths(
    total_width,
    gap=15,
    min_left_width=560,
    min_right_width=380,
    right_ratio=0.33,
):
    """Split dashboard content while keeping the information panel readable."""
    available = max(0.0, float(total_width) - float(gap))
    if available == 0.0:
        return 0, 0
    right_width = max(float(min_right_width), available * float(right_ratio))
    max_right_width = max(0.0, available - float(min_left_width))
    if max_right_width >= float(min_right_width):
        right_width = min(right_width, max_right_width)
    else:
        right_width = min(right_width, available)
    right_width = int(right_width)
    return int(available) - right_width, right_width


def dashboard_minimum_window_size():
    """Minimum size proven by the normal operator-layout acceptance."""
    return 1060, 760


def compute_ocr_stage_geometry(
    available_width, available_height, controls_inside_canvas=False,
):
    """Return the existing OCR-stage layout values without mutating Tk widgets."""
    available_width = float(available_width)
    available_height = float(available_height)
    if available_width < 100 or available_height < 100:
        return None

    stage_width = int(max(300, min(1600, available_width - 24)))
    stage_height = int(max(320, min(1200, available_height - 28)))
    stage_x = int(max(10, (available_width - stage_width) / 2))
    stage_y = int(max(10, (available_height - stage_height) / 2))
    header_height = int(max(60, min(80, stage_height * 0.12)))
    footer_height = 26
    padding = int(max(6, min(12, min(stage_width, stage_height) * 0.02)))
    button_size = int(max(48, min(64, min(stage_width * 0.1, stage_height * 0.12))))
    gap = int(max(4, min(10, min(stage_width, stage_height) * 0.015)))

    control_reserve = 0 if controls_inside_canvas else 2 * button_size + 2 * gap
    max_draw_width = max(
        120, min(1400, stage_width - 2 * padding - control_reserve))
    max_draw_height = max(
        100, min(1000, stage_height - header_height - footer_height - control_reserve))
    draw_size = int(min(max_draw_width, max_draw_height))
    draw_width = draw_size
    draw_height = draw_size

    center_x = stage_width / 2.0
    cluster_height = draw_height if controls_inside_canvas else (
        button_size + gap + draw_height + gap + button_size)
    control_height = max(120, stage_height - header_height - footer_height)
    cluster_y = header_height + max(0, (control_height - cluster_height) / 2.0)
    draw_x = center_x - draw_width / 2.0
    draw_y = cluster_y if controls_inside_canvas else cluster_y + button_size + gap
    bottom_gap = stage_height - (cluster_y + cluster_height)

    button_rects = {
        "X": (center_x - button_size / 2, draw_y - gap - button_size),
        "A": (draw_x - gap - button_size, draw_y + draw_height / 2 - button_size / 2),
        "C": (draw_x + draw_width + gap, draw_y + draw_height / 2 - button_size / 2),
        "B": (center_x - button_size / 2, draw_y + draw_height + gap),
        "CLEAR": (draw_x + draw_width + gap, draw_y + draw_height + gap),
    }
    return {
        "stage": (stage_x, stage_y, stage_width, stage_height),
        "draw": (draw_x, draw_y, draw_width, draw_height),
        "header_height": header_height,
        "footer_height": footer_height,
        "padding": padding,
        "button_size": button_size,
        "gap": gap,
        "hover": (padding, stage_height - 22 if bottom_gap >= 24 else stage_height - 20),
        "button_rects": button_rects,
    }


def build_canvas_reachable_preview_hit_zones(width, height, padding, button_size, reach_rect=None):
    """Build X/A/B/C controls inside the trajectory canvas coordinate domain."""
    width = float(width)
    height = float(height)
    padding = max(0.0, float(padding))
    radius = max(18.0, min(float(button_size) * 0.43, min(width, height) / 10.0))
    if reach_rect is None:
        x1, y1, x2, y2 = padding, padding, width - padding, height - padding
    else:
        x1, y1, x2, y2 = [float(v) for v in reach_rect]
        x1 = max(padding, min(width - padding, x1))
        x2 = max(padding, min(width - padding, x2))
        y1 = max(padding, min(height - padding, y1))
        y2 = max(padding, min(height - padding, y2))
        if x2 < x1:
            x1, x2 = x2, x1
        if y2 < y1:
            y1, y2 = y2, y1
    safe_margin = max(8.0, padding * 0.5)
    edge_inset = radius + safe_margin
    left = min(x2 - edge_inset, x1 + edge_inset)
    right = max(x1 + edge_inset, x2 - edge_inset)
    top = min(y2 - edge_inset, y1 + edge_inset)
    bottom = max(y1 + edge_inset, y2 - edge_inset)
    return [
        {"id": "X", "label": "X\nClear", "cx": left, "cy": top, "radius": radius},
        {"id": "C", "label": "C\nConfirm", "cx": right, "cy": top, "radius": radius},
        {"id": "A", "label": "A\nReject", "cx": left, "cy": bottom, "radius": radius},
        {"id": "B", "label": "B\nRecognize", "cx": right, "cy": bottom, "radius": radius},
    ]


def preview_button_style(button_id, enabled, active=False):
    """Return one consistent visual contract for visible preview controls."""
    button_id = str(button_id or "").upper()
    if not enabled:
        fill, outline = {
            "X": ("#F1F5F9", "#94A3B8"),
            "A": ("#FEF2F2", "#FCA5A5"),
            "B": ("#EFF6FF", "#93C5FD"),
            "C": ("#F0FDF4", "#86EFAC"),
        }.get(button_id, ("#F8FAFC", "#CBD5E1"))
        return {
            "fill": fill,
            "outline": outline,
            "text": "#94A3B8",
            "width": 2,
            "ring": "#334155" if button_id == "X" else "#DC2626",
        }
    fill, outline = {
        "X": ("#E2E8F0", "#475569"),
        "A": ("#FEE2E2", "#DC2626"),
        "B": ("#DBEAFE", "#2563EB"),
        "C": ("#DCFCE7", "#16A34A"),
    }.get(button_id, ("#F8FAFC", "#94A3B8"))
    if active and button_id == "X":
        fill, outline = "#CBD5E1", "#334155"
    elif active:
        fill, outline = "#FEF3C7", "#D97706"
    return {
        "fill": fill,
        "outline": outline,
        "text": "#1F2937",
        "width": 3 if active else 2,
        "ring": "#334155" if button_id == "X" else "#DC2626",
    }


def drawing_zone_style():
    """Neutral identity kept distinct from the C Confirm semantic green."""
    return {"outline": "#64748B", "text": "#475569"}


def build_canvas_reachable_drawing_zone(width, height, padding, reach_rect=None):
    """Central canvas region whose points are eligible for DTW capture."""
    width = float(width)
    height = float(height)
    padding = max(0.0, float(padding))
    if reach_rect is None:
        x1, y1, x2, y2 = padding, padding, width - padding, height - padding
    else:
        x1, y1, x2, y2 = [float(v) for v in reach_rect]
    if x2 < x1:
        x1, x2 = x2, x1
    if y2 < y1:
        y1, y2 = y2, y1
    reach_w = max(1.0, x2 - x1)
    reach_h = max(1.0, y2 - y1)
    square_side = min(reach_w, reach_h)
    square_x1 = (x1 + x2 - square_side) / 2.0
    square_y1 = (y1 + y2 - square_side) / 2.0
    # At the accepted minimum window, 66 px clears the existing 64 px control
    # circle plus its safe edge margin. Larger canvases retain bounded scaling.
    shared_margin = max(66.0, square_side * 0.15)
    margin_x = shared_margin
    margin_y = shared_margin
    if margin_x * 2.0 >= square_side:
        margin_x = square_side * 0.20
    if margin_y * 2.0 >= square_side:
        margin_y = square_side * 0.20
    return {
        "x1": square_x1 + margin_x,
        "y1": square_y1 + margin_y,
        "x2": square_x1 + square_side - margin_x,
        "y2": square_y1 + square_side - margin_y,
    }


def trajectory_point_in_drawing_zone(
    point,
    bounds,
    width,
    height,
    padding,
    drawing_zone,
    flip_x=False,
    flip_y=False,
    swap_xy=False,
):
    if point is None:
        return False
    cx, cy = map_trajectory_point(
        point[0],
        point[1],
        bounds,
        width,
        height,
        padding,
        flip_x=flip_x,
        flip_y=flip_y,
        swap_xy=swap_xy,
        clamp=False,
    )
    return is_point_inside_rect(cx, cy, drawing_zone)


def canvas_zones_to_stage_zones(canvas_zones, canvas_x, canvas_y):
    zones = []
    for zone in canvas_zones:
        cx = float(canvas_x) + float(zone["cx"])
        cy = float(canvas_y) + float(zone["cy"])
        radius = float(zone["radius"])
        zones.append({
            "id": str(zone["id"]).upper(),
            "label": zone.get("label", ""),
            "x1": cx - radius,
            "y1": cy - radius,
            "x2": cx + radius,
            "y2": cy + radius,
            "cx": cx,
            "cy": cy,
            "radius": radius,
        })
    return zones
