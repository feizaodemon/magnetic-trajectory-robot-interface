"""Pure trajectory display bounds and writing-gate decisions."""


def robust_min_max(values):
    values = sorted(values)
    count = len(values)
    if count == 0:
        return 0.0, 0.0
    if count < 20:
        return values[0], values[-1]
    low = int(count * 0.02)
    high = max(int(count * 0.98) - 1, low)
    return values[low], values[high]


def enforce_min_span(low, high, min_span):
    if high - low >= min_span:
        return low, high
    midpoint = (low + high) / 2.0
    return midpoint - min_span / 2.0, midpoint + min_span / 2.0


def normalize_scale_mode(value):
    """Return fixed, fixed_symmetric, or auto for the public aliases."""
    mode = str(value or "fixed").strip().lower()
    if mode == "fixed_symmetric":
        return "fixed_symmetric"
    if mode == "auto":
        return "auto"
    return "fixed"


def normalize_stroke_gate_mode(value):
    mode = str(value or "off").strip().lower()
    if mode in ("off", "z_range", "speed", "legacy_like"):
        return mode
    return "off"


def optional_float(value):
    if value is None:
        return None
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ("", "none", "nan"):
            return None
        try:
            return float(text)
        except ValueError:
            return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def stroke_gate_pass(
    mode,
    z,
    speed,
    z_min=None,
    z_max=None,
    min_speed=0.0,
    max_speed=0.0,
):
    """Return whether a sample extends the preview stroke."""
    mode = normalize_stroke_gate_mode(mode)
    if mode == "off":
        return True

    def speed_allowed():
        if speed is None:
            return True
        if speed < min_speed:
            return False
        if max_speed and max_speed > 0 and speed > max_speed:
            return False
        return True

    def z_allowed():
        if z_min is None or z_max is None or z is None:
            return True
        return z_min <= z <= z_max

    if mode == "z_range":
        return z_allowed()
    if mode == "speed":
        return speed_allowed()
    if mode == "legacy_like":
        return speed_allowed() and z_allowed()
    return True


def compute_trajectory_bounds(
    points,
    fixed_bounds,
    scale_mode="fixed",
    auto_center=False,
    padding_ratio=0.12,
    min_span=0.01,
    window_points=None,
    projection_extent=0.06,
):
    """Return fixed or robust auto-fit world bounds for trajectory display."""
    mode = normalize_scale_mode(scale_mode)
    if mode == "fixed_symmetric":
        extent = (
            projection_extent
            if projection_extent and projection_extent > 0
            else 0.06
        )
        return (-extent, extent, -extent, extent)
    if mode != "auto" or not points:
        return fixed_bounds

    points = list(points)
    if window_points and window_points > 0:
        points = points[-int(window_points):]
    x_values = [float(point[0]) for point in points]
    y_values = [float(point[1]) for point in points]
    x_min, x_max = robust_min_max(x_values)
    y_min, y_max = robust_min_max(y_values)

    if auto_center:
        center_x = (x_min + x_max) / 2.0
        center_y = (y_min + y_max) / 2.0
        half = max(
            (x_max - x_min) / 2.0,
            (y_max - y_min) / 2.0,
            min_span / 2.0,
        )
        x_min, x_max = center_x - half, center_x + half
        y_min, y_max = center_y - half, center_y + half

    x_min, x_max = enforce_min_span(x_min, x_max, min_span)
    y_min, y_max = enforce_min_span(y_min, y_max, min_span)
    x_padding = (x_max - x_min) * padding_ratio
    y_padding = (y_max - y_min) * padding_ratio
    return (
        x_min - x_padding,
        x_max + x_padding,
        y_min - y_padding,
        y_max + y_padding,
    )


def split_trail_segments(trail):
    """Split points around None stroke breaks and drop single-point runs."""
    segments = []
    current = []
    for point in trail:
        if point is None:
            if len(current) >= 2:
                segments.append(current)
            current = []
        else:
            current.append(point)
    if len(current) >= 2:
        segments.append(current)
    return segments
