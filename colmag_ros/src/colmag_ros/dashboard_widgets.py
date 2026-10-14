"""Canvas-drawn dashboard widgets: progress stepper, notice card, symbol tile, confidence meter.

Each function clears and redraws one Tk canvas. They hold no state and publish
nothing.
"""

from colmag_ros import dashboard_theme as theme


def rounded_rect(canvas, x1, y1, x2, y2, radius, **options):
    """A rounded rectangle as a smoothed polygon."""
    r = max(0.0, min(float(radius), (x2 - x1) / 2.0, (y2 - y1) / 2.0))
    points = [
        x1 + r, y1, x2 - r, y1, x2, y1, x2, y1 + r,
        x2, y2 - r, x2, y2, x2 - r, y2, x1 + r, y2,
        x1, y2, x1, y2 - r, x1, y1 + r, x1, y1,
    ]
    return canvas.create_polygon(*points, smooth=True, **options)


def draw_control_button(canvas, cx, cy, radius, text, style, progress=0.0):
    """Round dwell control: big letter, small caption, optional dwell ring.

    ``text`` is ``"<letter>\\n<caption>"``; ``style`` comes from
    :func:`colmag_ros.dashboard_geometry.preview_button_style`.
    """
    canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius,
                       fill=style["fill"], outline=style["outline"], width=style["width"])
    letter, _newline, caption = str(text).partition("\n")
    if caption:
        canvas.create_text(cx, cy - radius * 0.2, text=letter, fill=style["text"],
                           font=theme.font(max(9, int(radius * 0.6)), bold=True))
        canvas.create_text(cx, cy + radius * 0.42, text=caption, fill=style["text"],
                           font=theme.font(max(7, int(radius * 0.27)), bold=True))
    else:
        canvas.create_text(cx, cy, text=letter, fill=style["text"],
                           font=theme.font(max(8, int(radius * 0.4)), bold=True))
    if progress > 0.0:
        canvas.create_arc(cx - radius + 3, cy - radius + 3, cx + radius - 3, cy + radius - 3,
                          start=90, extent=-360 * progress, style="arc",
                          outline=style["ring"], width=4)


def draw_workflow_stepper(canvas, width, steps, current_index):
    """Numbered steps joined by a line; finished steps carry a check mark."""
    canvas.delete("all")
    count = len(steps)
    if count == 0 or width <= 0:
        return
    radius = 11
    margin = 36
    cy = radius + 6
    if count == 1:
        xs = [width / 2.0]
    else:
        span = max(1.0, float(width) - 2 * margin)
        xs = [margin + span * index / (count - 1) for index in range(count)]
    for index in range(count - 1):
        color = theme.ACCENT if index < current_index else theme.BORDER_STRONG
        canvas.create_line(xs[index] + radius + 4, cy, xs[index + 1] - radius - 4, cy,
                           fill=color, width=2)
    accent, accent_soft = theme.tone_colors("accent")
    for index, (_stage, label) in enumerate(steps):
        x = xs[index]
        if index < current_index:
            fill, outline, mark, mark_color = accent, accent, "✓", theme.SURFACE
            label_color, bold = theme.TEXT_SECONDARY, False
        elif index == current_index:
            canvas.create_oval(x - radius - 5, cy - radius - 5, x + radius + 5, cy + radius + 5,
                               fill=accent_soft, outline="")
            fill, outline, mark, mark_color = accent, accent, str(index + 1), theme.SURFACE
            label_color, bold = theme.TEXT, True
        else:
            fill, outline, mark, mark_color = (
                theme.SURFACE, theme.BORDER_STRONG, str(index + 1), theme.TEXT_MUTED)
            label_color, bold = theme.TEXT_MUTED, False
        canvas.create_oval(x - radius, cy - radius, x + radius, cy + radius,
                           fill=fill, outline=outline, width=2)
        canvas.create_text(x, cy, text=mark, fill=mark_color, font=theme.font(9, bold=True))
        canvas.create_text(x, cy + radius + 9, text=label, fill=label_color,
                           font=theme.font(9, bold=bold), anchor="n", justify="center")


def draw_notice_card(canvas, cx, cy, width, height, title, body, tone):
    """Centered rounded card: tone-colored title over secondary body text."""
    strong, _soft = theme.tone_colors(tone)
    x1, y1 = cx - width / 2.0, cy - height / 2.0
    rounded_rect(canvas, x1, y1, x1 + width, y1 + height, 12,
                 fill=theme.SURFACE, outline=strong, width=2)
    canvas.create_text(cx, y1 + 28, text=title, fill=strong, font=theme.font(15, bold=True))
    canvas.create_text(cx, y1 + 50, text=body, fill=theme.TEXT_SECONDARY, font=theme.font(10),
                       anchor="n", justify="center", width=width - 32)


# Keycap grid cells (column, row): a WASD cluster, a Q/E column and an O/C
# gripper column.
PAD_CELLS = {"W": (1, 0), "A": (0, 1), "S": (1, 1), "D": (2, 1), "Q": (3.5, 0), "E": (3.5, 1),
             "O": (5, 0), "C": (5, 1)}


def draw_teleop_pad(canvas, width, height, keys, active, caption):
    """Keycaps for the teleop keys; keys whose value is in ``active`` light up.

    ``keys`` is ``[(letter, direction or gripper command, label)]``; letters
    without a cell are skipped.
    """
    canvas.delete("all")
    gap = 10
    # Shrink the keycaps so all six columns fit inside the canvas.
    cap = max(36, min(64, int((width - 16 - 5 * gap) / 6)))
    cluster_w = 6 * cap + 5 * gap
    block_h = 2 * cap + gap + 34
    x0 = (width - cluster_w) / 2.0
    top = max(8.0, (height - block_h) / 2.0)
    canvas.create_text(width / 2.0, top + 2 * cap + gap + 18, text=caption, anchor="n",
                       fill=theme.TEXT_MUTED, font=theme.font(10))
    for letter, vector, axis_label in keys:
        cell = PAD_CELLS.get(letter)
        if cell is None:
            continue
        x = x0 + cell[0] * (cap + gap)
        y = top + cell[1] * (cap + gap)
        lit = vector in active
        rounded_rect(canvas, x, y + 3, x + cap, y + cap + 3, 10, fill=theme.BORDER, outline="")
        rounded_rect(canvas, x, y, x + cap, y + cap, 10,
                     fill=theme.ACCENT if lit else theme.SURFACE,
                     outline=theme.ACCENT if lit else theme.BORDER_STRONG, width=2)
        canvas.create_text(x + cap / 2.0, y + cap * 0.40, text=letter,
                           fill=theme.SURFACE if lit else theme.TEXT,
                           font=theme.font(18, bold=True))
        canvas.create_text(x + cap / 2.0, y + cap * 0.76, text=axis_label,
                           fill=theme.STROKE_HALO if lit else theme.TEXT_MUTED,
                           font=theme.font(9, bold=True))


def draw_symbol_tile(canvas, size, label, tone):
    """Large rounded tile showing the recognized symbol (or a placeholder)."""
    canvas.delete("all")
    strong, soft = theme.tone_colors(tone)
    if tone == "neutral":
        strong, soft = theme.TEXT_MUTED, theme.SURFACE_MUTED
    rounded_rect(canvas, 2, 2, size - 2, size - 2, 14,
                 fill=soft, outline=strong, width=2)
    canvas.create_text(size / 2.0, size / 2.0, text=label, fill=strong,
                       font=theme.font(max(12, int(size * 0.42)), bold=True))


def draw_confidence_meter(canvas, width, fraction, tone):
    """Horizontal bar on a track; ``fraction`` None draws nothing."""
    canvas.delete("all")
    if width <= 4 or fraction is None:
        return
    height = 10
    rounded_rect(canvas, 1, 1, width - 1, height - 1, 4,
                 fill=theme.BORDER, outline="")
    fraction = max(0.0, min(1.0, float(fraction)))
    fill_width = (width - 2) * fraction
    if fill_width >= 2:
        rounded_rect(canvas, 1, 1, 1 + fill_width, height - 1, 4,
                     fill=theme.tone_colors(tone)[0], outline="")
