"""Canvas drawing, coordinate mapping, and canvas hit tests for the trajectory dashboard."""

import math

from colmag_ros.dashboard_geometry import (
    BOARD_CONTROL_LAYOUT_CANVAS_REACHABLE,
    build_canvas_reachable_drawing_zone,
    drawing_zone_style,
    map_canvas_to_world,
    map_trajectory_point,
    map_world_to_canvas,
    preview_button_style,
    resolve_preview_button_hit,
    trajectory_point_in_drawing_zone,
)
from colmag_ros.dashboard_trajectory_processing import (
    compute_trajectory_bounds,
    split_trail_segments,
)


class DashboardCanvasViewMixin:
    """Draws the trajectory canvas and maps between world, canvas, and stage coordinates."""

    def _use_canvas_reachable_preview_controls(self):
        return (
            self.config.is_trajectory_mode
            and self.config.candidate_preview_enabled
            and self.config.hover_buttons_enabled
            and self.config.board_control_layout == BOARD_CONTROL_LAYOUT_CANVAS_REACHABLE
        )

    def _reachable_trajectory_canvas_rect(self):
        bounds = self._active_bounds
        return self._trajectory_canvas_rect_for_bounds(bounds)

    def _trajectory_canvas_rect_for_bounds(self, bounds):
        x_min, x_max, y_min, y_max = bounds
        corners = [
            map_trajectory_point(
                x, y, bounds, self.canvas_width, self.canvas_height, self.config.padding,
                flip_x=self.config.traj_flip_x, flip_y=self.config.traj_flip_y,
                swap_xy=self.config.traj_swap_xy,
            )
            for x in (x_min, x_max)
            for y in (y_min, y_max)
        ]
        xs = [p[0] for p in corners]
        ys = [p[1] for p in corners]
        return (min(xs), min(ys), max(xs), max(ys))

    def _drawing_zone_canvas_rect(self):
        return build_canvas_reachable_drawing_zone(
            self.canvas_width,
            self.canvas_height,
            self.config.padding,
            reach_rect=self._trajectory_canvas_rect_for_bounds(self.config.bounds),
        )

    def _trajectory_point_in_drawing_zone(self, pt):
        return trajectory_point_in_drawing_zone(
            pt,
            self.config.bounds,
            self.canvas_width,
            self.canvas_height,
            self.config.padding,
            self._drawing_zone_canvas_rect(),
            flip_x=self.config.traj_flip_x,
            flip_y=self.config.traj_flip_y,
            swap_xy=self.config.traj_swap_xy,
        )

    def _trajectory_point_in_control_zone(self, pt):
        if not self._use_canvas_reachable_preview_controls():
            return False
        if pt is None:
            return False
        zones = self.preview_canvas_button_zones or []
        if not zones:
            return False
        cx, cy = self._canvas_xy(pt[0], pt[1])
        return bool(resolve_preview_button_hit(cx, cy, zones))

    def _redraw_canvas(self):
        # Clear first, so state changes during this frame schedule another redraw.
        self._redraw_pending = False
        self.canvas.delete("all")

        # Refresh the active scaling window once per frame (auto / fixed_symmetric).
        if self.config.is_trajectory_mode:
            self._active_bounds = self._active_trajectory_bounds()

        if self.config.demo_input_mode == "ocr_canvas" and self._ocr_draw_area:
            x0, y0, x1, y1 = self._ocr_draw_area
            zone_style = drawing_zone_style()
            self.canvas.create_rectangle(
                x0, y0, x1, y1, fill="#FFFFFF",
                outline=zone_style["outline"], width=2, dash=(5, 4))
            self.canvas.create_text(
                self.canvas_width / 2, y0 + 18, text="Drawing zone",
                fill=zone_style["text"], font=("TkDefaultFont", 10, "bold"),
            )
        elif self.config.is_trajectory_mode:
            # Draw x/y axes through the mapped world origin so orientation and
            # scale are visually checkable.
            ox, oy = self._canvas_xy(0.0, 0.0)
            self.canvas.create_line(self.config.padding, oy, self.canvas_width-self.config.padding, oy, fill="#D5DBDB", dash=(4, 4))
            self.canvas.create_line(ox, self.config.padding, ox, self.canvas_height-self.config.padding, fill="#D5DBDB", dash=(4, 4))
        else:
            # Grid lines
            self.canvas.create_line(self.config.padding, self.canvas_height/2, self.canvas_width-self.config.padding, self.canvas_height/2, fill="#ECF0F1", dash=(4, 4))
            self.canvas.create_line(self.canvas_width/2, self.config.padding, self.canvas_width/2, self.canvas_height-self.config.padding, fill="#ECF0F1", dash=(4, 4))

        if self.config.is_trajectory_mode and self._use_canvas_reachable_preview_controls():
            zone = self.preview_canvas_drawing_zone or self._drawing_zone_canvas_rect()
            zone_style = drawing_zone_style()
            self.canvas.create_rectangle(
                zone["x1"], zone["y1"], zone["x2"], zone["y2"],
                outline=zone_style["outline"], width=2, dash=(6, 4), fill="")
            self.canvas.create_text(
                zone["x1"] + 8, zone["y1"] + 12,
                text="Drawing zone", anchor=self.tk.W,
                fill=zone_style["text"],
                font=("TkDefaultFont", 9, "bold"))

        if self.config.demo_input_mode != "ocr_canvas":
            for segment in split_trail_segments(self.drawing_zone_trail):
                coords = []
                for x, y in segment:
                    cx, cy = self._canvas_xy(x, y)
                    coords.extend([cx, cy])
                self.canvas.create_line(*coords, fill="#16A34A", width=4)

        # Draw frozen sample sent to recognition. It intentionally differs from
        # the live trail color so the operator can inspect what B recognized.
        if len(self.captured_path) > 1:
            pts = []
            for x, y in self.captured_path:
                if self.config.is_trajectory_mode:
                    cx, cy = self._canvas_xy(x, y)
                else:
                    cx, cy = map_world_to_canvas(
                        x, y, self.config.bounds, self.canvas_width, self.canvas_height, self.config.padding)
                pts.extend([cx, cy])
            self.canvas.create_line(*pts, fill="#F59E0B", width=6, smooth=True)
            self.canvas.create_line(*pts, fill="#FEF3C7", width=2, smooth=True)
            if self.config.dashboard_debug_sample_panel:
                label = "Captured sample"
                if self.captured_sample_point_count:
                    label += " (%d pts)" % self.captured_sample_point_count
                self.canvas.create_text(
                    self.config.padding + 8, self.config.padding + 12,
                    text=label, anchor=self.tk.W,
                    fill="#92400E", font=("TkDefaultFont", 10, "bold"))

        # Draw live trail. ``None`` entries are stroke breaks (gated / non-writing
        # points) that split the polyline into separate segments. Single-point
        # segments are dropped so create_line is never called with < 2 points.
        if self.config.demo_input_mode != "ocr_canvas":
            for segment in split_trail_segments(self.trail):
                coords = []
                for x, y in segment:
                    cx, cy = self._canvas_xy(x, y)
                    coords.extend([cx, cy])
                self.canvas.create_line(*coords, fill="#2563EB", width=3)

        # Draw manual strokes for OCR Canvas
        for stroke in self.all_strokes:
            if len(stroke) > 1:
                pts = []
                for x, y in stroke:
                    cx, cy = map_world_to_canvas(x, y, self.config.bounds, self.canvas_width, self.canvas_height, self.config.padding)
                    pts.extend([cx, cy])
                self.canvas.create_line(*pts, fill="#8E44AD", width=3, smooth=True)
        if len(self.drawing_stroke) > 1:
            pts = []
            for x, y in self.drawing_stroke:
                cx, cy = map_world_to_canvas(x, y, self.config.bounds, self.canvas_width, self.canvas_height, self.config.padding)
                pts.extend([cx, cy])
            self.canvas.create_line(*pts, fill="#8E44AD", width=3, smooth=True)

        # Draw current point (grey when gated as non-writing or invalid).
        if self.config.is_trajectory_mode:
            if self.latest_point is not None:
                cx, cy = self._canvas_xy(self.latest_point[0], self.latest_point[1])
                r = 8
                drawing = self.latest_valid and self.latest_writing
                fill, outline = ("#E74C3C", "#C0392B") if drawing else ("#AEB6BF", "#7F8C8D")
                self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r, fill=fill, outline=outline)
        elif self.trail and self.config.demo_input_mode != "ocr_canvas":
            x, y = self.trail[-1]
            cx, cy = self._canvas_xy(x, y)
            r = 8
            self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r, fill="#E74C3C", outline="#C0392B")

        self._draw_canvas_reachable_preview_buttons()

    def _draw_canvas_reachable_preview_buttons(self):
        if not self._use_canvas_reachable_preview_controls():
            return
        current_button = self.ocr_hover_button
        progress = self.ocr_dwell_progress
        for zone in self.preview_canvas_button_zones or []:
            button_id = str(zone.get("id", "")).upper()
            enabled = self._preview_button_enabled(button_id)
            active = button_id == current_button
            radius = float(zone["radius"])
            cx = float(zone["cx"])
            cy = float(zone["cy"])
            style = preview_button_style(button_id, enabled, active=active)
            self.canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius,
                                    fill=style["fill"], outline=style["outline"], width=style["width"])
            self.canvas.create_text(cx, cy, text=zone.get("label", button_id),
                                    font=("TkDefaultFont", max(8, int(radius * 0.34)), "bold"),
                                    justify=self.tk.CENTER, fill=style["text"])
            if active and enabled and progress > 0.0 and self.config.hover_progress_enabled:
                self.canvas.create_arc(cx - radius + 3, cy - radius + 3,
                                       cx + radius - 3, cy + radius - 3,
                                       start=90, extent=-360 * progress,
                                       style=self.tk.ARC, outline=style["ring"], width=4)

    def _active_trajectory_bounds(self):
        if self.config.is_trajectory_mode and self.config.traj_scale_mode in ("auto", "fixed_symmetric"):
            pts = [p for p in self.trail if p is not None]  # drop stroke breaks
            return compute_trajectory_bounds(
                pts, self.config.bounds, scale_mode=self.config.traj_scale_mode,
                auto_center=self.config.traj_auto_center,
                padding_ratio=self.config.traj_auto_padding_ratio,
                min_span=self.config.traj_auto_min_span,
                window_points=self.config.traj_trace_window_points,
                projection_extent=self.config.traj_projection_extent,
            )
        return self.config.bounds

    def _canvas_xy(self, x, y):
        # ``self._active_bounds`` is refreshed once per redraw so auto-fit is not
        # recomputed for every trail point.
        if self.config.is_trajectory_mode:
            return map_trajectory_point(
                x, y, self._active_bounds,
                self.canvas_width, self.canvas_height, self.config.padding,
                flip_x=self.config.traj_flip_x, flip_y=self.config.traj_flip_y,
                swap_xy=self.config.traj_swap_xy,
            )
        return map_world_to_canvas(x, y, self.config.bounds, self.canvas_width, self.canvas_height, self.config.padding)

    def _canvas_xy_unclamped(self, x, y):
        """Like :meth:`_canvas_xy` but without clamping to the canvas rectangle.

        The buttons surround the trajectory canvas, so a clamped point (which is
        what the red dot is drawn at) can never leave the canvas to reach a
        button. Hit-testing the live board cursor uses the unclamped mapped
        position so pushing the magnet past the drawing region moves the logical
        cursor into the surrounding button zones.
        """
        if self.config.is_trajectory_mode:
            return map_trajectory_point(
                x, y, self._active_bounds,
                self.canvas_width, self.canvas_height, self.config.padding,
                flip_x=self.config.traj_flip_x, flip_y=self.config.traj_flip_y,
                swap_xy=self.config.traj_swap_xy, clamp=False,
            )
        return map_world_to_canvas(x, y, self.config.bounds, self.canvas_width, self.canvas_height, self.config.padding)

    def _canvas_xy_to_stage_xy(self, canvas_x, canvas_y):
        """Convert a trajectory-canvas pixel to the stage-local coordinate system
        used by the preview button hit zones. Same frame the mouse pointer is
        mapped into by :meth:`_preview_button_at_event`, so the red-point check
        and the visible circles agree."""
        canvas_root_x = float(self.canvas.winfo_rootx())
        canvas_root_y = float(self.canvas.winfo_rooty())
        stage_root_x = float(self.ocr_stage_card.winfo_rootx())
        stage_root_y = float(self.ocr_stage_card.winfo_rooty())
        stage_x = canvas_root_x + float(canvas_x) - stage_root_x
        stage_y = canvas_root_y + float(canvas_y) - stage_root_y
        return stage_x, stage_y

    def _resolve_preview_button_at_stage_xy(self, stage_x, stage_y):
        """Resolve a stage-local (x, y) to a preview button id (or "")."""
        return resolve_preview_button_hit(stage_x, stage_y, self.preview_button_hit_zones)

    def _board_cursor_preview_button(self):
        """Preview button currently under the live board's red current point.

        Returns the button id (X/A/B/C) or "" if the cursor is not over any
        button. Uses the unclamped mapped position so an extreme board position
        reaches the buttons around the canvas, then converts to the same
        stage-local frame as the hit zones."""
        if not self._preview_pointer_tracking_enabled():
            return ""
        if self.latest_point is None:
            return ""
        if not self.latest_input_active:
            return ""
        if not self.preview_button_hit_zones:
            return ""
        try:
            if self._use_canvas_reachable_preview_controls():
                cx, cy = self._canvas_xy(self.latest_point[0], self.latest_point[1])
            else:
                cx, cy = self._canvas_xy_unclamped(self.latest_point[0], self.latest_point[1])
            stage_x, stage_y = self._canvas_xy_to_stage_xy(cx, cy)
        except (AttributeError, TypeError, ValueError, self.tk.TclError):
            return ""
        return self._resolve_preview_button_at_stage_xy(stage_x, stage_y)

    def _is_in_draw_area(self, x, y):
        if self.config.demo_input_mode != "ocr_canvas" or not self._ocr_draw_area:
            return True
        x0, y0, x1, y1 = self._ocr_draw_area
        return x0 <= x <= x1 and y0 <= y <= y1

    def _on_canvas_press(self, event):
        if self.ocr_panel_state != "DRAWING":
            return
        if not self._is_in_draw_area(event.x, event.y):
            return
        self._cancel_ocr_auto_recognize()
        self.is_drawing = True
        world_x, world_y = map_canvas_to_world(event.x, event.y, self.config.bounds, self.canvas_width, self.canvas_height, self.config.padding)
        self.drawing_stroke = [(world_x, world_y)]
        self.rospy.loginfo("ocr stroke started point_count=1")
        self._schedule_ui_refresh()

    def _on_canvas_motion(self, event):
        if not self.is_drawing:
            return
        if not self._is_in_draw_area(event.x, event.y):
            return
        world_x, world_y = map_canvas_to_world(event.x, event.y, self.config.bounds, self.canvas_width, self.canvas_height, self.config.padding)
        if self.drawing_stroke:
            last_x, last_y = self.drawing_stroke[-1]
            if math.hypot(world_x - last_x, world_y - last_y) > 0.002:
                self.drawing_stroke.append((world_x, world_y))
                if len(self.drawing_stroke) == 2 or len(self.drawing_stroke) % 10 == 0:
                    self.rospy.loginfo("ocr stroke point added point_count=%d", len(self.drawing_stroke))
                self._schedule_ui_refresh()

    def _on_canvas_release(self, event):
        if not self.is_drawing:
            return
        self.is_drawing = False
        if len(self.drawing_stroke) > 1:
            self.all_strokes.append(self.drawing_stroke)
            self.rospy.loginfo("ocr stroke released stroke_points=%d total_points=%d", len(self.drawing_stroke), len(self._collect_ocr_points()))
        self.drawing_stroke = []
        self._schedule_ocr_auto_recognize()
        self._schedule_ui_refresh()
