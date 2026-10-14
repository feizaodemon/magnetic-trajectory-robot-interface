"""Tk window layout for the trajectory dashboard."""

from colmag_ros import dashboard_theme as theme
from colmag_ros.dashboard_candidate_display import (
    operator_workflow_steps,
    workflow_step_index,
)
from colmag_ros.dashboard_geometry import (
    build_canvas_reachable_preview_hit_zones,
    canvas_zones_to_stage_zones,
    compute_dashboard_panel_widths,
    compute_ocr_stage_geometry,
)
from colmag_ros.dashboard_texts import (
    HEADER_TITLE,
    TRAJECTORY_MODE,
    default_header_subtitle,
    default_main_panel_title,
)
from colmag_ros.dashboard_widgets import (
    draw_confidence_meter,
    draw_symbol_tile,
    draw_workflow_stepper,
)

RECOGNITION_TILE_SIZE = 84


class DashboardLayoutMixin:
    """Builds the dashboard window: header, canvas area, status cards, footer."""

    def _build_ui(self):
        tk = self.tk
        ttk = self.ttk

        style = ttk.Style(self.root)
        style.theme_use('clam')
        style.configure("CardTitle.TLabel", background=theme.SURFACE,
                        foreground=theme.TEXT_MUTED, font=theme.font(9, bold=True))
        style.configure("Header.TButton", background=theme.HEADER_CHIP,
                        foreground=theme.HEADER_TEXT, bordercolor=theme.HEADER_CHIP_ACTIVE,
                        lightcolor=theme.HEADER_CHIP, darkcolor=theme.HEADER_CHIP,
                        focuscolor=theme.HEADER_CHIP, padding=(12, 5), font=theme.font(10))
        style.map("Header.TButton", background=[("active", theme.HEADER_CHIP_ACTIVE)])

        main_frame = tk.Frame(self.root, bg=theme.APP_BG)
        main_frame.pack(fill=tk.BOTH, expand=True)

        self._build_header(main_frame)
        self._build_safety_footer(main_frame)

        content_frame = tk.Frame(main_frame, bg=theme.APP_BG)
        # The stage geometry keeps a 12-14 px margin inside the left frame; the
        # paddings below line the stage up with the header text and the cards.
        content_frame.pack(fill=tk.BOTH, expand=True, padx=(6, 0))
        self.dashboard_content_frame = content_frame

        left_frame = tk.Frame(content_frame, bg=theme.APP_BG)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.dashboard_left_frame = left_frame

        self._build_canvas_area(left_frame)

        right_frame = tk.Frame(content_frame, bg=theme.APP_BG, width=380)
        right_frame.pack_propagate(False)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(4, 18), pady=14)
        self.dashboard_right_frame = right_frame
        self._build_status_cards(right_frame)
        content_frame.bind("<Configure>", self._layout_dashboard_columns)
        right_frame.bind("<Configure>", self._layout_dashboard_status_wrap)

    def _layout_dashboard_columns(self, event=None):
        total_width = getattr(event, "width", self.dashboard_content_frame.winfo_width())
        _left_width, right_width = compute_dashboard_panel_widths(total_width)
        self.dashboard_right_frame.configure(width=right_width)

    def _layout_dashboard_status_wrap(self, event=None):
        right_width = getattr(event, "width", self.dashboard_right_frame.winfo_width())
        for label, base_wraplength, inset in self.dashboard_wrapped_labels:
            label.configure(wraplength=max(base_wraplength, int(right_width) - inset))

    def _build_header(self, parent):
        tk = self.tk
        ttk = self.ttk
        header = tk.Frame(parent, bg=theme.HEADER_BG)
        header.pack(fill=tk.X)

        left_frame = tk.Frame(header, bg=theme.HEADER_BG)
        left_frame.pack(side=tk.LEFT, padx=18, pady=12)

        if self.config.subtitle_text:
            sub = self.config.subtitle_text
        else:
            sub = default_header_subtitle(self.config.demo_input_mode)
        tk.Label(left_frame, text=HEADER_TITLE, bg=theme.HEADER_BG, fg=theme.HEADER_TEXT,
                 font=theme.font(15, bold=True)).pack(anchor=tk.W)
        tk.Label(left_frame, text=sub, bg=theme.HEADER_BG, fg=theme.HEADER_MUTED,
                 font=theme.font(10)).pack(anchor=tk.W, pady=(2, 0))

        right_frame = tk.Frame(header, bg=theme.HEADER_BG)
        right_frame.pack(side=tk.RIGHT, padx=18, pady=12)
        if self.config.control_mode_enabled:
            tk.Label(right_frame, textvariable=self.control_mode_var, bg=theme.HEADER_CHIP,
                     fg=theme.HEADER_TEXT, font=theme.font(10, bold=True),
                     padx=10, pady=5).pack(side=tk.LEFT, padx=(0, 8))
            self.control_mode_button = ttk.Button(
                right_frame,
                textvariable=self.control_mode_button_var,
                command=self._request_control_mode_switch,
                style="Header.TButton",
            )
            self.control_mode_button.pack(side=tk.LEFT)
        if self.config.safety_badge:
            tk.Label(right_frame, text=self.config.safety_badge,
                     bg=theme.tone_colors("warning")[0], fg=theme.SURFACE,
                     font=theme.font(10, bold=True), padx=8, pady=5).pack(side=tk.LEFT, padx=(8, 0))

    def _build_canvas_area(self, parent):
        tk = self.tk
        stage_host = tk.Frame(parent, bg=theme.APP_BG)
        stage_host.pack(fill=tk.BOTH, expand=True)
        self.ocr_stage_frame = stage_host
        stage_host.bind("<Configure>", self._layout_ocr_stage)

        stage = tk.Frame(stage_host, bg=theme.SURFACE, highlightthickness=1,
                         highlightbackground=theme.BORDER)
        self.ocr_stage_card = stage

        title_text = default_main_panel_title(self.config.demo_input_mode)
        self.ocr_title_label = tk.Label(stage, text=title_text, bg=theme.SURFACE, fg=theme.TEXT,
                                        font=theme.font(14, bold=True))
        self.ocr_hint_label = tk.Label(stage, textvariable=self.ocr_hint_var, bg=theme.SURFACE,
                                       fg=theme.TEXT_SECONDARY, font=theme.font(10),
                                       justify=tk.LEFT)

        self.canvas = tk.Canvas(stage, bg=theme.SURFACE, highlightthickness=0)

        if self.config.demo_input_mode == "ocr_canvas":
            buttons = [
                ("X", "X\nClear", lambda e, bid="X": self._handle_ocr_action(bid)),
                ("A", "A\nStop", lambda e, bid="A": self._handle_ocr_action(bid)),
                ("C", "C\nConfirm", lambda e, bid="C": self._handle_ocr_action(bid)),
                ("B", "B\nDraw", lambda e, bid="B": self._handle_ocr_action(bid)),
            ]
        elif self.config.is_trajectory_mode and self.config.candidate_preview_enabled and self.config.hover_buttons_enabled:
            buttons = [
                ("X", "X\nClear", lambda e, bid="X": self._handle_preview_button_action(bid)),
                ("A", "A\nReject", lambda e, bid="A": self._handle_preview_button_action(bid)),
                ("C", "C\nConfirm", lambda e, bid="C": self._handle_preview_button_action(bid)),
                ("B", "B\nRecognize", lambda e, bid="B": self._handle_preview_button_action(bid)),
            ]
        elif self.config.is_trajectory_mode:
            buttons = []
        else:
            buttons = [
                ("X", "A\nRecog", lambda e: self._on_recognize_button()),
                ("A", "X\nReject", lambda e: self._on_reject_button()),
                ("C", "B\nConfirm", lambda e: self._on_confirm_publish_button()),
                ("B", "C\nResamp", lambda e: self._on_resample_button()),
                ("CLEAR", "CLEAR", lambda e: self._on_clear_button()),
            ]

        for button_id, label_text, handler in buttons:
            widget = tk.Canvas(stage, bg=theme.SURFACE, highlightthickness=0, cursor="hand2")
            preview_button = (
                self.config.is_trajectory_mode and self.config.candidate_preview_enabled
                and self.config.hover_buttons_enabled
            )
            if self.config.demo_input_mode == "ocr_canvas" or (
                preview_button and self.config.hover_dwell_enabled
            ):
                widget.bind("<Enter>", lambda _event, bid=button_id: self._set_ocr_hover_button(bid))
                widget.bind("<Leave>", lambda _event, bid=button_id: self._clear_ocr_hover_button(bid))
                if preview_button:
                    widget.bind("<Motion>", self._handle_preview_pointer_motion)
            else:
                def on_enter(e, cv=widget): cv.config(cursor="hand2")
                def on_leave(e, cv=widget): cv.config(cursor="")
                widget.bind("<Enter>", on_enter)
                widget.bind("<Leave>", on_leave)

            if preview_button:
                widget.bind("<Button-1>", self._handle_preview_pointer_click)
                widget.tag_bind("all", "<Button-1>", self._handle_preview_pointer_click)
            elif self.config.hover_click_enabled or self.config.demo_input_mode != TRAJECTORY_MODE:
                widget.bind("<Button-1>", handler)
                # Support text clicks as well if they miss the circle slightly
                widget.tag_bind("all", "<Button-1>", handler)

            self.ocr_button_widgets[button_id] = {
                "canvas": widget,
                "base_text": label_text,
                "size": 82,
            }
        self._update_ocr_toolbar_visuals()

        self.canvas.bind("<ButtonPress-1>", self._on_canvas_press)
        self.canvas.bind("<B1-Motion>", self._on_canvas_motion)
        self.canvas.bind("<ButtonRelease-1>", self._on_canvas_release)
        self.root.bind("<Key>", self._on_key_press)
        if self.config.is_trajectory_mode and self.config.candidate_preview_enabled and self.config.hover_buttons_enabled:
            for widget in (stage, stage_host, self.canvas):
                widget.bind("<Motion>", self._handle_preview_pointer_motion, add="+")
                widget.bind("<Button-1>", self._handle_preview_pointer_click, add="+")
            self.root.bind("<Motion>", self._handle_preview_pointer_motion, add="+")
            self.root.bind("<Leave>", self._handle_preview_pointer_leave, add="+")
        if self.config.demo_input_mode == "ocr_canvas" or self._preview_dwell_active():
            self.root.after(50, self._process_ocr_toolbar_dwell)

    def _layout_ocr_stage(self, event=None):

        available_w = self.ocr_stage_frame.winfo_width()
        available_h = self.ocr_stage_frame.winfo_height()
        use_canvas_controls = self._use_canvas_reachable_preview_controls()
        geometry = compute_ocr_stage_geometry(
            available_w, available_h,
            controls_inside_canvas=use_canvas_controls,
        )
        if geometry is None:
            return

        stage_x, stage_y, stage_w, stage_h = geometry["stage"]
        draw_x, draw_y, draw_w, draw_h = geometry["draw"]
        pad = geometry["padding"]
        button_size = geometry["button_size"]
        self.ocr_stage_card.place(x=stage_x, y=stage_y, width=stage_w, height=stage_h)

        self.canvas_width = int(draw_w)
        self.canvas_height = int(draw_h)
        self.canvas.config(width=self.canvas_width, height=self.canvas_height)
        self.canvas.place(x=int(draw_x), y=int(draw_y), width=self.canvas_width, height=self.canvas_height)
        self._ocr_draw_area = (10, 24, self.canvas_width - 10, self.canvas_height - 10)

        self.rospy.loginfo_once("M48 geometry: window=%dx%d left_panel=%dx%d draw_canvas=%dx%d draw_w=%d draw_h=%d",
                                available_w, available_h, stage_w, stage_h,
                                self.canvas_width, self.canvas_height, draw_w, draw_h)

        text_x = pad + 8
        self.ocr_title_label.place(x=text_x, y=12)
        self.ocr_hint_label.configure(wraplength=max(180, stage_w - 2 * text_x))
        self.ocr_hint_label.place(x=text_x, y=38)
        hit_zones = []
        canvas_control_zones = []
        for button_id, widgets in self.ocr_button_widgets.items():
            widgets["size"] = button_size
            button = widgets["canvas"]
            button.configure(width=button_size, height=button_size)
            if use_canvas_controls and button_id in ("X", "A", "B", "C"):
                button.place_forget()
                continue
            bx, by = geometry["button_rects"][button_id]
            button.place(x=int(bx), y=int(by), width=button_size, height=button_size)
            if self.config.is_trajectory_mode and self.config.candidate_preview_enabled and button_id in ("X", "A", "B", "C"):
                hit_zones.append({
                    "id": button_id,
                    "label": widgets["base_text"],
                    "x1": float(bx),
                    "y1": float(by),
                    "x2": float(bx + button_size),
                    "y2": float(by + button_size),
                    "cx": float(bx + button_size / 2.0),
                    "cy": float(by + button_size / 2.0),
                    "radius": float(button_size / 2.0),
                })
        if use_canvas_controls:
            canvas_control_zones = build_canvas_reachable_preview_hit_zones(
                self.canvas_width,
                self.canvas_height,
                self.config.padding,
                button_size,
                reach_rect=self._reachable_trajectory_canvas_rect(),
            )
            hit_zones = canvas_zones_to_stage_zones(canvas_control_zones, draw_x, draw_y)
        self.preview_button_hit_zones = hit_zones
        self.preview_canvas_button_zones = canvas_control_zones
        self.preview_canvas_drawing_zone = self._drawing_zone_canvas_rect()

        self._redraw_canvas()
        self._update_ocr_toolbar_visuals()

    # -- Status cards -------------------------------------------------------

    def _card(self, parent, title, expand=False):
        tk = self.tk
        card = tk.Frame(parent, bg=theme.SURFACE, highlightthickness=1,
                        highlightbackground=theme.BORDER, padx=14, pady=10)
        card.pack(fill=tk.BOTH if expand else tk.X, expand=expand, pady=(0, 10))
        self.ttk.Label(card, text=title, style="CardTitle.TLabel").pack(anchor=tk.W, pady=(0, 6))
        return card

    def _card_label(self, parent, var=None, text=None, size=10, bold=False,
                    color=theme.TEXT, wraplength=330, inset=44, **pack_options):
        tk = self.tk
        options = {"textvariable": var} if var is not None else {"text": text}
        label = tk.Label(parent, bg=theme.SURFACE, fg=color, font=theme.font(size, bold=bold),
                         wraplength=wraplength, justify=tk.LEFT, anchor=tk.W, **options)
        label.pack(anchor=tk.W, fill=tk.X, **pack_options)
        self.dashboard_wrapped_labels.append((label, wraplength, inset))
        return label

    def _build_status_cards(self, parent):
        self.dashboard_wrapped_labels = []
        self.status_tone_bar = None
        self.workflow_canvas = None
        self.recognition_tile = None
        self.confidence_meter = None
        self.robot_state_dot = None

        if self.config.is_trajectory_mode:
            self._build_trajectory_status_cards(parent)
        elif self.config.demo_input_mode == "ocr_canvas":
            self._build_mouse_status_cards(parent)
        else:
            card = self._card(parent, "Status")
            for var in (self.ui_state_var, self.capture_var, self.confirm_var):
                self._card_label(card, var)
            card = self._card(parent, "Recognition")
            for var in (
                self.backend_status_var, self.recognizer_mode_var, self.recognizer_model_var,
                self.recognizer_status_var, self.recognizer_labels_var, *self.rank_vars,
            ):
                self._card_label(card, var)
            card = self._card(parent, "Safety")
            for text in (self.config.route_scope_text, self.config.confirm_policy_text):
                self._card_label(card, text=text, size=9)

    def _build_trajectory_status_cards(self, parent):
        preview = self.config.candidate_preview_enabled
        self._build_status_card(
            parent, self.traj_operator_status_var if preview else None, self.traj_clutch_mode_var)
        if not preview:
            return
        self._build_workflow_card(parent, "draw")
        self._build_recognition_card(
            parent, self.traj_operator_result_var, self.traj_operator_mapping_var,
            self.traj_operator_candidate_vars)
        self._build_robot_card(parent)
        if self.config.dashboard_debug_sample_panel:
            self._build_debug_card(parent, "Trajectory Diagnostics", (
                self.traj_mode_var, self.traj_window_var,
                self.traj_valid_var, self.traj_xyz_var, self.traj_count_var,
                self.traj_input_gate_var, self.traj_backend_var,
                self.traj_recognizer_detail_var, self.traj_result_var,
                self.traj_recognition_progress_var,
                self.traj_candidate_debug_var,
            ))
            self._build_debug_card(parent, "Drawing Sample", (
                self.traj_captured_sample_var,
                self.traj_candidate_points_var, self.traj_sample_state_var,
                self.traj_sample_cleanup_var,
            ))
            self._build_debug_card(parent, "Interaction", (
                self.traj_recording_var,
                self.traj_interaction_var, self.traj_semantic_var,
                self.traj_confirm_var,
            ))
        if self.config.trajectory_recording_enabled:
            rec_card = self._card(parent, "Segment recording (preview)")
            self.ttk.Button(rec_card, text="Start (record segment)  [S]",
                            command=self._start_recording).pack(anchor=self.tk.W)

    def _build_mouse_status_cards(self, parent):
        self._build_status_card(parent, self.mouse_operator_status_var, self.mouse_mode_var)
        self._build_workflow_card(parent, "start")
        self._build_recognition_card(
            parent, self.mouse_operator_result_var, self.mouse_operator_mapping_var,
            self.mouse_operator_candidate_vars)
        self._build_robot_card(parent)
        if self.config.dashboard_debug_sample_panel:
            self._build_debug_card(parent, "Mouse Diagnostics", (
                self.ui_state_var, self.capture_var,
                self.backend_status_var, self.recognizer_mode_var,
                self.recognizer_model_var, self.recognizer_labels_var,
                self.dwell_var, self.mouse_candidate_debug_var,
            ))

    def _build_status_card(self, parent, status_var, mode_var):
        """Next-step instruction with a tone stripe, and the pointer mode as a chip."""
        tk = self.tk
        card = self._card(parent, "Status")
        row = tk.Frame(card, bg=theme.SURFACE)
        row.pack(fill=tk.X)
        self.status_tone_bar = tk.Frame(row, bg=theme.tone_colors("neutral")[0], width=4)
        self.status_tone_bar.pack(side=tk.LEFT, fill=tk.Y)
        body = tk.Frame(row, bg=theme.SURFACE)
        body.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(12, 0))
        if status_var is not None:
            # Two fixed lines, so a wrapping instruction does not shift the cards below.
            label = self._card_label(body, status_var, size=14, bold=True, inset=62)
            label.configure(height=2, anchor=tk.NW)
        chip = tk.Label(body, textvariable=mode_var, bg=theme.APP_BG, fg=theme.TEXT_SECONDARY,
                        font=theme.font(9, bold=True), padx=8, pady=2)
        chip.pack(anchor=tk.W, pady=(6, 0))

    def _build_workflow_card(self, parent, stage):
        tk = self.tk
        card = self._card(parent, "Progress")
        self.workflow_stage = stage
        self.workflow_canvas = tk.Canvas(card, height=74, bg=theme.SURFACE, highlightthickness=0)
        self.workflow_canvas.pack(fill=tk.X)
        self.workflow_canvas.bind("<Configure>", lambda _event: self._draw_workflow())

    def _build_recognition_card(self, parent, result_var, mapping_var, row_vars):
        tk = self.tk
        card = self._card(parent, "Recognition", expand=True)
        top = tk.Frame(card, bg=theme.SURFACE)
        top.pack(fill=tk.X)
        size = RECOGNITION_TILE_SIZE
        self.recognition_tile = tk.Canvas(top, width=size, height=size, bg=theme.SURFACE,
                                          highlightthickness=0)
        self.recognition_tile.pack(side=tk.LEFT, anchor=tk.N)
        text_column = tk.Frame(top, bg=theme.SURFACE)
        text_column.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(14, 0))
        column_inset = 44 + size + 14
        self._card_label(text_column, result_var, size=13, bold=True, wraplength=200,
                         inset=column_inset, pady=(4, 0))
        self._card_label(text_column, mapping_var, size=11, color=theme.TEXT_SECONDARY,
                         wraplength=200, inset=column_inset, pady=(4, 0))
        self._card_label(card, row_vars[0], size=10, bold=True, pady=(12, 6))
        self.confidence_meter = tk.Canvas(card, height=10, bg=theme.SURFACE, highlightthickness=0)
        self.confidence_meter.pack(fill=tk.X)
        self.confidence_meter.bind("<Configure>", lambda _event: self._draw_recognition_visual())
        for var in row_vars[1:]:
            self._card_label(card, var, size=10, color=theme.TEXT_SECONDARY, pady=(8, 0))
        self.recognition_visual = {"label": "?", "tone": "neutral", "fraction": None}
        self._draw_recognition_visual()

    def _build_robot_card(self, parent):
        """Last task command and backend execution state; shown when confirms can dispatch."""
        if not (self.config.integrated_confirm_enabled and self.config.publish_confirmed_label):
            return
        tk = self.tk
        card = self._card(parent, "Robot task")
        self._card_label(card, self.task_var, size=12, bold=True)
        row = tk.Frame(card, bg=theme.SURFACE)
        row.pack(fill=tk.X, pady=(4, 0))
        self.robot_state_dot = tk.Canvas(row, width=12, height=12, bg=theme.SURFACE,
                                         highlightthickness=0)
        self.robot_state_dot.pack(side=tk.LEFT)
        self._card_label(row, self.gazebo_var, size=10, color=theme.TEXT_SECONDARY,
                         inset=64, padx=(6, 0))
        self._show_robot_state_tone("neutral")

    def _build_debug_card(self, parent, title, variables):
        card = self._card(parent, title)
        for var in variables:
            self._card_label(card, var, size=9, color=theme.TEXT_SECONDARY)

    # -- Card updates -------------------------------------------------------

    def _show_status_tone(self, tone):
        if self.status_tone_bar is not None:
            self.status_tone_bar.configure(bg=theme.tone_colors(tone)[0])

    def _show_workflow_stage(self, stage):
        self.workflow_stage = stage
        self._draw_workflow()

    def _draw_workflow(self):
        if self.workflow_canvas is None:
            return
        steps = operator_workflow_steps(self.config.interaction_profile)
        draw_workflow_stepper(
            self.workflow_canvas, self.workflow_canvas.winfo_width(), steps,
            workflow_step_index(self.workflow_stage, steps))

    def _show_recognition_visual(self, label, tone, fraction):
        self.recognition_visual = {"label": label, "tone": tone, "fraction": fraction}
        self._draw_recognition_visual()

    def _draw_recognition_visual(self):
        if self.recognition_tile is None:
            return
        visual = self.recognition_visual
        draw_symbol_tile(self.recognition_tile, RECOGNITION_TILE_SIZE,
                         visual["label"], visual["tone"])
        draw_confidence_meter(self.confidence_meter, self.confidence_meter.winfo_width(),
                              visual["fraction"], visual["tone"])

    def _show_robot_state_tone(self, tone):
        if self.robot_state_dot is None:
            return
        self.robot_state_dot.delete("all")
        self.robot_state_dot.create_oval(2, 2, 11, 11, fill=theme.tone_colors(tone)[0], outline="")

    def _build_safety_footer(self, parent):
        if not self.config.footer_text:
            return
        tk = self.tk
        footer = tk.Frame(parent, bg=theme.BORDER, pady=8, padx=16)
        footer.pack(fill=tk.X, side=tk.BOTTOM)
        tk.Label(footer, text=self.config.footer_text, fg=theme.TEXT_SECONDARY, bg=theme.BORDER,
                 font=theme.font(10, bold=True)).pack(anchor=tk.CENTER)
