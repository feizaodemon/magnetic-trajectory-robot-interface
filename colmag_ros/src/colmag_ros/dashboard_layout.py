"""Tk window layout for the trajectory dashboard."""

from colmag_ros.dashboard_geometry import (
    build_canvas_reachable_preview_hit_zones,
    canvas_zones_to_stage_zones,
    compute_dashboard_panel_widths,
    compute_ocr_stage_geometry,
)
from colmag_ros.dashboard_texts import (
    TRAJECTORY_MODE,
    default_header_subtitle,
    default_header_title,
    default_main_panel_title,
)


class DashboardLayoutMixin:
    """Builds the dashboard window: header, canvas area, status cards, footer."""

    def _build_ui(self):
        tk = self.tk
        ttk = self.ttk

        style = ttk.Style(self.root)
        style.theme_use('clam')
        style.configure("Header.TFrame", background="#2C3E50")
        style.configure("HeaderTitle.TLabel", background="#2C3E50", foreground="#ECF0F1", font=("TkDefaultFont", 16, "bold"))
        style.configure("HeaderSub.TLabel", background="#2C3E50", foreground="#BDC3C7", font=("TkDefaultFont", 10))
        style.configure("Badge.TLabel", background="#F39C12", foreground="#FFFFFF", font=("TkDefaultFont", 10, "bold"), padding=4)
        style.configure("Card.TFrame", background="#FFFFFF", relief="flat")
        style.configure("CardTitle.TLabel", background="#FFFFFF", foreground="#7F8C8D", font=("TkDefaultFont", 10, "bold"))

        main_frame = tk.Frame(self.root, bg="#F0F0F0")
        main_frame.pack(fill=tk.BOTH, expand=True)

        self._build_header(main_frame)

        content_frame = tk.Frame(main_frame, bg="#F0F0F0")
        content_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=10)
        self.dashboard_content_frame = content_frame

        left_frame = tk.Frame(content_frame, bg="#F0F0F0")
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.dashboard_left_frame = left_frame

        self._build_canvas_area(left_frame)

        right_frame = tk.Frame(content_frame, bg="#F0F0F0", width=380)
        right_frame.pack_propagate(False)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(15, 0))
        self.dashboard_right_frame = right_frame
        self._build_status_cards(right_frame)
        content_frame.bind("<Configure>", self._layout_dashboard_columns)
        right_frame.bind("<Configure>", self._layout_dashboard_status_wrap)

        self._build_safety_footer(main_frame)

    def _layout_dashboard_columns(self, event=None):
        total_width = getattr(event, "width", self.dashboard_content_frame.winfo_width())
        _left_width, right_width = compute_dashboard_panel_widths(total_width)
        self.dashboard_right_frame.configure(width=right_width)

    def _layout_dashboard_status_wrap(self, event=None):
        right_width = getattr(event, "width", self.dashboard_right_frame.winfo_width())
        for label, base_wraplength in self.dashboard_wrapped_labels:
            label.configure(wraplength=max(base_wraplength, int(right_width) - 40))

    def _build_header(self, parent):
        tk = self.tk
        ttk = self.ttk
        header = ttk.Frame(parent, style="Header.TFrame")
        header.pack(fill=tk.X)

        left_frame = ttk.Frame(header, style="Header.TFrame")
        left_frame.pack(side=tk.LEFT, padx=15, pady=10)

        title = default_header_title(self.config.demo_input_mode)
        if self.config.subtitle_text:
            sub = self.config.subtitle_text
        else:
            sub = default_header_subtitle(self.config.demo_input_mode)
        ttk.Label(left_frame, text=title, style="HeaderTitle.TLabel").pack(anchor=tk.W)
        ttk.Label(left_frame, text=sub, style="HeaderSub.TLabel").pack(anchor=tk.W)

        right_frame = ttk.Frame(header, style="Header.TFrame")
        right_frame.pack(side=tk.RIGHT, padx=15, pady=10)
        if self.config.control_mode_enabled:
            mode_frame = ttk.Frame(right_frame, style="Header.TFrame")
            mode_frame.pack(side=tk.LEFT, padx=(0, 10))
            ttk.Label(
                mode_frame,
                textvariable=self.control_mode_var,
                style="HeaderSub.TLabel",
            ).pack(anchor=tk.E)
            self.control_mode_button = ttk.Button(
                mode_frame,
                textvariable=self.control_mode_button_var,
                command=self._request_control_mode_switch,
            )
            self.control_mode_button.pack(anchor=tk.E, pady=(3, 0))
        if self.config.safety_badge:
            ttk.Label(
                right_frame, text=self.config.safety_badge, style="Badge.TLabel"
            ).pack(anchor=tk.E)

    def _build_canvas_area(self, parent):
        tk = self.tk
        stage_host = tk.Frame(parent, bg="#F0F0F0")
        stage_host.pack(fill=tk.BOTH, expand=True, pady=10, padx=10)
        self.ocr_stage_frame = stage_host
        stage_host.bind("<Configure>", self._layout_ocr_stage)

        stage = tk.Frame(stage_host, bg="#EEF2F5", bd=1, relief=tk.SOLID, highlightthickness=1, highlightbackground="#CBD5E1")
        self.ocr_stage_card = stage

        title_text = default_main_panel_title(self.config.demo_input_mode)
        self.ocr_title_label = tk.Label(stage, text=title_text, bg="#EEF2F5", fg="#1F2937", font=("TkDefaultFont", 14, "bold"))
        self.ocr_hint_label = tk.Label(stage, textvariable=self.ocr_hint_var, bg="#EEF2F5", fg="#64748B", font=("TkDefaultFont", 10, "bold"))

        self.canvas = tk.Canvas(stage, bg="white", highlightthickness=1, highlightbackground="#CBD5E1")

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
            widget = tk.Canvas(stage, bg="#EEF2F5", highlightthickness=0, cursor="hand2")
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

        self.ocr_title_label.place(x=pad, y=14)
        self.ocr_hint_label.configure(wraplength=max(180, stage_w - 2 * pad))
        self.ocr_hint_label.place(x=pad, y=38)
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

    def _build_status_cards(self, parent):
        tk = self.tk
        ttk = self.ttk
        self.dashboard_wrapped_labels = []

        def create_card(
            title, *vars, static_texts=None, wraplength=340, font_size=10,
            emphasized_vars=(), secondary_vars=(), expand=False,
        ):
            card = ttk.Frame(parent, style="Card.TFrame", padding=8)
            card.pack(fill=tk.BOTH if expand else tk.X, expand=expand, pady=(0, 8))
            ttk.Label(card, text=title, style="CardTitle.TLabel").pack(anchor=tk.W, pady=(0, 5))
            if static_texts:
                for st in static_texts:
                    label = ttk.Label(card, text=st, background="white", font=("TkDefaultFont", 9), wraplength=wraplength, justify=tk.LEFT)
                    label.pack(anchor=tk.W, fill=tk.X)
                    self.dashboard_wrapped_labels.append((label, wraplength))
            for var in vars:
                if var:
                    emphasized = var in emphasized_vars
                    font = ("TkDefaultFont", font_size + 3, "bold") if emphasized else ("TkDefaultFont", font_size)
                    foreground = "#64748B" if var in secondary_vars else "#1F2937"
                    label = ttk.Label(
                        card, textvariable=var, background="white",
                        foreground=foreground, font=font,
                        wraplength=wraplength, justify=tk.LEFT,
                    )
                    label.pack(anchor=tk.W, fill=tk.X)
                    self.dashboard_wrapped_labels.append((label, wraplength))
            return card

        if self.config.is_trajectory_mode:
            self._build_trajectory_status_cards(parent, create_card)
        elif self.config.demo_input_mode == "ocr_canvas":
            self._build_mouse_status_cards(create_card)
        else:
            create_card("Status", self.ui_state_var, self.capture_var, self.confirm_var)
            create_card(
                "Recognition",
                self.backend_status_var,
                self.recognizer_mode_var,
                self.recognizer_model_var,
                self.recognizer_status_var,
                self.recognizer_labels_var,
                self.rank_vars[0],
                self.rank_vars[1],
                self.rank_vars[2],
            )
            create_card("Safety", static_texts=[
                self.config.route_scope_text,
                self.config.confirm_policy_text,
            ])

    def _build_trajectory_status_cards(self, parent, create_card):
        create_card("Interaction", self.traj_clutch_mode_var, font_size=13,
                    emphasized_vars=(self.traj_clutch_mode_var,))
        if not self.config.candidate_preview_enabled:
            return
        create_card("Progress", self.traj_workflow_var, font_size=11,
                    emphasized_vars=(self.traj_workflow_var,))
        create_card(
            "Recognition", self.traj_operator_result_var,
            self.traj_operator_mapping_var, *self.traj_operator_candidate_vars,
            wraplength=350, font_size=11,
            emphasized_vars=(self.traj_operator_result_var,),
            secondary_vars=tuple(self.traj_operator_candidate_vars[1:]),
            expand=True,
        )
        create_card("Status", self.traj_operator_status_var, font_size=12,
                    emphasized_vars=(self.traj_operator_status_var,))
        if self.config.dashboard_debug_sample_panel:
            create_card(
                "Trajectory Diagnostics", self.traj_mode_var, self.traj_window_var,
                self.traj_valid_var, self.traj_xyz_var, self.traj_count_var,
                self.traj_input_gate_var, self.traj_backend_var,
                self.traj_recognizer_detail_var, self.traj_result_var,
                self.traj_recognition_progress_var,
                self.traj_candidate_debug_var, font_size=9,
            )
            create_card(
                "Drawing Sample", self.traj_captured_sample_var,
                self.traj_candidate_points_var, self.traj_sample_state_var,
                self.traj_sample_cleanup_var,
            )
            create_card(
                "Interaction", self.traj_recording_var,
                self.traj_interaction_var, self.traj_semantic_var,
                self.traj_confirm_var,
            )
        if self.config.trajectory_recording_enabled:
            rec_card = self.ttk.Frame(parent, style="Card.TFrame", padding=10)
            rec_card.pack(fill=self.tk.X, pady=(0, 10))
            self.ttk.Label(rec_card, text="Segment recording (preview)",
                           style="CardTitle.TLabel").pack(anchor=self.tk.W, pady=(0, 5))
            self.ttk.Button(rec_card, text="Start (record segment)  [S]",
                            command=self._start_recording).pack(anchor=self.tk.W)

    def _build_mouse_status_cards(self, create_card):
        create_card("Interaction", self.mouse_mode_var, font_size=13,
                    emphasized_vars=(self.mouse_mode_var,))
        create_card("Progress", self.mouse_workflow_var, font_size=11,
                    emphasized_vars=(self.mouse_workflow_var,))
        create_card(
            "Recognition", self.mouse_operator_result_var,
            self.mouse_operator_mapping_var,
            *self.mouse_operator_candidate_vars, font_size=11,
            emphasized_vars=(self.mouse_operator_result_var,),
            secondary_vars=tuple(self.mouse_operator_candidate_vars[1:]),
            expand=True,
        )
        create_card("Status", self.mouse_operator_status_var, font_size=12,
                    emphasized_vars=(self.mouse_operator_status_var,))
        if self.config.dashboard_debug_sample_panel:
            create_card(
                "Mouse Diagnostics", self.ui_state_var, self.capture_var,
                self.backend_status_var, self.recognizer_mode_var,
                self.recognizer_model_var, self.recognizer_labels_var,
                self.dwell_var, self.mouse_candidate_debug_var, font_size=9,
            )

    def _build_safety_footer(self, parent):
        if not self.config.footer_text:
            return
        tk = self.tk
        ttk = self.ttk
        footer = tk.Frame(parent, bg="#EAECEE", pady=10, padx=15)
        footer.pack(fill=tk.X, side=tk.BOTTOM)

        text = self.config.footer_text
        ttk.Label(footer, text=text, foreground="#7F8C8D", background="#EAECEE", font=("TkDefaultFont", 10, "bold")).pack(anchor=tk.CENTER)
