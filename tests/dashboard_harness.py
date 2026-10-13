"""Headless harness for MagneticTrajectoryDashboardNode.

The node imports rospy, std_msgs and tkinter inside
``_initialize_runtime_dependencies``. The harness puts recording fakes for
those modules into ``sys.modules`` and then builds a real node through its
normal ``__init__``, so tests can drive topics, key presses and Tk callbacks
without a display or a ROS master.

Tk ``after`` callbacks are queued, not run. ``flush()`` runs the zero-delay
ones (variable updates and the coalesced redraw); timed jobs such as the 50 ms
dwell loop stay queued until a test runs them explicitly.
"""

import itertools
import json
import sys
import time
import types


SAMPLE_CAPTURE = {
    "sample_id": "cap_1",
    "sequence_id": 1,
    "points": [{"x": 0.0, "y": 0.0}, {"x": 0.01, "y": 0.02}, {"x": 0.02, "y": 0.0}],
}
SAMPLE_CANDIDATES = {
    "backend": "dtw_template_bank",
    "sample_id": "cap_1",
    "sequence_id": 1,
    "candidates": [
        {"rank": 1, "label": "2", "confidence": 0.88},
        {"rank": 2, "label": "1", "confidence": 0.20},
    ],
}


class FakeTclError(Exception):
    pass


class FakeVar:
    def __init__(self, master=None, value=None):
        self._value = value

    def get(self):
        return self._value

    def set(self, value):
        self._value = value


class FakeWidget:
    """Records construction options, geometry calls and event bindings."""

    def __init__(self, master=None, **options):
        self.master = master
        self.options = dict(options)
        self.children = []
        self.bindings = {}
        self.pack_options = None
        self.place_options = None
        self.size = (800, 600)
        if isinstance(master, FakeWidget):
            master.children.append(self)

    def pack(self, **options):
        self.pack_options = options

    def pack_propagate(self, _flag):
        pass

    def place(self, **options):
        self.place_options = options

    def place_forget(self):
        self.place_options = None

    def configure(self, **options):
        self.options.update(options)

    config = configure

    def bind(self, sequence, callback, add=None):
        self.bindings.setdefault(sequence, []).append(callback)

    def tag_bind(self, _tag, sequence, callback):
        self.bind(sequence, callback)

    def winfo_width(self):
        return self.size[0]

    def winfo_height(self):
        return self.size[1]

    def winfo_rootx(self):
        return 0

    def winfo_rooty(self):
        return 0

    def destroy(self):
        pass

    def descendants(self):
        for child in self.children:
            yield child
            yield from child.descendants()


class FakeCanvas(FakeWidget):
    def __init__(self, master=None, **options):
        super().__init__(master, **options)
        self.items = []
        self._ids = itertools.count(1)

    def delete(self, tag):
        if tag == "all":
            self.items = []

    def _create(self, kind, coords, options):
        self.items.append((kind, coords, options))
        return next(self._ids)

    def create_line(self, *coords, **options):
        return self._create("line", coords, options)

    def create_oval(self, *coords, **options):
        return self._create("oval", coords, options)

    def create_rectangle(self, *coords, **options):
        return self._create("rectangle", coords, options)

    def create_text(self, *coords, **options):
        return self._create("text", coords, options)

    def create_arc(self, *coords, **options):
        return self._create("arc", coords, options)

    def drawn(self, kind):
        return [options for item_kind, _coords, options in self.items if item_kind == kind]


class FakeTk(FakeWidget):
    def __init__(self):
        super().__init__(None)
        self.window = {}
        self.jobs = []
        self._job_ids = itertools.count(1)

    def title(self, text):
        self.window["title"] = text

    def geometry(self, spec):
        self.window["geometry"] = spec

    def minsize(self, width, height):
        self.window["minsize"] = (width, height)

    def protocol(self, name, callback):
        self.window[name] = callback

    def after(self, delay_ms, callback=None, *args):
        job_id = "after#%d" % next(self._job_ids)
        self.jobs.append((job_id, delay_ms, callback, args))
        return job_id

    def after_cancel(self, job_id):
        self.jobs = [job for job in self.jobs if job[0] != job_id]

    def mainloop(self):
        pass

    def quit(self):
        pass


class FakeStyle:
    def __init__(self, master=None):
        self.styles = {}

    def theme_use(self, _name):
        pass

    def configure(self, name, **options):
        self.styles[name] = options


def _fake_tk_modules():
    tk = types.ModuleType("tkinter")
    tk.Tk = FakeTk
    tk.Frame = FakeWidget
    tk.Label = FakeWidget
    tk.Canvas = FakeCanvas
    tk.StringVar = FakeVar
    tk.TclError = FakeTclError
    for name in ("ARC", "BOTH", "BOTTOM", "CENTER", "E", "LEFT", "RIGHT", "SOLID", "W", "X", "Y"):
        setattr(tk, name, name.lower())
    ttk = types.ModuleType("tkinter.ttk")
    ttk.Frame = FakeWidget
    ttk.Label = FakeWidget
    ttk.Button = FakeWidget
    ttk.Style = FakeStyle
    tk.ttk = ttk
    return tk, ttk


class FakeString:
    def __init__(self, data=""):
        self.data = data


class FakePublisher:
    def __init__(self, topic, msg_class, **options):
        self.topic = topic
        self.options = options
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeRospy(types.ModuleType):
    """Stand-in for the rospy module: params from a dict, recorded pub/sub and logs."""

    def __init__(self, params):
        super().__init__("rospy")
        self.params = dict(params)
        self.publishers = []
        self.subscribers = {}
        self.registrations = []  # ("pub" | "sub", topic) in creation order
        self.logs = []

    def get_param(self, name, default=None):
        return self.params.get(name, default)

    def Publisher(self, topic, msg_class, **options):
        publisher = FakePublisher(topic, msg_class, **options)
        self.publishers.append(publisher)
        self.registrations.append(("pub", topic))
        return publisher

    def Subscriber(self, topic, msg_class, callback, **options):
        self.subscribers.setdefault(topic, []).append(callback)
        self.registrations.append(("sub", topic))
        return (topic, callback)

    def _log(self, level, fmt, *args):
        self.logs.append((level, fmt % args if args else fmt))

    def loginfo(self, fmt, *args):
        self._log("info", fmt, *args)

    loginfo_once = loginfo

    def logwarn(self, fmt, *args):
        self._log("warn", fmt, *args)

    def logerr(self, fmt, *args):
        self._log("error", fmt, *args)

    def signal_shutdown(self, _reason):
        pass

    def on_shutdown(self, _callback):
        pass


class Event:
    def __init__(self, **fields):
        self.__dict__.update(fields)


class DashboardHarness:
    """A real dashboard node wired to fake rospy / std_msgs / tkinter.

    ``params`` are private node params without the ``~`` prefix. Time is a
    manual clock: ``time.time()`` returns ``harness.now`` until ``advance``.
    Code that bound ``time.time`` at import (e.g. a ``now_fn=time.time``
    default) still reads the real clock.
    """

    def __init__(self, monkeypatch, **params):
        from magnetic_trajectory_dashboard_node import MagneticTrajectoryDashboardNode

        self.now = 1000.0
        monkeypatch.setattr(time, "time", lambda: self.now)
        self.rospy = FakeRospy({"~" + name: value for name, value in params.items()})
        tk, ttk = _fake_tk_modules()
        std_msgs = types.ModuleType("std_msgs")
        std_msgs.msg = types.ModuleType("std_msgs.msg")
        std_msgs.msg.String = FakeString
        for name, module in (
            ("rospy", self.rospy),
            ("std_msgs", std_msgs),
            ("std_msgs.msg", std_msgs.msg),
            ("tkinter", tk),
            ("tkinter.ttk", ttk),
        ):
            monkeypatch.setitem(sys.modules, name, module)
        self.node = MagneticTrajectoryDashboardNode()
        self.root = self.node.root
        self.flush()

    def advance(self, seconds):
        self.now += seconds

    def flush(self):
        """Run queued zero-delay ``after`` callbacks, including ones they queue."""
        for _ in range(1000):
            due = [job for job in self.root.jobs if job[1] == 0]
            if not due:
                return
            self.root.jobs = [job for job in self.root.jobs if job[1] != 0]
            for _job_id, _delay, callback, args in due:
                if callback is not None:
                    callback(*args)
        raise AssertionError("zero-delay after callbacks keep rescheduling")

    def run_jobs(self, delay_ms):
        """Run the queued ``after`` jobs that were scheduled with ``delay_ms``."""
        due = [job for job in self.root.jobs if job[1] == delay_ms]
        self.root.jobs = [job for job in self.root.jobs if job[1] != delay_ms]
        for _job_id, _delay, callback, args in due:
            callback(*args)
        self.flush()
        return len(due)

    def send(self, topic, payload):
        """Deliver a String message to the node's subscribers of ``topic``."""
        data = payload if isinstance(payload, str) else json.dumps(payload)
        callbacks = self.rospy.subscribers.get(topic, [])
        assert callbacks, "dashboard does not subscribe to %s" % topic
        for callback in callbacks:
            callback(FakeString(data))
        self.flush()

    def send_sample_and_candidates(self):
        """Publish a captured sample and its recognizer candidates, as the DTW route does."""
        self.send(self.node.symbol_capture_topic, SAMPLE_CAPTURE)
        self.send(self.node.symbol_candidates_topic, SAMPLE_CANDIDATES)

    def fire(self, widget, sequence, **event_fields):
        """Invoke the callbacks bound to ``sequence`` on ``widget``."""
        results = [callback(Event(**event_fields)) for callback in widget.bindings.get(sequence, [])]
        self.flush()
        return results

    def press_key(self, char, keysym=None):
        return self.fire(self.root, "<Key>", char=char, keysym=keysym or char)

    def draw_stroke(self, *canvas_points):
        """Drag the mouse across the drawing canvas through ``canvas_points``."""
        canvas = self.node.canvas
        (x, y), rest = canvas_points[0], canvas_points[1:]
        self.fire(canvas, "<ButtonPress-1>", x=x, y=y)
        for x, y in rest:
            self.fire(canvas, "<B1-Motion>", x=x, y=y)
        self.fire(canvas, "<ButtonRelease-1>", x=x, y=y)

    def publisher_topics(self):
        return [publisher.topic for publisher in self.rospy.publishers]

    def messages(self, topic):
        return [
            msg.data
            for publisher in self.rospy.publishers
            if publisher.topic == topic
            for msg in publisher.messages
        ]

    def published_json(self, topic):
        return [json.loads(data) for data in self.messages(topic)]

    def widgets(self):
        return list(self.root.descendants())

    def card_titles(self):
        return [
            widget.options["text"]
            for widget in self.widgets()
            if widget.options.get("style") == "CardTitle.TLabel"
        ]

    def card(self, title):
        for widget in self.widgets():
            if widget.options.get("style") == "CardTitle.TLabel" and widget.options.get("text") == title:
                return widget.master
        raise AssertionError("no status card titled %r" % title)

    def shown_variables(self):
        """Tk variables bound to a visible label or button."""
        return [
            widget.options["textvariable"]
            for widget in self.widgets()
            if "textvariable" in widget.options
        ]

    def canvas_texts(self):
        return [options.get("text", "") for options in self.node.canvas.drawn("text")]

    def ui_texts(self):
        """Every text the operator can see: static labels, bound variables, canvas text."""
        static = [str(widget.options["text"]) for widget in self.widgets() if "text" in widget.options]
        variables = [str(var.get()) for var in self.shown_variables()]
        return static + variables + self.canvas_texts()
