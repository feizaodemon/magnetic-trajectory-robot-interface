"""Subprocess seam, diagnostic log and the owned process groups.

Moved unchanged from scripts/fr3_lab_runtime.py, which re-exports every
name. Real-FR3 lab tooling: keep values and behaviour as they are.
"""

from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Iterable, Optional, Sequence

from fr3_lab.textparse import (
    command_fingerprint,
    redact,
    _text_or_empty,
)


@dataclass(frozen=True)
class CommandResult:
    returncode: int
    stdout: str = ""
    stderr: str = ""


class CommandRunner:
    """Small subprocess seam used by the operator and its tests."""

    def run(self, args: Sequence[str], timeout: Optional[float] = None) -> CommandResult:
        try:
            completed = subprocess.run(
                list(args),
                text=True,
                capture_output=True,
                timeout=timeout,
                check=False,
            )
        except (subprocess.TimeoutExpired, TimeoutError) as exc:
            stdout = _text_or_empty(getattr(exc, "stdout", ""))
            stderr = _text_or_empty(getattr(exc, "stderr", ""))
            return CommandResult(124, stdout, stderr)
        except OSError as exc:
            return CommandResult(127, "", str(exc))
        return CommandResult(completed.returncode, completed.stdout, completed.stderr)


class DiagnosticLog:
    def __init__(self, path: Path, secrets: Iterable[str]):
        self.path = path
        self.secrets = tuple(secret for secret in secrets if secret)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = self.path.open("a", encoding="utf-8")

    def write(self, message: str) -> None:
        timestamp = datetime.now(timezone.utc).isoformat()
        self._stream.write("{} {}\n".format(timestamp, redact(message, self.secrets)))
        self._stream.flush()

    def close(self) -> None:
        self._stream.close()


class OwnedProcess:
    """Own one process group and clean up only that group."""

    def __init__(
        self,
        args: Sequence[str],
        log: DiagnosticLog,
        secrets: Iterable[str],
        popen_factory: Callable[..., subprocess.Popen] = subprocess.Popen,
        killpg: Callable[[int, int], None] = os.killpg,
        getpgid: Callable[[int], int] = os.getpgid,
    ):
        self.args = tuple(args)
        self.log = log
        self.secrets = tuple(secret for secret in secrets if secret)
        self._killpg = killpg
        self._process = popen_factory(
            list(self.args),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        self.pid = int(self._process.pid)
        self.pgid = int(getpgid(self.pid))
        self.started_at = time.monotonic()
        self.command_fingerprint = command_fingerprint(self.args, self.secrets)
        self._reader = threading.Thread(target=self._pump_output, daemon=True)
        self._reader.start()
        self.log.write(
            "owned_process_started pid={} pgid={} fingerprint={}".format(
                self.pid, self.pgid, self.command_fingerprint
            )
        )

    def _pump_output(self) -> None:
        stream = self._process.stdout
        if stream is None:
            return
        for line in stream:
            self.log.write("process " + line.rstrip("\n"))

    def poll(self) -> Optional[int]:
        return self._process.poll()

    def cleanup(self, interrupt_timeout: float = 10.0, term_timeout: float = 5.0) -> bool:
        if self._process.poll() is not None:
            self._reader.join(timeout=1.0)
            self.log.write("owned_process_already_stopped")
            return True
        if not self._signal_and_wait(signal.SIGINT, interrupt_timeout):
            if not self._signal_and_wait(signal.SIGTERM, term_timeout):
                self.log.write("owned_process_cleanup_incomplete")
                return False
        self._reader.join(timeout=1.0)
        self.log.write("owned_process_cleanup_complete")
        return True

    def _signal_and_wait(self, sig: int, timeout: float) -> bool:
        try:
            self._killpg(self.pgid, sig)
        except ProcessLookupError:
            return True
        except OSError as exc:
            self.log.write("owned_process_signal_failed signal={} error={}".format(sig, exc))
            return False
        try:
            self._process.wait(timeout=timeout)
            return True
        except subprocess.TimeoutExpired:
            self.log.write("owned_process_wait_timeout signal={}".format(sig))
            return False


class GoalObserver:
    """Count FJT goal messages without storing their payloads."""

    def __init__(self, topic: str, log: DiagnosticLog):
        self._count = 0
        self._lock = threading.Lock()
        self._process = subprocess.Popen(
            ["rostopic", "echo", "-p", topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        self._pgid = os.getpgid(self._process.pid)
        self._log = log
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    def _read(self) -> None:
        stream = self._process.stdout
        if stream is None:
            return
        header_seen = False
        for line in stream:
            if not line.strip():
                continue
            if not header_seen:
                header_seen = True
                continue
            with self._lock:
                self._count += 1

    @property
    def count(self) -> int:
        with self._lock:
            return self._count

    def cleanup(self) -> bool:
        if self._process.poll() is not None:
            return True
        try:
            os.killpg(self._pgid, signal.SIGINT)
            self._process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(self._pgid, signal.SIGTERM)
                self._process.wait(timeout=2.0)
            except (subprocess.TimeoutExpired, OSError):
                self._log.write("goal_observer_cleanup_incomplete")
                return False
        except ProcessLookupError:
            return True
        self._reader.join(timeout=1.0)
        return True
