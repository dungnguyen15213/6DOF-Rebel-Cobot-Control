"""Thread-safe tracking of one controller-side program execution (CRI EXECACK/EXECPAUSE/EXECEND)."""

import threading
import time
from dataclasses import dataclass
from typing import Callable


@dataclass(frozen=True)
class ProgramOutcome:
    """Final result of one program run.

    success is True only for EXECEND with reason PLAN. ``reason`` is the CRI reason
    (PLAN, USER, PLC, ERROR) or one of the local reasons TIMEOUT, ABORTED, DISCONNECTED.
    """

    success: bool
    reason: str
    message: str = ""
    last_command: int | None = None


class ProgramExecution:
    """Fed by the receive thread, waited on by the thread that started the program.

    Messages are ignored until ``arm()`` is called (right before StartProgram), so an EXECEND that
    belongs to an earlier move or program can never be mistaken for the completion of this one.
    EXECEND frames without a reason are plain Move completions and are ignored for programs.
    """

    IDLE = "idle"
    ARMED = "armed"
    FINISHED = "finished"

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._state = self.IDLE
        self._outcome: ProgramOutcome | None = None
        self._last_command: int | None = None
        self.paused = False

    def arm(self) -> None:
        with self._condition:
            self._state = self.ARMED
            self._outcome = None
            self._last_command = None
            self.paused = False

    def disarm(self) -> None:
        with self._condition:
            self._state = self.IDLE
            self._outcome = None
            self.paused = False

    @property
    def armed(self) -> bool:
        with self._condition:
            return self._state == self.ARMED

    @property
    def last_command(self) -> int | None:
        with self._condition:
            return self._last_command

    def handle(self, notification: dict) -> None:
        """Called from the receive thread for every parsed EXEC* notification."""
        event = notification.get("exec_event")
        if event is None:
            return
        with self._condition:
            if self._state != self.ARMED:
                return
            cmd_nr = notification.get("cmd_nr")
            if cmd_nr is not None:
                self._last_command = cmd_nr
            if event == "EXECACK":
                self.paused = False
            elif event == "EXECPAUSE":
                self.paused = True
            elif event in ("EXECEND", "EXECERROR"):
                reason = notification.get("reason")
                if reason is None or reason not in ("PLAN", "USER", "PLC", "ERROR"):
                    return  # e.g. MOVETO belongs to a single move, not the program
                error = notification.get("error")
                self._outcome = ProgramOutcome(
                    success=reason == "PLAN" and event == "EXECEND" and not error,
                    reason=reason,
                    message=error or "",
                    last_command=self._last_command,
                )
                self._state = self.FINISHED
                self._condition.notify_all()

    def wait(self, timeout: float, abort: Callable[[], str | None] | None = None,
             poll_sec: float = 0.05) -> ProgramOutcome:
        """Block for the outcome. ``abort()`` returns a reason code to give up early (stop, disconnect)."""
        deadline = time.monotonic() + timeout
        with self._condition:
            while True:
                if self._outcome is not None:
                    return self._outcome
                if abort is not None:
                    reason = abort()
                    if reason:
                        return ProgramOutcome(False, reason, last_command=self._last_command)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return ProgramOutcome(False, "TIMEOUT", "The program did not finish in time.",
                                          self._last_command)
                self._condition.wait(min(poll_sec, remaining))
