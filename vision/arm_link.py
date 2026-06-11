"""
Pi-side serial link to the arm Mega over USB.

Wraps pyserial and maps Python calls onto the newline-terminated commands
the arm's USB-serial parser understands (see arm_mega handleUsbCommand):
    p  -> pickup sequence    x  -> abort
    h  -> home               a  -> toggle auto mode
    ?  -> status dump        v  -> hover (lift to approach pose)
    b<deg> -> set base angle absolutely (visual-servo nudge)

Commands are newline-terminated — the Mega's parser is line-buffered now
(so b<deg> can carry a multi-digit argument), so a bare byte won't run.

Auto-detects the Mega by USB VID/PID so /dev/ttyACMx ordering doesn't
matter (controller and arm both enumerate as ACM*; only VID/PID is stable).

Usage:
    from arm_link import ArmLink
    with ArmLink() as arm:
        arm.set_auto(True)
        arm.pickup()
        for line in arm.messages(timeout=10):
            print(line)

Interactive shell:
    python3 vision/arm_link.py
"""
import re
import time

import serial
import serial.tools.list_ports

# Arduino Mega 2560 R3 (ATmega16U2 USB bridge). The Uno R4 Minima
# controller uses 2341:0069 — distinct, so we can tell them apart.
MEGA_VID_PID = (0x2341, 0x0042)
DEFAULT_BAUD = 115200

# Parses the '?' response, e.g.: "[seq] auto=ON state=0 idx=0 atHome=yes"
_STATUS_RE = re.compile(
    r"\[seq\]\s+auto=(?P<auto>ON|OFF)\s+state=(?P<state>\d+)"
    r"\s+idx=(?P<idx>\d+)\s+atHome=(?P<home>yes|no)"
)


def find_mega_port() -> str:
    for p in serial.tools.list_ports.comports():
        if (p.vid, p.pid) == MEGA_VID_PID:
            return p.device
    raise RuntimeError(
        f"No Arduino Mega 2560 found by USB VID/PID "
        f"{MEGA_VID_PID[0]:#06x}:{MEGA_VID_PID[1]:#06x}. "
        "Is the arm plugged in to the Pi?"
    )


class ArmLink:
    """Open a USB-serial connection to the arm Mega.

    Use as a context manager so the port closes cleanly. Opening the
    port toggles DTR, which resets the Mega — we sleep 2 s after open
    to let it boot before sending the first command.
    """

    def __init__(self, port: str | None = None, baud: int = DEFAULT_BAUD,
                 timeout: float = 0.2):
        self.port = port or find_mega_port()
        self.baud = baud
        self._timeout = timeout
        self._ser: serial.Serial | None = None

    def __enter__(self) -> "ArmLink":
        self._ser = serial.Serial(self.port, self.baud, timeout=self._timeout)
        time.sleep(2.0)            # wait for Mega boot after DTR-triggered reset
        self._ser.reset_input_buffer()
        return self

    def __exit__(self, *_):
        if self._ser:
            self._ser.close()
            self._ser = None

    # ---- raw I/O -----------------------------------------------------
    def send_raw(self, byte: bytes) -> None:
        """Write one or more bytes to the arm. Public so callers can poke
        commands the wrappers below don't cover yet (e.g. future b<deg>)."""
        if not self._ser:
            raise RuntimeError("ArmLink is not open — use `with ArmLink() as arm:`")
        self._ser.write(byte)
        self._ser.flush()

    def messages(self, timeout: float = 1.0):
        """Yield log lines until `timeout` seconds of silence on the wire.

        Resets the silence timer each time bytes arrive, so a chatty arm
        keeps the generator alive. Useful for following a sequence run.
        """
        if not self._ser:
            raise RuntimeError("ArmLink is not open")
        buf = b""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            chunk = self._ser.read(64)
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    yield line.decode(errors="replace").rstrip("\r")
                deadline = time.monotonic() + timeout
        if buf.strip():
            yield buf.decode(errors="replace").rstrip("\r")

    def send_line(self, text: str) -> None:
        """Write a newline-terminated command line to the arm. The Mega's
        parser dispatches on CR/LF, so every command goes through here."""
        self.send_raw(f"{text}\n".encode())

    # ---- high-level commands ----------------------------------------
    def pickup(self) -> None:
        self.send_line("p")

    def abort(self) -> None:
        self.send_line("x")

    def home(self) -> None:
        self.send_line("h")

    def toggle_auto(self) -> None:
        self.send_line("a")

    def hover(self) -> None:
        """Lift to the approach/hover pose (auto mode only). Base is left
        where it is so a following base() call can aim it over the rock."""
        self.send_line("v")

    def base(self, deg: int) -> None:
        """Set the base angle absolutely (auto mode only) — the visual-servo
        nudge. Clamped to the arm's base limits on the firmware side."""
        self.send_line(f"b{int(deg)}")

    def status(self, wait: float = 0.5) -> dict | None:
        """Send '?' and return parsed state, or None if no response."""
        self.send_line("?")
        for line in self.messages(timeout=wait):
            m = _STATUS_RE.search(line)
            if m:
                return {
                    "auto":    m["auto"] == "ON",
                    "state":   int(m["state"]),
                    "idx":     int(m["idx"]),
                    "at_home": m["home"] == "yes",
                }
        return None

    def set_auto(self, on: bool) -> None:
        """Ensure auto mode is in the requested state, regardless of
        what it was before. Reads status, toggles only if needed."""
        s = self.status()
        if s is None:
            raise RuntimeError("Could not read arm status (no '?' response)")
        if s["auto"] != on:
            self.toggle_auto()


def _repl() -> None:
    """Interactive shell — useful for verifying the link by hand."""
    print("arm_link REPL — commands: p=pickup  x=abort  h=home  a=toggle auto "
          " v=hover  b<deg>=base  ?=status  q=quit")
    with ArmLink() as arm:
        print(f"Connected on {arm.port} @ {arm.baud}")
        while True:
            try:
                cmd = input("> ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not cmd:
                continue
            if cmd == "q":
                break
            if cmd == "?":
                print(arm.status())
                continue
            # Single-letter commands, or b<deg> with an integer argument.
            if cmd in {"p", "x", "h", "a", "v"} or (
                cmd.startswith("b") and cmd[1:].strip().lstrip("-").isdigit()
            ):
                arm.send_line(cmd)
                for line in arm.messages(timeout=0.5):
                    print(f"  {line}")
                continue
            print("unknown command")


if __name__ == "__main__":
    _repl()
