import threading
import time


# -------------------fake controller at the serial boundary-----------------
class FakeController:
    def __init__(self, fork_firmware=True, homed=True):
        self.fork_firmware = fork_firmware
        self.homed = homed
        self.lock = threading.RLock()
        self.commands = []
        self.modal_tools = [0.0, 0.0]
        self.physical_tools = [0.0, 0.0]
        self.queue = []
        self.active = None
        self.position = (0.0, 0.0, 0.0)
        self.reply_delays = {}
        self.drop_reply_prefixes = set()
        self.reject_prefixes = {}
        self.segments_per_poll = 1
        self.on_command = None

    def version(self):
        return "v1.0.5-sd.1" if self.fork_firmware else "v1.0.5"

    def advance(self, n=1):
        with self.lock:
            for _ in range(n):
                if self.queue:
                    self.active = self.queue.pop(0)
                    self.physical_tools = list(self.active["tools"])
                    self.position = self.active["end"]
                else:
                    self.active = None
                    return

    def idle(self):
        with self.lock:
            if self.fork_firmware:
                return not self.queue and self.active is None
            return not self.queue

    def handle(self, line):
        with self.lock:
            self.commands.append(line)
            if self.on_command:
                self.on_command(self, line)
            for prefix, message in self.reject_prefixes.items():
                if line.startswith(prefix):
                    return f"error: {message}\n"
            parts = line.split()
            cmd = parts[0]
            if cmd == "ECHO":
                return f"data:{' '.join(parts[1:])}\nok\n"
            words = {p[0]: float(p[1:]) for p in parts[1:] if len(p) > 1}
            if cmd == "M58":
                return self.version() + "\nok\n"
            if cmd in ("G0", "G1"):
                if not self.homed:
                    return "error: not all joints calibrated and homed\n"
                end = (words.get("X", self.position[0]), words.get("Y", self.position[1]),
                       words.get("Z", self.position[2]))
                self.queue.append({"kind": "move", "end": end, "tools": list(self.modal_tools), "cmd": line})
                return "ok\n"
            if cmd == "G4":
                if not self.homed:
                    return "error: not all joints calibrated and homed\n"
                end = self.queue[-1]["end"] if self.queue else self.position
                self.queue.append({"kind": "dwell", "end": end, "tools": list(self.modal_tools), "cmd": line})
                return "ok\n"
            if cmd == "M3":
                tool = int(words.get("T", 0))
                value = words.get("S", 0.0)
                if not 0.0 <= value <= 1.0:
                    return "error: tool output must be within [0, 1]\n"
                self.modal_tools[tool] = value
                return "ok\n"
            if cmd == "M53":
                result = "1" if self.idle() else "0"
                self.advance(self.segments_per_poll)
                return f"{result}\nok\n"
            if cmd == "M50":
                x, y, z = self.position
                return f"X{x:.6f} Y{y:.6f} Z{z:.6f}\nok\n"
            if cmd in ("M204", "G24", "M17", "M18"):
                return "ok\n"
            if cmd == "ECHO":
                return f"data:{' '.join(parts[1:])}\nok\n"
            return "error: unknown command\n"


class FakeSerial:
    def __init__(self, controller, port=None, baud=None, timeout=None):
        self.controller = controller
        self.is_open = True
        self._out = bytearray()
        self._lock = threading.Lock()
        self._pending = ""
        self._release_at = 0.0
        self._outbox = []
        self._outbox_cv = threading.Condition()
        self._sender = threading.Thread(target=self._send_loop, daemon=True)
        self._sender.start()

    def _send_loop(self):
        while True:
            with self._outbox_cv:
                while not self._outbox and self.is_open:
                    self._outbox_cv.wait()
                if not self._outbox:
                    return
                release, text = self._outbox[0]
            remaining = release - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            with self._outbox_cv:
                self._outbox.pop(0)
            self._emit(text)

    @property
    def in_waiting(self):
        with self._lock:
            return len(self._out)

    def read(self, n):
        with self._lock:
            data = bytes(self._out[:n])
            del self._out[:n]
            return data

    def _emit(self, text):
        with self._lock:
            self._out.extend(text.encode("ascii"))

    def write(self, data):
        self._pending += data.decode("ascii")
        while "\n" in self._pending:
            line, self._pending = self._pending.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            reply = self.controller.handle(line)
            if any(line.startswith(p) for p in self.controller.drop_reply_prefixes):
                continue
            delay = next((d for p, d in self.controller.reply_delays.items() if line.startswith(p)), 0.0)
            with self._outbox_cv:
                self._release_at = max(self._release_at, time.monotonic() + delay)
                self._outbox.append((self._release_at, reply))
                self._outbox_cv.notify()
        return len(data)

    def flush(self):
        pass

    def close(self):
        with self._outbox_cv:
            self.is_open = False
            self._outbox_cv.notify()


def connect_fake(oms, controller):
    ok = oms.connect("FAKE", serial_factory=lambda port, baud, timeout=None: FakeSerial(controller))
    assert ok
    return oms


def wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return predicate()
