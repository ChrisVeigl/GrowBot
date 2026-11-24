"""Desktop relay tester for the GrowBot continuous-rotation servo port."""

import json
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk
from urllib.parse import quote, urlsplit

import websocket


RELAY = "wss://growbot-relay.growbot.workers.dev"
FRAME_INTERVAL = 0.1
MAX_STEP_MS = 2000
ACK_TIMEOUT = 5.0


def step_pose(direction, speed, invert_left=False, invert_right=False):
    if direction not in ("forward", "back", "left", "right"):
        raise ValueError("unknown direction")
    if not 5 <= speed <= 40:
        raise ValueError("speed must be 5 to 40 degrees from neutral")
    left, right = {
        "forward": (1, 1), "back": (-1, -1),
        "left": (-1, 1), "right": (1, -1),
    }[direction]
    return (90 + left * speed * (-1 if invert_left else 1),
            90 + right * speed * (-1 if invert_right else 1))


def relay_url(base, code):
    parsed = urlsplit(base)
    if parsed.scheme not in ("ws", "wss") or not parsed.netloc or parsed.query or parsed.fragment:
        raise ValueError("relay must be a ws:// or wss:// base URL")
    if not code or len(code) > 128:
        raise ValueError("enter a valid pairing code")
    return base.rstrip("/") + "/d/" + quote(code, safe="")


class RelayWorker(threading.Thread):
    def __init__(self, base, code, events):
        super().__init__(daemon=True)
        self.url = relay_url(base, code)
        self.code = code
        self.events = events
        self.commands = queue.Queue()
        self.awake = False
        self.step = None
        self.next_frame = 0.0
        self.rid = 0
        self.pending = {}

    def emit(self, kind, value):
        self.events.put((kind, value))

    def send(self, message):
        self.ws.send(json.dumps(message, separators=(",", ":")))

    def request(self, kind, **fields):
        self.rid += 1
        message = {"t": kind, "rid": self.rid}
        message.update(fields)
        self.send(message)
        self.pending[self.rid] = (kind, time.monotonic() + ACK_TIMEOUT)
        self.emit("log", "Sent " + json.dumps(message))

    def neutral(self):
        self.step = None
        self.send({"t": "pose", "lr": "90,90"})

    def halt(self):
        self.neutral()
        self.request("stop")

    def command(self, name, args):
        if name == "close":
            return False
        if not self.awake:
            self.emit("log", "Not sent: relay has not reported the Pico awake.")
            return True
        if name == "step":
            left, right, duration_ms = args
            if not (0 <= left <= 180 and 0 <= right <= 180 and 100 <= duration_ms <= MAX_STEP_MS):
                raise ValueError("invalid step")
            if self.step is not None:
                self.neutral()
            now = time.monotonic()
            self.step = (left, right, now + duration_ms / 1000)
            self.next_frame = now
            self.emit("log", "Step: %s,%s for %s ms" % (left, right, duration_ms))
        elif name == "stop":
            self.halt()
        elif name == "mood":
            self.request("state", mood=args)
        elif name == "wifi":
            self.request("wifi_status")
        else:
            raise ValueError("unknown tester command")
        return True

    def tick(self):
        now = time.monotonic()
        if self.step is not None:
            left, right, end = self.step
            if now >= end:
                self.halt()
                self.emit("log", "Step finished: neutral and stop sent.")
            elif now >= self.next_frame:
                self.send({"t": "pose", "lr": "%d,%d" % (left, right)})
                self.next_frame = now + FRAME_INTERVAL
        for rid, (kind, deadline) in list(self.pending.items()):
            if now >= deadline:
                del self.pending[rid]
                self.emit("log", "No response to %s request %s (Pico may not support it)." % (kind, rid))

    def receive(self, text):
        try:
            message = json.loads(text)
        except (ValueError, TypeError):
            self.emit("log", "Invalid relay message: %r" % (text,))
            return
        if not isinstance(message, dict):
            self.emit("log", "Unexpected relay message: %r" % (message,))
            return
        kind = message.get("t")
        if kind == "status" and message.get("awake") in (0, 1):
            self.awake = bool(message["awake"])
            if not self.awake:
                self.step = None
            self.emit("status", "Pico awake" if self.awake else "Pico offline")
            self.emit("log", "Relay: " + json.dumps(message))
        elif kind in ("ack", "wifi_status"):
            self.pending.pop(message.get("rid"), None)
            self.emit("log", "Received " + json.dumps(message))
        elif kind != "alive":
            self.emit("log", "Received " + json.dumps(message))

    def run(self):
        ws = None
        try:
            ws = websocket.create_connection(self.url, timeout=3)
            self.ws = ws
            ws.settimeout(0.05)
            self.send({"t": "attach", "id": self.code, "code": self.code})
            self.emit("status", "Connected; waiting for Pico status")
            running = True
            while running:
                while True:
                    try:
                        name, args = self.commands.get_nowait()
                    except queue.Empty:
                        break
                    if not self.command(name, args):
                        running = False
                        break
                if not running:
                    break
                self.tick()
                try:
                    text = ws.recv()
                except websocket.WebSocketTimeoutException:
                    continue
                if not text:
                    raise ConnectionError("relay closed the connection")
                self.receive(text)
        except (OSError, ValueError, websocket.WebSocketException) as e:
            self.emit("log", "Connection error: %s" % e)
        finally:
            if ws is not None:
                try:
                    if self.awake:
                        self.halt()
                except (OSError, websocket.WebSocketException) as e:
                    self.emit("log", "Could not send final stop: %s" % e)
                try:
                    ws.close()
                except (OSError, websocket.WebSocketException) as e:
                    self.emit("log", "Could not close relay: %s" % e)
            self.awake = False
            self.emit("status", "Disconnected")
            self.emit("closed", None)


class Tester(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("GrowBot servo wheels tester")
        self.events = queue.Queue()
        self.worker = None
        self.code = tk.StringVar()
        self.relay = tk.StringVar(value=RELAY)
        self.status = tk.StringVar(value="Disconnected")
        self.speed = tk.IntVar(value=20)
        self.duration = tk.IntVar(value=500)
        self.invert_left = tk.BooleanVar()
        self.invert_right = tk.BooleanVar()
        self.protocol("WM_DELETE_WINDOW", self.close)

        frame = ttk.Frame(self, padding=10)
        frame.grid(sticky="nsew")
        ttk.Label(frame, text="Pairing code").grid(row=0, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.code, width=20).grid(row=0, column=1, sticky="ew")
        self.connect_button = ttk.Button(frame, text="Connect", command=self.connect)
        self.connect_button.grid(row=0, column=2, padx=4)
        ttk.Button(frame, text="Disconnect", command=self.disconnect).grid(row=0, column=3)
        ttk.Label(frame, text="Relay").grid(row=1, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.relay, width=45).grid(row=1, column=1, columnspan=3, sticky="ew")
        ttk.Label(frame, textvariable=self.status).grid(row=2, column=0, columnspan=4, sticky="w")
        ttk.Label(frame, text="Speed (degrees from 90)").grid(row=3, column=0, columnspan=2, sticky="w")
        tk.Scale(frame, from_=5, to=40, orient="horizontal", variable=self.speed).grid(row=3, column=2, columnspan=2, sticky="ew")
        ttk.Label(frame, text="Step duration (ms, 100–2000)").grid(row=4, column=0, columnspan=2, sticky="w")
        ttk.Spinbox(frame, from_=100, to=2000, increment=100, textvariable=self.duration, width=8).grid(row=4, column=2, sticky="w")
        ttk.Checkbutton(frame, text="Invert left", variable=self.invert_left).grid(row=5, column=0, sticky="w")
        ttk.Checkbutton(frame, text="Invert right", variable=self.invert_right).grid(row=5, column=1, sticky="w")
        for text, direction, row, col in (
                ("Forward", "forward", 6, 1), ("Left", "left", 7, 0),
                ("Right", "right", 7, 2), ("Back", "back", 8, 1)):
            ttk.Button(frame, text=text, command=lambda d=direction: self.move(d)).grid(row=row, column=col, sticky="ew")
        ttk.Button(frame, text="STOP", command=lambda: self.send("stop")).grid(row=7, column=1, sticky="ew")
        ttk.Label(frame, text="Custom mood LED:").grid(row=9, column=0, sticky="w")
        for col, mood in enumerate(("happy", "sad", "neutral", "off")):
            ttk.Button(frame, text=mood.title(), command=lambda m=mood: self.send("mood", m)).grid(row=10, column=col, sticky="ew")
        ttk.Button(frame, text="Request Wi-Fi status", command=lambda: self.send("wifi")).grid(row=11, column=0, columnspan=2, sticky="w")
        self.log = tk.Text(frame, width=74, height=12, state="disabled", wrap="word")
        self.log.grid(row=12, column=0, columnspan=4, sticky="nsew", pady=6)
        ttk.Label(frame, text="Test with wheels raised. STOP and neutral are not physical brakes.").grid(row=13, column=0, columnspan=4, sticky="w")
        self.after(50, self.read_events)

    def append(self, text):
        self.log.config(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.config(state="disabled")

    def connect(self):
        if self.worker is not None:
            self.append("Already connected or connecting.")
            return
        try:
            self.worker = RelayWorker(self.relay.get().strip(), self.code.get().strip(), self.events)
        except ValueError as e:
            self.append(str(e))
            return
        self.status.set("Connecting")
        self.worker.start()

    def send(self, command, args=None):
        if self.worker is None:
            self.append("Connect first.")
        else:
            self.worker.commands.put((command, args))

    def move(self, direction):
        try:
            left, right = step_pose(direction, self.speed.get(), self.invert_left.get(), self.invert_right.get())
            duration = self.duration.get()
            if not 100 <= duration <= MAX_STEP_MS:
                raise ValueError("duration must be 100–2000 ms")
        except (ValueError, tk.TclError) as e:
            self.append("Invalid step: %s" % e)
            return
        self.send("step", (left, right, duration))

    def disconnect(self):
        self.send("close")

    def read_events(self):
        while True:
            try:
                kind, value = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == "log":
                self.append(value)
            elif kind == "status":
                self.status.set(value)
            elif kind == "closed":
                self.worker = None
        self.after(50, self.read_events)

    def close(self):
        if self.worker is not None:
            self.worker.commands.put(("close", None))
            self.worker.join(timeout=1)
        self.destroy()


if __name__ == "__main__":
    Tester().mainloop()
