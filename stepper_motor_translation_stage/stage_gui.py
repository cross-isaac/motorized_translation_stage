#!/usr/bin/env python3
"""
Translation-stage controller GUI.

Protocol (firmware is the source of truth for position):
    SET  <inches>   tell the firmware where the stage physically is (no motion)
    GOTO <inches>   move to an absolute position
    POS             ask for the current position
    ESC byte (0x1B) emergency stop during a move

Requires:  py -m pip install pyserial   (tkinter ships with CPython).
Run:       py stage_gui.py
"""

import queue
import threading
import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports

BAUD = 115200
MIN_IN, MAX_IN = 0.0, 1.0
STOP_BYTE = b"\x1b"   # ESC


class StageGUI:
    def __init__(self, root):
        self.root = root
        self.ser = None
        self.stop_flag = threading.Event()
        self.rx = queue.Queue()

        root.title("Translation Stage Control")
        root.geometry("460x540")

        # --- connection ---
        conn = ttk.LabelFrame(root, text="Connection")
        conn.pack(fill="x", padx=10, pady=8)
        self.port_var = tk.StringVar()
        self.port_menu = ttk.Combobox(conn, textvariable=self.port_var,
                                      width=22, state="readonly")
        self.port_menu.grid(row=0, column=0, padx=6, pady=6)
        ttk.Button(conn, text="Refresh", command=self.refresh_ports).grid(row=0, column=1, padx=4)
        self.connect_btn = ttk.Button(conn, text="Connect", command=self.toggle_connect)
        self.connect_btn.grid(row=0, column=2, padx=4)

        # --- current position readout ---
        pos = ttk.LabelFrame(root, text="Current position")
        pos.pack(fill="x", padx=10, pady=8)
        self.pos_var = tk.StringVar(value="--")
        tk.Label(pos, textvariable=self.pos_var, font=("Helvetica", 28)).pack(pady=8)

        # --- emergency stop (also bound to the Escape key) ---
        self.stop_btn = tk.Button(root, text="STOP  (Esc)", command=self.send_stop,
                                  bg="#c0392b", fg="white",
                                  font=("Helvetica", 16, "bold"), height=2)
        self.stop_btn.pack(fill="x", padx=10, pady=4)
        self.root.bind("<Escape>", self.send_stop)

        # --- set current position (manual datum) ---
        setf = ttk.LabelFrame(root, text="Set current position (no motion)")
        setf.pack(fill="x", padx=10, pady=8)
        self.set_entry = ttk.Entry(setf, width=12)
        self.set_entry.grid(row=0, column=0, padx=6, pady=6)
        ttk.Label(setf, text="in").grid(row=0, column=1)
        ttk.Button(setf, text="Set", command=self.send_set).grid(row=0, column=2, padx=6)

        # --- go to position ---
        gof = ttk.LabelFrame(root, text="Go to position")
        gof.pack(fill="x", padx=10, pady=8)
        self.go_entry = ttk.Entry(gof, width=12)
        self.go_entry.grid(row=0, column=0, padx=6, pady=6)
        ttk.Label(gof, text="in").grid(row=0, column=1)
        ttk.Button(gof, text="Move", command=self.send_goto).grid(row=0, column=2, padx=6)

        # --- log ---
        logf = ttk.LabelFrame(root, text="Log")
        logf.pack(fill="both", expand=True, padx=10, pady=8)
        self.log = tk.Text(logf, height=7, state="disabled", wrap="word")
        self.log.pack(fill="both", expand=True, padx=4, pady=4)

        self.refresh_ports()
        self.root.after(50, self.poll_rx)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- serial plumbing ----------
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_menu["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def toggle_connect(self):
        self.disconnect() if self.ser else self.connect()

    def connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("No port", "Pick a serial port first.")
            return
        try:
            self.ser = serial.Serial(port, BAUD, timeout=0.2)
        except serial.SerialException as e:
            messagebox.showerror("Connection failed", str(e))
            self.ser = None
            return
        self.stop_flag.clear()
        threading.Thread(target=self.read_loop, daemon=True).start()
        self.connect_btn.config(text="Disconnect")
        self.logmsg(f"[connected {port}]")
        # ESP32 resets when the port opens, so ask where it thinks it is.
        self.root.after(1500, lambda: self.send("POS"))

    def disconnect(self):
        self.stop_flag.set()
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connect_btn.config(text="Connect")
        self.pos_var.set("--")
        self.logmsg("[disconnected]")

    def read_loop(self):
        while not self.stop_flag.is_set() and self.ser:
            try:
                line = self.ser.readline().decode(errors="replace").strip()
            except Exception:
                break
            if line:
                self.rx.put(line)

    def send(self, cmd):
        if not self.ser:
            messagebox.showwarning("Not connected", "Connect to the ESP32 first.")
            return
        try:
            self.ser.write((cmd + "\n").encode())
            self.logmsg("> " + cmd)
        except Exception as e:
            messagebox.showerror("Write failed", str(e))

    def send_stop(self, event=None):
        """Emergency stop: writes a raw ESC byte, caught mid-move by the firmware."""
        if not self.ser:
            return
        try:
            self.ser.write(STOP_BYTE)
            self.logmsg("[STOP sent]")
        except Exception as e:
            messagebox.showerror("Write failed", str(e))

    # ---------- command helpers ----------
    def _read_field(self, entry):
        try:
            v = float(entry.get())
        except ValueError:
            messagebox.showwarning("Bad value", "Enter a number in inches.")
            return None
        if not (MIN_IN <= v <= MAX_IN):
            messagebox.showwarning("Out of range", "Value must be between 0 and 1 inch.")
            return None
        return v

    def send_set(self):
        v = self._read_field(self.set_entry)
        if v is not None:
            self.send(f"SET {v:.5f}")
            self.root.after(150, lambda: self.send("POS"))  # refresh readout after SET

    def send_goto(self):
        v = self._read_field(self.go_entry)
        if v is not None:
            self.send(f"GOTO {v:.5f}")   # firmware streams POS during the move

    # ---------- UI updates ----------
    def poll_rx(self):
        while not self.rx.empty():
            line = self.rx.get()
            self.logmsg(line)
            # Update the big readout from lines like: "POS 0.30000 in  (2400 steps)"
            if line.startswith("POS "):
                parts = line.split()
                try:
                    self.pos_var.set(f"{float(parts[1]):.4f} in")
                except (IndexError, ValueError):
                    pass  # e.g. "POS unknown ..." -- leave readout as-is
        self.root.after(50, self.poll_rx)

    def logmsg(self, text):
        self.log.config(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.config(state="disabled")

    def on_close(self):
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    StageGUI(root)
    root.mainloop()