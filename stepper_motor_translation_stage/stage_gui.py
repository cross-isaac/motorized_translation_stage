#!/usr/bin/env python3
"""
Translation-stage controller GUI.

Protocol (firmware is the source of truth for position):
    SET  <inches>   tell the firmware where the stage physically is (no motion)
    GOTO <inches>   move to an absolute position
    POS             ask for the current position
    ESC byte (0x1B) emergency stop during a move

This GUI can also run a "program": an ordered list of (position, dwell) steps
that it executes automatically -- e.g. go to 0.5 in and hold 1 min, then 0.4 in
and hold 2 min, and so on, optionally looping. The sequencing lives here in
Python; the firmware only ever sees plain SET/GOTO/POS commands, so no firmware
changes or re-flashing are needed.

Requires:  py -m pip install pyserial   (tkinter ships with CPython).
Run:       py stage_gui.py
"""

import csv
import json
import queue
import threading
import time
from datetime import datetime
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

import serial
import serial.tools.list_ports

BAUD = 115200
MIN_IN, MAX_IN = 0.0, 1.0
STOP_BYTE = b"\x1b"   # ESC

# Dwell time units -> seconds.
UNITS = {"sec": 1, "min": 60, "hr": 3600}

ACTIVE_BG = "#fff3b0"   # highlight color for the running step's row
IDLE_BG   = "white"


class StageGUI:
    def __init__(self, root):
        self.root = root
        self.ser = None
        self.stop_flag = threading.Event()
        self.rx = queue.Queue()

        # --- program-runner state ---
        self.step_rows = []        # list of row dicts (the editable table)
        self.prog_running = False
        self.prog_steps = []       # [{'pos': float, 'dwell_s': float, 'row': dict}, ...]
        self.prog_index = 0
        self.prog_state = None     # "move" | "dwell"
        self.awaiting_move = False
        self.move_result = None    # "ok" | "stopped" | "err:<msg>"
        self.dwell_deadline = 0.0
        self.prog_loop = False
        self.prog_repeat = 1
        self.loops_done = 0

        # --- CSV logging state ---
        self.last_pos_in = None    # most recent position parsed from a POS line
        self.csv_file = None
        self.csv_writer = None
        self.run_start = 0.0       # time.monotonic() at program start
        self.last_sample = 0.0     # last CSV sample time (for 1 Hz dwell sampling)

        root.title("Translation Stage Control")
        root.geometry("560x860")
        root.minsize(520, 700)

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

        # --- manual control (set datum + jog to a position) ---
        man = ttk.LabelFrame(root, text="Manual control")
        man.pack(fill="x", padx=10, pady=8)
        ttk.Label(man, text="Set current position (no motion):").grid(
            row=0, column=0, columnspan=3, sticky="w", padx=6, pady=(6, 0))
        self.set_entry = ttk.Entry(man, width=10)
        self.set_entry.grid(row=1, column=0, padx=6, pady=4)
        ttk.Label(man, text="in").grid(row=1, column=1)
        ttk.Button(man, text="Set", command=self.send_set).grid(row=1, column=2, padx=6)

        ttk.Label(man, text="Go to position:").grid(
            row=2, column=0, columnspan=3, sticky="w", padx=6, pady=(6, 0))
        self.go_entry = ttk.Entry(man, width=10)
        self.go_entry.grid(row=3, column=0, padx=6, pady=4)
        ttk.Label(man, text="in").grid(row=3, column=1)
        ttk.Button(man, text="Move", command=self.send_goto).grid(row=3, column=2, padx=6)

        # jog: nudge the stage by a fixed step relative to where it is now
        ttk.Label(man, text="Jog (relative move):").grid(
            row=4, column=0, columnspan=3, sticky="w", padx=6, pady=(6, 0))
        jog = ttk.Frame(man)
        jog.grid(row=5, column=0, columnspan=4, sticky="w", padx=6, pady=4)
        ttk.Button(jog, text="−", width=4,
                   command=lambda: self.send_jog(-1)).grid(row=0, column=0)
        self.jog_entry = ttk.Entry(jog, width=8, justify="center")
        self.jog_entry.insert(0, "0.01")
        self.jog_entry.grid(row=0, column=1, padx=4)
        ttk.Label(jog, text="in").grid(row=0, column=2)
        ttk.Button(jog, text="+", width=4,
                   command=lambda: self.send_jog(+1)).grid(row=0, column=3, padx=(4, 0))

        # --- program editor ---
        self.build_program_section(root)

        # --- log ---
        logf = ttk.LabelFrame(root, text="Log")
        logf.pack(fill="both", expand=True, padx=10, pady=8)
        self.log = tk.Text(logf, height=6, state="disabled", wrap="word")
        self.log.pack(fill="both", expand=True, padx=4, pady=4)

        self.refresh_ports()
        self.root.after(50, self.poll_rx)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # =====================================================================
    #  Program section (editable step table + runner controls)
    # =====================================================================
    def build_program_section(self, root):
        prog = ttk.LabelFrame(root, text="Program (position + hold time)")
        prog.pack(fill="both", expand=False, padx=10, pady=8)

        # column headers
        hdr = ttk.Frame(prog)
        hdr.pack(fill="x", padx=6, pady=(6, 0))
        ttk.Label(hdr, text="#",        width=3).grid(row=0, column=0)
        ttk.Label(hdr, text="Position", width=10).grid(row=0, column=1)
        ttk.Label(hdr, text="in",       width=3).grid(row=0, column=2)
        ttk.Label(hdr, text="Hold",     width=8).grid(row=0, column=3)
        ttk.Label(hdr, text="Unit",     width=6).grid(row=0, column=4)

        # scrollable list of step rows
        body = ttk.Frame(prog)
        body.pack(fill="x", padx=6)
        self.steps_canvas = tk.Canvas(body, height=170, highlightthickness=0)
        vsb = ttk.Scrollbar(body, orient="vertical", command=self.steps_canvas.yview)
        self.steps_frame = tk.Frame(self.steps_canvas)
        self.steps_frame.bind(
            "<Configure>",
            lambda e: self.steps_canvas.configure(scrollregion=self.steps_canvas.bbox("all")))
        self.steps_canvas.create_window((0, 0), window=self.steps_frame, anchor="nw")
        self.steps_canvas.configure(yscrollcommand=vsb.set)
        self.steps_canvas.pack(side="left", fill="x", expand=True)
        vsb.pack(side="right", fill="y")

        # add-step + file buttons
        btns = ttk.Frame(prog)
        btns.pack(fill="x", padx=6, pady=6)
        self.add_btn = ttk.Button(btns, text="+ Add step", command=self.add_step)
        self.add_btn.grid(row=0, column=0, padx=2)
        self.load_btn = ttk.Button(btns, text="Load...", command=self.load_program)
        self.load_btn.grid(row=0, column=1, padx=2)
        self.save_btn = ttk.Button(btns, text="Save...", command=self.save_program)
        self.save_btn.grid(row=0, column=2, padx=2)

        # loop controls
        loopf = ttk.Frame(prog)
        loopf.pack(fill="x", padx=6, pady=(0, 6))
        self.loop_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(loopf, text="Loop", variable=self.loop_var).grid(row=0, column=0)
        ttk.Label(loopf, text="repeat").grid(row=0, column=1, padx=(10, 2))
        self.repeat_entry = ttk.Entry(loopf, width=6)
        self.repeat_entry.insert(0, "1")
        self.repeat_entry.grid(row=0, column=2)
        ttk.Label(loopf, text="times  (0 = forever)").grid(row=0, column=3, padx=4)
        self.csv_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(loopf, text="Log position to CSV",
                        variable=self.csv_var).grid(row=1, column=0, columnspan=4,
                                                    sticky="w", pady=(4, 0))

        # run / stop
        runf = ttk.Frame(prog)
        runf.pack(fill="x", padx=6, pady=(0, 6))
        self.run_btn = ttk.Button(runf, text="Run Program", command=self.run_program)
        self.run_btn.grid(row=0, column=0, padx=2)
        self.stop_prog_btn = ttk.Button(runf, text="Stop Program",
                                        command=self.stop_program, state="disabled")
        self.stop_prog_btn.grid(row=0, column=1, padx=2)

        # status line
        self.prog_status = tk.StringVar(value="Idle.")
        ttk.Label(prog, textvariable=self.prog_status,
                  font=("Helvetica", 10, "italic")).pack(fill="x", padx=8, pady=(0, 6))

        # start with a couple of example rows to make the format obvious
        self.add_step(0.5, "1", "min")
        self.add_step(0.4, "2", "min")

    def add_step(self, pos="", dwell="", unit="sec"):
        row = {}
        f = self.steps_frame
        r = len(self.step_rows)

        idx_lbl = tk.Label(f, text=str(r + 1), width=3, bg=IDLE_BG)
        idx_lbl.grid(row=r, column=0, pady=1)

        pos_var = tk.StringVar(value=("" if pos == "" else f"{float(pos):g}"))
        pos_e = tk.Entry(f, width=10, textvariable=pos_var, justify="right")
        pos_e.grid(row=r, column=1, padx=2, pady=1)

        tk.Label(f, text="in", width=3, bg=IDLE_BG).grid(row=r, column=2)

        dwell_var = tk.StringVar(value=str(dwell))
        dwell_e = tk.Entry(f, width=8, textvariable=dwell_var, justify="right")
        dwell_e.grid(row=r, column=3, padx=2, pady=1)

        unit_var = tk.StringVar(value=unit)
        unit_cb = ttk.Combobox(f, textvariable=unit_var, values=list(UNITS),
                               width=5, state="readonly")
        unit_cb.grid(row=r, column=4, padx=2, pady=1)

        rm_btn = tk.Button(f, text="✕", width=2,
                           command=lambda: self.remove_step(row))
        rm_btn.grid(row=r, column=5, padx=2)

        row.update(dict(idx=idx_lbl, pos=pos_var, dwell=dwell_var, unit=unit_var,
                        pos_entry=pos_e, dwell_entry=dwell_e, rm=rm_btn))
        self.step_rows.append(row)

    def remove_step(self, row):
        if self.prog_running:
            return
        self.step_rows.remove(row)
        self.regrid_steps()  # rebuilds the whole grid, so numbering stays contiguous

    def regrid_steps(self):
        """Rebuild the grid so row numbers stay contiguous after a removal."""
        # Easiest robust approach: destroy and recreate from captured values.
        vals = [(r["pos"].get(), r["dwell"].get(), r["unit"].get()) for r in self.step_rows]
        for child in self.steps_frame.winfo_children():
            child.destroy()
        self.step_rows = []
        for pos, dwell, unit in vals:
            self.add_step(pos, dwell, unit)

    def collect_steps(self):
        """Validate the table and return a list of {'pos','dwell_s','row'} or None."""
        steps = []
        for r in self.step_rows:
            pos_s = r["pos"].get().strip()
            dwell_s = r["dwell"].get().strip()
            if pos_s == "" and dwell_s == "":
                continue  # skip blank rows
            try:
                pos = float(pos_s)
            except ValueError:
                messagebox.showwarning("Bad program", "Every step needs a numeric position.")
                return None
            if not (MIN_IN <= pos <= MAX_IN):
                messagebox.showwarning("Out of range",
                                       f"Position {pos} is outside 0..1 inch.")
                return None
            try:
                dwell = float(dwell_s)
            except ValueError:
                messagebox.showwarning("Bad program", "Every step needs a numeric hold time.")
                return None
            if dwell < 0:
                messagebox.showwarning("Bad program", "Hold time cannot be negative.")
                return None
            steps.append({"pos": pos, "dwell_s": dwell * UNITS[r["unit"].get()], "row": r})
        if not steps:
            messagebox.showwarning("Empty program", "Add at least one step first.")
            return None
        return steps

    # ---------- save / load ----------
    def save_program(self):
        steps = [{"pos": r["pos"].get(), "dwell": r["dwell"].get(), "unit": r["unit"].get()}
                 for r in self.step_rows]
        data = {"loop": self.loop_var.get(),
                "repeat": self.repeat_entry.get(),
                "steps": steps}
        path = filedialog.asksaveasfilename(defaultextension=".json",
                                            filetypes=[("Program", "*.json"), ("All", "*.*")])
        if not path:
            return
        try:
            with open(path, "w") as fh:
                json.dump(data, fh, indent=2)
            self.logmsg(f"[saved program to {path}]")
        except OSError as e:
            messagebox.showerror("Save failed", str(e))

    def load_program(self):
        if self.prog_running:
            return
        path = filedialog.askopenfilename(
            filetypes=[("Program", "*.json"), ("All", "*.*")])
        if not path:
            return
        try:
            with open(path) as fh:
                data = json.load(fh)
        except (OSError, json.JSONDecodeError) as e:
            messagebox.showerror("Load failed", str(e))
            return
        for child in self.steps_frame.winfo_children():
            child.destroy()
        self.step_rows = []
        for s in data.get("steps", []):
            self.add_step(s.get("pos", ""), s.get("dwell", ""), s.get("unit", "sec"))
        self.loop_var.set(bool(data.get("loop", False)))
        self.repeat_entry.delete(0, "end")
        self.repeat_entry.insert(0, str(data.get("repeat", "1")))
        self.logmsg(f"[loaded program from {path}]")

    # =====================================================================
    #  Program runner (non-blocking state machine driven by prog_tick)
    # =====================================================================
    def run_program(self):
        if self.prog_running:
            return
        if not self.ser:
            messagebox.showwarning("Not connected", "Connect to the ESP32 first.")
            return
        steps = self.collect_steps()
        if steps is None:
            return
        # repeat / loop settings
        self.prog_loop = self.loop_var.get()
        try:
            self.prog_repeat = int(float(self.repeat_entry.get()))
        except ValueError:
            self.prog_repeat = 1
        if self.prog_repeat < 0:
            self.prog_repeat = 0

        self.prog_steps = steps
        self.prog_index = 0
        self.loops_done = 0
        self.run_start = time.monotonic()
        self.last_sample = 0.0
        self._open_csv()
        self.prog_running = True
        self.set_edit_enabled(False)
        self.run_btn.config(state="disabled")
        self.stop_prog_btn.config(state="normal")
        self.logmsg("[program started]")
        self._start_step()
        self.root.after(100, self.prog_tick)

    def _start_step(self):
        self._highlight(self.prog_index)
        step = self.prog_steps[self.prog_index]
        n = len(self.prog_steps)
        pass_txt = ""
        if self.prog_loop:
            total = "∞" if self.prog_repeat == 0 else self.prog_repeat
            pass_txt = f"  [pass {self.loops_done + 1}/{total}]"
        self.prog_status.set(
            f"Step {self.prog_index + 1}/{n}: moving to {step['pos']:.4f} in...{pass_txt}")
        self.move_result = None
        self.awaiting_move = True
        self.prog_state = "move"
        self.send(f"GOTO {step['pos']:.5f}")
        self._csv_log("move_start")

    def _begin_dwell(self):
        step = self.prog_steps[self.prog_index]
        self.dwell_deadline = time.monotonic() + step["dwell_s"]
        self.prog_state = "dwell"
        self._csv_log("dwell_start")

    def _advance(self):
        self.prog_index += 1
        if self.prog_index >= len(self.prog_steps):
            self.loops_done += 1
            keep_going = self.prog_loop and (
                self.prog_repeat == 0 or self.loops_done < self.prog_repeat)
            if keep_going:
                self.prog_index = 0
                self._start_step()
            else:
                self._finish_program("Program complete.")
        else:
            self._start_step()

    def prog_tick(self):
        if not self.prog_running:
            return
        # Sample position ~once per second so long holds get regular CSV rows.
        now = time.monotonic()
        if self.csv_writer and now - self.last_sample >= 1.0:
            self.last_sample = now
            self._csv_log(self.prog_state or "")
        if self.prog_state == "move":
            if not self.awaiting_move:
                if self.move_result == "ok":
                    self._begin_dwell()
                elif self.move_result == "stopped":
                    self._finish_program("Program stopped (emergency stop during move).")
                    return
                else:
                    self._finish_program(f"Program aborted -- {self.move_result}")
                    return
        elif self.prog_state == "dwell":
            remaining = self.dwell_deadline - time.monotonic()
            if remaining <= 0:
                self._advance()
            else:
                step = self.prog_steps[self.prog_index]
                n = len(self.prog_steps)
                self.prog_status.set(
                    f"Step {self.prog_index + 1}/{n}: holding at {step['pos']:.4f} in"
                    f"  --  {self._fmt_time(remaining)} left")
        self.root.after(100, self.prog_tick)

    def stop_program(self):
        if not self.prog_running:
            return
        if self.awaiting_move:
            self.send_stop()  # motor is moving -- fire ESC too
        self._finish_program("Program stopped by user.")

    def _finish_program(self, msg):
        self._csv_log("end")
        self._close_csv()
        self.prog_running = False
        self.prog_state = None
        self.awaiting_move = False
        self._highlight(None)
        self.set_edit_enabled(True)
        self.run_btn.config(state="normal")
        self.stop_prog_btn.config(state="disabled")
        self.prog_status.set(msg)
        self.logmsg(f"[{msg}]")

    # ---------- CSV position logging ----------
    def _open_csv(self):
        self.csv_file = None
        self.csv_writer = None
        if not self.csv_var.get():
            return
        default = datetime.now().strftime("stage_log_%Y%m%d_%H%M%S.csv")
        path = filedialog.asksaveasfilename(
            title="Save position log", defaultextension=".csv", initialfile=default,
            filetypes=[("CSV", "*.csv"), ("All", "*.*")])
        if not path:
            self.logmsg("[no file chosen -- running without a CSV log]")
            return
        try:
            self.csv_file = open(path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                ["timestamp", "elapsed_s", "position_in", "target_in", "step", "phase"])
            self.csv_file.flush()
            self.logmsg(f"[logging position to {path}]")
        except OSError as e:
            messagebox.showerror("Log failed", str(e))
            self.csv_file = None
            self.csv_writer = None

    def _csv_log(self, phase):
        if not self.csv_writer:
            return
        ts = datetime.now().isoformat(sep=" ", timespec="milliseconds")  # absolute date/time
        elapsed = time.monotonic() - self.run_start
        pos = "" if self.last_pos_in is None else f"{self.last_pos_in:.5f}"
        if 0 <= self.prog_index < len(self.prog_steps):
            target = f"{self.prog_steps[self.prog_index]['pos']:.5f}"
            step = self.prog_index + 1
        else:
            target, step = "", ""
        try:
            self.csv_writer.writerow([ts, f"{elapsed:.3f}", pos, target, step, phase])
            self.csv_file.flush()
        except (OSError, ValueError):
            pass  # file closed underneath us; ignore

    def _close_csv(self):
        if self.csv_file:
            try:
                self.csv_file.close()
            except OSError:
                pass
        self.csv_file = None
        self.csv_writer = None

    def _highlight(self, active_index):
        for i, r in enumerate(self.step_rows):
            bg = ACTIVE_BG if (active_index is not None and i == active_index) else IDLE_BG
            r["idx"].config(bg=bg)
            r["pos_entry"].config(bg=bg)
            r["dwell_entry"].config(bg=bg)

    def set_edit_enabled(self, enabled):
        state = "normal" if enabled else "disabled"
        ro = "readonly" if enabled else "disabled"
        for r in self.step_rows:
            r["pos_entry"].config(state=state)
            r["dwell_entry"].config(state=state)
            r["rm"].config(state=state)
        for b in (self.add_btn, self.load_btn, self.save_btn):
            b.config(state=state)
        self.repeat_entry.config(state=state)

    @staticmethod
    def _fmt_time(secs):
        secs = int(round(secs))
        h, rem = divmod(secs, 3600)
        m, s = divmod(rem, 60)
        if h:
            return f"{h}:{m:02d}:{s:02d}"
        return f"{m}:{s:02d}"

    # =====================================================================
    #  Serial plumbing
    # =====================================================================
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
        if self.prog_running:
            self._finish_program("Program stopped (disconnected).")
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
        if self.prog_running:
            # abort the program too, but don't recurse into another ESC
            self._csv_log("stop")
            self._close_csv()
            self.prog_running = False
            self.prog_state = None
            self.awaiting_move = False
            self._highlight(None)
            self.set_edit_enabled(True)
            self.run_btn.config(state="normal")
            self.stop_prog_btn.config(state="disabled")
            self.prog_status.set("Program stopped (emergency stop).")
        if not self.ser:
            return
        try:
            self.ser.write(STOP_BYTE)
            self.logmsg("[STOP sent]")
        except Exception as e:
            messagebox.showerror("Write failed", str(e))

    # ---------- manual command helpers ----------
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
        if self.prog_running:
            messagebox.showinfo("Program running", "Stop the program before jogging manually.")
            return
        v = self._read_field(self.go_entry)
        if v is not None:
            self.send(f"GOTO {v:.5f}")   # firmware streams POS during the move

    def send_jog(self, direction):
        """Move by +/- the step size, relative to the current position."""
        if self.prog_running:
            messagebox.showinfo("Program running", "Stop the program before jogging manually.")
            return
        if self.last_pos_in is None:
            messagebox.showwarning("Position unknown",
                                   "Set the current position first, then jog.")
            return
        try:
            step = float(self.jog_entry.get())
        except ValueError:
            messagebox.showwarning("Bad step", "Enter a numeric jog step in inches.")
            return
        target = self.last_pos_in + direction * abs(step)
        if not (MIN_IN <= target <= MAX_IN):
            messagebox.showwarning("Out of range",
                                   f"Jog would reach {target:.4f} in (limits are 0..1).")
            return
        self.send(f"GOTO {target:.5f}")   # firmware streams POS during the move

    # =====================================================================
    #  UI updates
    # =====================================================================
    def poll_rx(self):
        while not self.rx.empty():
            line = self.rx.get()
            self.logmsg(line)
            # Update the big readout from lines like: "POS 0.30000 in  (2400 steps)"
            if line.startswith("POS "):
                parts = line.split()
                try:
                    self.last_pos_in = float(parts[1])
                    self.pos_var.set(f"{self.last_pos_in:.4f} in")
                    if self.prog_running:
                        self._csv_log(self.prog_state or "move")
                except (IndexError, ValueError):
                    pass  # e.g. "POS unknown ..." -- leave readout as-is
            # Let the program runner know when a move has finished.
            if self.awaiting_move:
                if line.startswith("OK moved") or line.startswith("OK already there"):
                    self.move_result = "ok"
                    self.awaiting_move = False
                elif line.startswith("STOPPED"):
                    self.move_result = "stopped"
                    self.awaiting_move = False
                elif line.startswith("ERR"):
                    self.move_result = f"err: {line}"
                    self.awaiting_move = False
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
