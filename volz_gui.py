#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VOLZ RS-485 (Duplex) Helper GUI
Features:
- Open/Close serial port (COM selection + baud)
- Scan IDs (safe scan 0x01..0x1E or broadcast 0x1F when only one device on the bus)
- Set (change) ID (0xAA -> 0x55)
- Send Position (0x76 -> 0x56) single-shot
- Force ACE1/ACE2 Master (0x35/0x33)
- Hex log (TX/RX), minimal half-duplex support (manual RTS optional)
Dependencies: pyserial
  pip install pyserial
"""

import time
import sys
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText

import serial
from serial.tools import list_ports

# ---------------- Protocol constants ----------------
CMD_GET_ID,   RSP_GET_ID   = 0xDA, 0x6D
CMD_SET_ID,   RSP_SET_ID   = 0xAA, 0x55
CMD_SET_POS,  RSP_SET_POS  = 0x76, 0x56
CMD_FORCE_A2, RSP_FORCE_A2 = 0x33, 0x34
CMD_FORCE_A1, RSP_FORCE_A1 = 0x35, 0x36
CMD_RESET_DEF, RSP_RESET_DEF = 0xB4, 0x5A  # Reset Default Role & Flags, args 'A','S'

BID = 0x1F  # broadcast (only when exactly one device on the bus)

# ---------------- Utilities ----------------
freshness = 0  # 0..15

def crc16_volz(cmd, dev_id, a1, a2):
    """CRC16 poly=0x8005, init=0xFFFF, no-reflect, xorout=0; returns (hi, lo)"""
    crc = 0xFFFF
    for b in (cmd, dev_id, a1, a2):
        crc = ((b << 8) ^ crc) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x8005) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return (crc >> 8) & 0xFF, crc & 0xFF

def deg_to_raw12(deg):
    deg = max(-170.0, min(170.0, float(deg)))
    raw = round(deg * 4096.0 / 360.0) & 0x0FFF
    return raw

def pack_setpos_args(angle_deg):
    global freshness
    raw = deg_to_raw12(angle_deg)
    arg1 = ((freshness & 0x0F) << 4) | ((raw >> 8) & 0x0F)
    arg2 = raw & 0xFF
    freshness = (freshness + 1) & 0x0F
    return arg1, arg2

def hexs(b):
    return " ".join(f"{x:02X}" for x in b)

def read_exact(ser, n, timeout_s=0.12):
    """Read up to n bytes until timeout; may return < n"""
    t0 = time.time()
    buf = b""
    while len(buf) < n and (time.time() - t0) < timeout_s:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf += chunk
        else:
            time.sleep(0.001)
    return buf

# ---------------- Serial layer helpers ----------------
class Link:
    def __init__(self):
        self.ser = None
        self.manual_rts = False
        self.rts_tx_level = True
        self.wait_ms = 8
        self.timeout = 0.06

    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def open(self, port, baud):
        self.close()
        self.ser = serial.Serial(port, baudrate=int(baud), bytesize=8, parity='N', stopbits=1, timeout=0.02)

    def close(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def txrx(self, cmd, dev_id, a1=0, a2=0, expect_rsp=None):
        """Send a 6B frame and try to read a 6B response. Returns (ok, rsp_bytes or b'')."""
        if not self.is_open():
            raise RuntimeError("Serial not open")
        c_hi, c_lo = crc16_volz(cmd, dev_id, a1, a2)
        frame = bytes([cmd, dev_id, a1, a2, c_hi, c_lo])
        app.log(f"TX: {hexs(frame)}")
        self.ser.reset_input_buffer()

        if self.manual_rts:
            self.ser.setRTS(self.rts_tx_level)  # to TX
        self.ser.write(frame); self.ser.flush()
        if self.manual_rts:
            self.ser.setRTS(not self.rts_tx_level)  # to RX

        time.sleep(self.wait_ms / 1000.0)

        rsp = read_exact(self.ser, 6, timeout_s=self.timeout)
        if rsp == frame:  # local echo
            rsp2 = read_exact(self.ser, 6, timeout_s=self.timeout)
            if rsp2:
                rsp = rsp2

        if rsp:
            app.log(f"RX: {hexs(rsp)}")
            # optional CRC check
            r_hi, r_lo = crc16_volz(rsp[0], rsp[1], rsp[2], rsp[3])
            if r_hi != rsp[4] or r_lo != rsp[5]:
                app.log("!! RX CRC mismatch", tag="warn")
        else:
            app.log("RX: <no data>", tag="warn")

        ok = (len(rsp) == 6)
        if ok and expect_rsp is not None:
            ok = (rsp[0] == expect_rsp and rsp[1] == dev_id)
        return ok, rsp if ok else b""

link = Link()

# ---------------- GUI ----------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("VOLZ RS-485 Helper @ NTU EVTOL CENTRE")
        self.geometry("820x620")
        self.resizable(True, True)

        # Top: Port controls
        fr = ttk.LabelFrame(self, text="Serial")
        fr.pack(fill="x", padx=8, pady=6)

        ttk.Label(fr, text="Port").grid(row=0, column=0, padx=4, pady=4, sticky="w")
        self.cbo_port = ttk.Combobox(fr, width=16, state="readonly")
        self.cbo_port.grid(row=0, column=1, padx=4, pady=4)
        self.btn_refresh = ttk.Button(fr, text="Refresh", command=self.refresh_ports)
        self.btn_refresh.grid(row=0, column=2, padx=4, pady=4)

        ttk.Label(fr, text="Baud").grid(row=0, column=3, padx=12, pady=4, sticky="e")
        self.cbo_baud = ttk.Combobox(fr, width=10, state="readonly",
                                     values=("115200","57600","38400","19200","9600"))
        self.cbo_baud.set("115200")
        self.cbo_baud.grid(row=0, column=4, padx=4, pady=4)

        self.btn_open = ttk.Button(fr, text="Open", width=10, command=self.toggle_open)
        self.btn_open.grid(row=0, column=5, padx=8, pady=4)

        # Half-duplex options
        ttk.Label(fr, text="Manual RTS").grid(row=1, column=0, padx=4, pady=4, sticky="w")
        self.var_manual = tk.BooleanVar(value=False)
        self.chk_manual = ttk.Checkbutton(fr, variable=self.var_manual, command=self.on_manual_toggle)
        self.chk_manual.grid(row=1, column=1, padx=4, pady=4, sticky="w")

        ttk.Label(fr, text="RTS TX level").grid(row=1, column=2, padx=4, pady=4, sticky="e")
        self.var_rts_level = tk.BooleanVar(value=True)
        self.cbo_rts = ttk.Combobox(fr, width=5, state="readonly", values=("True","False"))
        self.cbo_rts.set("True")
        self.cbo_rts.grid(row=1, column=3, padx=4, pady=4, sticky="w")

        ttk.Label(fr, text="Wait ms").grid(row=1, column=4, padx=4, pady=4, sticky="e")
        self.ent_wait = ttk.Entry(fr, width=6)
        self.ent_wait.insert(0, "8")
        self.ent_wait.grid(row=1, column=5, padx=4, pady=4, sticky="w")

        # Middle: actions
        body = ttk.Frame(self)
        body.pack(fill="both", expand=True, padx=8, pady=4)

        # Column 1: Scan & ID
        col1 = ttk.LabelFrame(body, text="Scan / ID")
        col1.pack(side="left", fill="y", padx=6, pady=4)

        self.var_scan_broadcast = tk.BooleanVar(value=False)
        ttk.Checkbutton(col1, text="Use broadcast (only if single device)", variable=self.var_scan_broadcast).pack(anchor="w", padx=6, pady=4)

        self.btn_scan = ttk.Button(col1, text="Scan IDs", width=16, command=self.scan_ids)
        self.btn_scan.pack(padx=6, pady=4)

        ttk.Label(col1, text="Found IDs").pack(anchor="w", padx=6, pady=(8,2))
        self.lst_ids = tk.Listbox(col1, height=8, width=16, exportselection=False)
        self.lst_ids.pack(padx=6, pady=4)

        # Modify ID
        sep = ttk.Separator(col1, orient="horizontal")
        sep.pack(fill="x", padx=6, pady=6)
        ttk.Label(col1, text="Change ID").pack(anchor="w", padx=6, pady=2)

        frm_id = ttk.Frame(col1)
        frm_id.pack(padx=6, pady=2)

        ttk.Label(frm_id, text="From").grid(row=0, column=0, padx=3, pady=2)
        self.ent_old_id = ttk.Entry(frm_id, width=6)
        self.ent_old_id.insert(0, "3")
        self.ent_old_id.grid(row=0, column=1, padx=3, pady=2)

        ttk.Label(frm_id, text="To").grid(row=0, column=2, padx=3, pady=2)
        self.ent_new_id = ttk.Entry(frm_id, width=6)
        self.ent_new_id.insert(0, "4")
        self.ent_new_id.grid(row=0, column=3, padx=3, pady=2)

        self.btn_setid = ttk.Button(col1, text="Apply ID", width=16, command=self.change_id)
        self.btn_setid.pack(padx=6, pady=6)

        # Column 2: Position
        col2 = ttk.LabelFrame(body, text="Position")
        col2.pack(side="left", fill="y", padx=6, pady=4)

        ttk.Label(col2, text="Target ID").pack(anchor="w", padx=6, pady=(6,2))
        self.ent_pos_id = ttk.Entry(col2, width=8)
        self.ent_pos_id.insert(0, "1")
        self.ent_pos_id.pack(padx=6, pady=2)

        ttk.Label(col2, text="Angle (deg)").pack(anchor="w", padx=6, pady=(6,2))
        self.ent_angle = ttk.Entry(col2, width=10)
        self.ent_angle.insert(0, "30")
        self.ent_angle.pack(padx=6, pady=2)

        self.btn_sendpos = ttk.Button(col2, text="Send Position", width=16, command=self.send_position_once)
        self.btn_sendpos.pack(padx=6, pady=6)

        # Column 3: Master control
        col3 = ttk.LabelFrame(body, text="Master Control")
        col3.pack(side="left", fill="y", padx=6, pady=4)

        ttk.Label(col3, text="Target ID").pack(anchor="w", padx=6, pady=(6,2))
        self.ent_master_id = ttk.Entry(col3, width=8)
        self.ent_master_id.insert(0, "3")
        self.ent_master_id.pack(padx=6, pady=2)

        self.btn_a2 = ttk.Button(col3, text="ACE2 -> Master", width=16, command=self.force_a2)
        self.btn_a2.pack(padx=6, pady=6)
        self.btn_a1 = ttk.Button(col3, text="ACE1 -> Master", width=16, command=self.force_a1)
        self.btn_a1.pack(padx=6, pady=6)

        self.btn_reset_role = ttk.Button(col3, text="Reset Default Role", width=16, command=self.reset_default_role)
        self.btn_reset_role.pack(padx=6, pady=8)

        # Bottom: log
        self.txt = ScrolledText(self, height=16, font=("Consolas", 10))
        self.txt.pack(fill="both", expand=True, padx=8, pady=6)
        self.txt.tag_config("warn", foreground="orange red")

        self.refresh_ports()
        self.on_manual_toggle()

    # --------- helpers ----------
    def log(self, msg, tag=None):
        ts = time.strftime("%H:%M:%S")
        self.txt.insert("end", f"[{ts}] {msg}\n", tag)
        self.txt.see("end")

    # make globally accessible log
    def __getattr__(self, name):
        if name == "log":
            return self.log
        raise AttributeError

    # --------- button callbacks ----------
    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.cbo_port["values"] = ports
        if ports:
            self.cbo_port.set(ports[0])

    def on_manual_toggle(self):
        manual = self.var_manual.get()
        self.cbo_rts["state"] = "readonly" if manual else "disabled"
        link.manual_rts = manual
        link.rts_tx_level = (self.cbo_rts.get() == "True")
        try:
            link.wait_ms = int(self.ent_wait.get())
        except Exception:
            link.wait_ms = 8

    def toggle_open(self):
        try:
            if link.is_open():
                link.close()
                self.btn_open.config(text="Open")
                self.log("Serial closed.")
                return
            port = self.cbo_port.get().strip()
            if not port:
                messagebox.showwarning("Port", "Please select serial port")
                return
            baud = self.cbo_baud.get().strip() or "115200"
            link.open(port, baud)
            self.btn_open.config(text="Close")
            self.log(f"Serial open: {port} @ {baud}")
        except Exception as e:
            messagebox.showerror("Open failed", str(e))

    def scan_ids(self):
        if not link.is_open():
            messagebox.showwarning("Serial", "Please open serial port first")
            return
        self.lst_ids.delete(0, "end")
        use_bcast = self.var_scan_broadcast.get()
        try:
            if use_bcast:
                ok, rsp = link.txrx(CMD_GET_ID, BID, 0, 0, expect_rsp=RSP_GET_ID)
                if ok:
                    self.lst_ids.insert("end", f"0x{rsp[1]:02X}")
                else:
                    self.log("Broadcast scan: no response", tag="warn")
            else:
                found = []
                for i in range(1, 0x1F):
                    ok, rsp = link.txrx(CMD_GET_ID, i, 0, 0, expect_rsp=RSP_GET_ID)
                    if ok:
                        found.append(i)
                if found:
                    for i in found:
                        self.lst_ids.insert("end", f"0x{i:02X}")
                else:
                    self.log("No IDs found in 0x01..0x1E", tag="warn")
        except Exception as e:
            messagebox.showerror("Scan IDs", str(e))

    def change_id(self):
        if not link.is_open():
            messagebox.showwarning("Serial", "Please open the serial port first")
            return
        try:
            old_id = int(self.ent_old_id.get(), 0)
            new_id = int(self.ent_new_id.get(), 0)
            if not (1 <= old_id <= 0x1E or old_id == BID):
                raise ValueError("The old actuator ID must be in the range 0x01-0x1E\n--0x1F is reserved for broadcast--")
            if not (1 <= new_id <= 0x1E):
                raise ValueError("The new actuator ID must be in the range 0x01-0x1E\n--0x1F is reserved for broadcast--")
            ok, _ = link.txrx(CMD_SET_ID, old_id, new_id, new_id, expect_rsp=RSP_SET_ID)
            self.log("Set ID result: " + ("OK" if ok else "FAIL"), tag=None if ok else "warn")
            # quick verify
            time.sleep(0.02)
            ok2, _ = link.txrx(CMD_GET_ID, new_id, 0, 0, expect_rsp=RSP_GET_ID)
            self.log("Verify new ID: " + ("OK" if ok2 else "FAIL"), tag=None if ok2 else "warn")
        except Exception as e:
            messagebox.showerror("Set ID", str(e))

    def send_position_once(self):
        if not link.is_open():
            messagebox.showwarning("Serial", "Please open the serial port first")
            return
        try:
            dev_id = int(self.ent_pos_id.get(), 0)
            angle = float(self.ent_angle.get())
            arg1, arg2 = pack_setpos_args(angle)
            ok, _ = link.txrx(CMD_SET_POS, dev_id, arg1, arg2, expect_rsp=RSP_SET_POS)
            self.log(f"SetPos({angle:.1f}°) -> " + ("ACK" if ok else "----"), tag=None if ok else "warn")
        except Exception as e:
            messagebox.showerror("Set Position", str(e))

    def force_a2(self):
        if not link.is_open():
            messagebox.showwarning("Serial", "Please open the serial port first")
            return
        try:
            dev_id = int(self.ent_master_id.get(), 0)
            ok, _ = link.txrx(CMD_FORCE_A2, dev_id, 0xAA, 0x55, expect_rsp=RSP_FORCE_A2)
            self.log("ACE2 -> Master : " + ("OK" if ok else "FAIL"), tag=None if ok else "warn")
        except Exception as e:
            messagebox.showerror("ACE2 Master", str(e))

    def force_a1(self):
        if not link.is_open():
            messagebox.showwarning("Serial", "Please open the serial port first")
            return
        try:
            dev_id = int(self.ent_master_id.get(), 0)
            ok, _ = link.txrx(CMD_FORCE_A1, dev_id, 0xAA, 0x55, expect_rsp=RSP_FORCE_A1)
            self.log("ACE1 -> Master : " + ("OK" if ok else "FAIL"), tag=None if ok else "warn")
        except Exception as e:
            messagebox.showerror("ACE1 Master", str(e))

    def reset_default_role(self):
        if not link.is_open():
            messagebox.showwarning("Serial", "Please open the serial port first")
            return
        try:
            # 'A' 'S'
            ok, _ = link.txrx(CMD_RESET_DEF, int(self.ent_master_id.get(),0), 0x41, 0x53, expect_rsp=RSP_RESET_DEF)
            self.log("Reset Default Role : " + ("OK" if ok else "FAIL"), tag=None if ok else "warn")
        except Exception as e:
            messagebox.showerror("Reset Default", str(e))

# Expose app to link logger
app = None

def main():
    global app
    app = App()
    app.mainloop()

if __name__ == "__main__":
    main()
