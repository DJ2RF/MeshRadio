#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MeshRadio Console GUI
=====================

Live-Monitoring- und Steuerkonsole für MeshRadio-Nodes.

Funktionen
----------
- Live-Plots für:
    * Temperatur
    * Luftdruck
    * Luftfeuchte
    * Batteriespannung (mV)
    * Batteriestand (%)
    * RSSI
    * Link Quality
- Battery Monitoring
- Command Console
- One-shot Auto Command beim nächsten AWAKE
- Link-Quality-Anzeige auf Basis des RSSI
- Node-Liste mit Last-Seen-Zeiten
- TX-Window-Anzeige (nur für One-shot Auto Command)
- Help-Button mit Online-Dokumentation
- RX-Textlog mit DST-Filter

Wichtige Änderungen in dieser Version
------------------------------------
1. Die Quick-Buttons bleiben unverändert.
2. Die Auto-Command-Auswahl wurde erweitert.
3. Das RX-Log zeigt nur:
   - gesendete Kommandos (send ...)
   - "OK sent" direkt nach dem Senden
   - Antworten vom aktuell überwachten DST (from=<DST>)
4. Warnungen und Fehlerzeilen ("| W " / "| E ") werden unterdrückt.

Autor
-----
Friedrich Riedhammer DJ2RF

Copyright
---------
(c) 2026 Friedrich Riedhammer
fritz@nerdverlag.com
https://nerdverlag.com

Für Ausbildungs-, Experimentier- und Amateurfunkzwecke.
"""

import argparse
import datetime as dt
import re
import threading
import time
import webbrowser
from collections import deque

import serial

import matplotlib
matplotlib.use("TkAgg")

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import tkinter as tk
from tkinter import ttk


VERSION = "MRVIS v2.4 (c) nerdverlag.com"
HELP_URL = "https://nerdverlag.com/?page_id=719"

# ---------------------------------------------------------------------
# Regex / Parser
# ---------------------------------------------------------------------

AWAKE_RE = re.compile(r"\bSENSOR:AWAKE\b", re.IGNORECASE)

WX_RE = re.compile(
    r"SENSOR:AWAKE.*?WX.*?"
    r"t=(?P<t>-?\d+(?:\.\d+)?)C.*?"
    r"p=(?P<p>\d+)hPa.*?"
    r"rh=(?P<rh>\d+(?:\.\d+)?)%.*?"
    r"bat=(?P<bat_mv>\d+)mV.*?"
    r"bat=(?P<bat_pct>\d+)%",
    re.IGNORECASE,
)

RSSI_RE = re.compile(r"\b(?:RSSI|rssi)\s*[:=]\s*(?P<rssi>-?\d+)\b")
FROM_RE = re.compile(r"\bfrom\s*[:=]\s*(?P<call>[A-Za-z0-9/_-]+)")
SEND_RE = re.compile(r"^send\s+(?P<dst>\S+)\s+(?P<ack>[01])\s+(?P<payload>.+)$", re.IGNORECASE)


# ---------------------------------------------------------------------
# CLI arguments
# ---------------------------------------------------------------------

def parse_args():
    ap = argparse.ArgumentParser(
        prog="meshradio_console.py",
        description="MeshRadio Console GUI for live monitoring and control of MeshRadio nodes.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    ap.add_argument("--port", default="COM16", help="Serial port of the MeshRadio device")
    ap.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    ap.add_argument("--timeout", type=float, default=0.2, help="Serial read timeout in seconds")
    ap.add_argument("--window", type=int, default=30, help="TX window duration in seconds after AWAKE")
    ap.add_argument("--dst", default="DL1ABCF-1", help="Default destination callsign/node ID")
    ap.add_argument("--max-points", type=int, default=300, help="Maximum number of plot history samples")
    ap.add_argument("--rx-log-lines", type=int, default=300, help="Maximum number of lines kept in the RX text log")
    ap.add_argument("--always-allow-tx", action="store_true", help="Allow TX even when TX window is closed")

    return ap.parse_args()


# ---------------------------------------------------------------------
# Shared application state
# ---------------------------------------------------------------------

class Shared:
    def __init__(self, max_points: int, rx_log_lines: int):
        self.lock = threading.Lock()
        self.running = True
        self.ser = None

        # Sample format:
        # (ts, t_c, p_hpa, rh, mv, pct, rssi, lq)
        self.samples = deque(maxlen=max_points)

        # Last sample format:
        # (ts, t_c, p_hpa, rh, mv, pct, rssi, lq, raw_line)
        self.last_sample = None

        self.last_awake_ts = None

        self.auto_armed = False
        self.auto_payload = ""
        self.auto_dst = ""

        self.auto_last_sent_ts = None
        self.auto_last_sent_payload = ""

        self.last_rssi = None

        # call -> {"last_seen": datetime, "rssi": int|None, "last_line": str}
        self.nodes = {}

        # RX/TX monitor log lines as strings "HH:MM:SS | payload"
        self.rx_log = deque(maxlen=rx_log_lines)

        # Aktuell überwachte Gegenstelle für das RX-Log
        self.monitor_dst = ""
        self.last_tx_ts = None


S: Shared | None = None


# ---------------------------------------------------------------------
# Utility functions
# ---------------------------------------------------------------------

def make_send_cmd(dst: str, ack: int, payload: str) -> str:
    dst = (dst or "").strip() or "DL1ABCF-1"
    ack = 1 if ack else 0
    payload = (payload or "").strip()
    return f"send {dst} {ack} {payload}"


def append_rx_log(line: str):
    """Hängt eine Zeile an das sichtbare TX/RX-Monitorlog an."""
    global S
    assert S is not None

    ts = dt.datetime.now().strftime("%H:%M:%S")
    with S.lock:
        S.rx_log.append(f"{ts} | {line}")


def send_cli_line(line: str) -> bool:
    global S
    assert S is not None

    with S.lock:
        ser = S.ser

    if not ser:
        print("[WARN] Serial not ready")
        return False

    if not line.endswith("\n"):
        line += "\n"

    clean_line = line.strip()
    m = SEND_RE.match(clean_line)
    if m:
        with S.lock:
            S.monitor_dst = (m.group("dst") or "").strip()
            S.last_tx_ts = dt.datetime.now()
        append_rx_log(clean_line)

    try:
        ser.write(line.encode("utf-8"))
        ser.flush()
        print(f"[TX] {clean_line}")
        return True
    except Exception as e:
        print(f"[ERR] TX failed: {e}")
        return False


def tx_window_ok(window_s: int) -> tuple[bool, int]:
    global S
    assert S is not None

    with S.lock:
        t0 = S.last_awake_ts

    if not t0:
        return False, 0

    age = (dt.datetime.now() - t0).total_seconds()
    left = max(0, int(window_s - age))
    return age <= window_s, left


def rssi_to_quality(rssi: int | None) -> tuple[int | None, str]:
    if rssi is None:
        return None, "—"

    lo, hi = -120.0, -30.0
    q = int(round((float(rssi) - lo) * 100.0 / (hi - lo)))
    q = max(0, min(100, q))

    blocks = 10
    filled = int(round(q / 100 * blocks))
    bar = "█" * filled + "░" * (blocks - filled)
    return q, bar


def should_show_in_rx_log(line: str) -> bool:
    """
    Sichtbares RX/TX-Monitorlog:
    - send <dst> ...                     -> sichtbar (wird in send_cli_line geloggt)
    - OK sent                            -> sichtbar, kurz nach lokalem TX
    - Antworten vom überwachten DST      -> sichtbar, falls from=<monitor_dst>
    - W/E Debugmeldungen                 -> unsichtbar
    - alles andere                       -> unsichtbar
    """
    global S
    assert S is not None

    if not line:
        return False

    # Warnungen/Fehler komplett ausblenden
    if "| W " in line or "| E " in line:
        return False

    with S.lock:
        monitor_dst = (S.monitor_dst or "").strip()
        last_tx_ts = S.last_tx_ts

    # Lokale Sendebestätigung kurz nach einem TX anzeigen
    if "OK sent" in line:
        if last_tx_ts and (dt.datetime.now() - last_tx_ts).total_seconds() <= 5:
            return True
        return False

    # Nur Antworten vom aktuell überwachten Zielnode anzeigen
    if monitor_dst and f"from={monitor_dst}" in line:
        return True

    return False


# ---------------------------------------------------------------------
# Serial reader thread
# ---------------------------------------------------------------------

def serial_reader(args):
    global S
    assert S is not None

    try:
        ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    except Exception as e:
        print(f"[ERR] Cannot open {args.port}: {e}")
        with S.lock:
            S.running = False
        return

    with S.lock:
        S.ser = ser

    print(f"[OK] {VERSION} listening on {args.port} @ {args.baud}")

    while True:
        with S.lock:
            if not S.running:
                break

        try:
            raw = ser.readline()
        except Exception:
            time.sleep(0.05)
            continue

        if not raw:
            continue

        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            continue

        if should_show_in_rx_log(line):
            append_rx_log(line)

        awake = AWAKE_RE.search(line)
        wxm = WX_RE.search(line)

        rssi = None
        rm = RSSI_RE.search(line)
        if rm:
            try:
                rssi = int(rm.group("rssi"))
            except Exception:
                rssi = None

        call = None
        fm = FROM_RE.search(line)
        if fm:
            call = (fm.group("call") or "").strip() or None

        ts = dt.datetime.now()

        do_auto = False
        auto_payload = ""
        auto_dst = ""

        with S.lock:
            if awake:
                S.last_awake_ts = ts

            if rssi is not None:
                S.last_rssi = rssi

            if call:
                ent = S.nodes.get(call)
                if ent is None:
                    ent = {"last_seen": ts, "rssi": None, "last_line": ""}
                    S.nodes[call] = ent

                ent["last_seen"] = ts
                if rssi is not None:
                    ent["rssi"] = rssi
                ent["last_line"] = line

            if wxm:
                t_c = float(wxm.group("t"))
                p_hpa = int(wxm.group("p"))
                rh = float(wxm.group("rh"))
                mv = int(wxm.group("bat_mv"))
                pct = int(wxm.group("bat_pct"))

                hist_rssi = rssi if rssi is not None else S.last_rssi
                lq, _ = rssi_to_quality(hist_rssi)

                S.last_sample = (ts, t_c, p_hpa, rh, mv, pct, hist_rssi, lq, line)
                S.samples.append((ts, t_c, p_hpa, rh, mv, pct, hist_rssi, lq))

            if awake and S.auto_armed and S.auto_payload.strip():
                do_auto = True
                auto_payload = S.auto_payload.strip()
                auto_dst = (S.auto_dst or args.dst).strip() or args.dst
                S.auto_armed = False
                S.auto_payload = ""
                S.auto_dst = ""

        if do_auto:
            full = make_send_cmd(auto_dst, 1, auto_payload)
            ok = send_cli_line(full)
            if ok:
                with S.lock:
                    S.auto_last_sent_ts = dt.datetime.now()
                    S.auto_last_sent_payload = f"{auto_dst} | {auto_payload}"

    try:
        ser.close()
    except Exception:
        pass


# ---------------------------------------------------------------------
# Plot window
# ---------------------------------------------------------------------

def start_plot_window(root):
    global S
    assert S is not None

    win = tk.Toplevel(root)
    win.title(f"MeshRadio WX Plot — {VERSION}")
    win.geometry("1500x900")

    try:
        win.attributes("-topmost", False)
    except Exception:
        pass

    fig = Figure(figsize=(15, 9), dpi=100)
    gs = fig.add_gridspec(3, 3)

    ax_t = fig.add_subplot(gs[0, 0])
    ax_p = fig.add_subplot(gs[0, 1])
    ax_rh = fig.add_subplot(gs[0, 2])

    ax_mv = fig.add_subplot(gs[1, 0])
    ax_pct = fig.add_subplot(gs[1, 1])
    ax_empty_1 = fig.add_subplot(gs[1, 2])

    ax_rssi = fig.add_subplot(gs[2, 0])
    ax_lq = fig.add_subplot(gs[2, 1])
    ax_empty_2 = fig.add_subplot(gs[2, 2])

    ax_empty_1.axis("off")
    ax_empty_2.axis("off")

    (ln_t,) = ax_t.plot([], [], label="Temperature (°C)")
    (ln_p,) = ax_p.plot([], [], label="Pressure (hPa)")
    (ln_rh,) = ax_rh.plot([], [], label="Humidity (%)")
    (ln_mv,) = ax_mv.plot([], [], label="Battery (mV)")
    (ln_pct,) = ax_pct.plot([], [], label="Battery (%)")
    (ln_rssi,) = ax_rssi.plot([], [], label="RSSI (dBm)")
    (ln_lq,) = ax_lq.plot([], [], label="Link Quality (%)")

    all_axes = (ax_t, ax_p, ax_rh, ax_mv, ax_pct, ax_rssi, ax_lq)

    for ax in all_axes:
        ax.grid(True)
        ax.legend(loc="upper left")

    ax_t.set_ylabel("°C")
    ax_p.set_ylabel("hPa")
    ax_rh.set_ylabel("%")
    ax_mv.set_ylabel("mV")
    ax_pct.set_ylabel("%")
    ax_rssi.set_ylabel("dBm")
    ax_lq.set_ylabel("%")

    ax_mv.set_xlabel("Time (s)")
    ax_pct.set_xlabel("Time (s)")
    ax_rssi.set_xlabel("Time (s)")
    ax_lq.set_xlabel("Time (s)")

    ax_pct.set_ylim(0, 100)
    ax_lq.set_ylim(0, 100)

    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

    canvas = FigureCanvasTkAgg(fig, master=win)
    canvas_widget = canvas.get_tk_widget()
    canvas_widget.pack(fill="both", expand=True)

    closed = {"value": False}

    def on_close():
        closed["value"] = True
        try:
            win.destroy()
        except Exception:
            pass

    win.protocol("WM_DELETE_WINDOW", on_close)

    def update():
        global S
        assert S is not None

        if closed["value"]:
            return

        try:
            exists = win.winfo_exists()
        except Exception:
            exists = False

        if not exists:
            return

        with S.lock:
            sam = list(S.samples)
            last = S.last_sample
            lrssi = S.last_rssi

        if sam:
            t0 = sam[0][0]
            xs = [(x[0] - t0).total_seconds() for x in sam]

            ys_t = [x[1] for x in sam]
            ys_p = [x[2] for x in sam]
            ys_rh = [x[3] for x in sam]
            ys_mv = [x[4] for x in sam]
            ys_pct = [x[5] for x in sam]
            ys_rssi = [x[6] if x[6] is not None else float("nan") for x in sam]
            ys_lq = [x[7] if x[7] is not None else float("nan") for x in sam]

            ln_t.set_data(xs, ys_t)
            ln_p.set_data(xs, ys_p)
            ln_rh.set_data(xs, ys_rh)
            ln_mv.set_data(xs, ys_mv)
            ln_pct.set_data(xs, ys_pct)
            ln_rssi.set_data(xs, ys_rssi)
            ln_lq.set_data(xs, ys_lq)

            for ax in all_axes:
                ax.relim()
                ax.autoscale_view()

            ax_pct.set_ylim(0, 100)
            ax_lq.set_ylim(0, 100)

            if last:
                q, bar = rssi_to_quality(lrssi)
                link_txt = ""
                if q is not None:
                    link_txt = f"  LQ={q}% {bar}  RSSI={lrssi}dBm"

                fig.suptitle(
                    f"{last[0].strftime('%H:%M:%S')}  "
                    f"T={last[1]:.2f}°C  "
                    f"P={last[2]}hPa  "
                    f"RH={last[3]:.2f}%  "
                    f"BAT={last[4]}mV ({last[5]}%)"
                    f"{link_txt}",
                    fontsize=12,
                )

            fig.tight_layout(rect=[0, 0.03, 1, 0.95])

        canvas.draw_idle()

    return win, update


# ---------------------------------------------------------------------
# Control window
# ---------------------------------------------------------------------

def start_control_window(args, root):
    global S
    assert S is not None

    root.title(f"MeshRadio Control Panel — {VERSION}")

    frmTop = ttk.Frame(root, padding=10)
    frmTop.grid(row=0, column=0, sticky="ew")

    lamp_canvas = tk.Canvas(frmTop, width=40, height=40, highlightthickness=0)
    lamp_canvas.grid(row=0, column=0, rowspan=2, padx=(0, 10))
    lamp = lamp_canvas.create_oval(5, 5, 35, 35, fill="grey", outline="black")

    ttk.Label(frmTop, text=f"TX window ({args.window}s):", font=("Segoe UI", 11)).grid(
        row=0, column=1, sticky="w"
    )
    lblCnt = ttk.Label(frmTop, text="—", font=("Segoe UI", 12, "bold"))
    lblCnt.grid(row=1, column=1, sticky="w")

    frmMid = ttk.Frame(root, padding=10)
    frmMid.grid(row=1, column=0, sticky="ew")

    ttk.Label(frmMid, text="DST:").grid(row=0, column=0, sticky="w")
    varDst = tk.StringVar(value=args.dst)
    ttk.Entry(frmMid, textvariable=varDst, width=14).grid(
        row=0, column=1, sticky="w", padx=(6, 12)
    )

    ttk.Label(frmMid, text="Payload:").grid(row=0, column=2, sticky="w")
    varPay = tk.StringVar(value="CMD:STATUS?")
    ttk.Entry(frmMid, textvariable=varPay, width=48).grid(
        row=0, column=3, sticky="ew", padx=(6, 12)
    )
    frmMid.columnconfigure(3, weight=1)

    frmBot = ttk.Frame(root, padding=10)
    frmBot.grid(row=2, column=0, sticky="ew")

    frmAuto = ttk.LabelFrame(
        root, text="One-shot Auto Command (sent ONCE on next AWAKE)", padding=10
    )
    frmAuto.grid(row=3, column=0, sticky="ew", padx=10, pady=(0, 10))

    lblStatus = ttk.Label(frmAuto, text="status: ready", foreground="#444")
    lblStatus.grid(row=4, column=0, columnspan=2, sticky="w", pady=(4, 0))

    def set_status(msg: str, ok: bool = True):
        lblStatus.config(text=f"status: {msg}", foreground=("#228822" if ok else "#aa3333"))

    def open_help():
        try:
            webbrowser.open(HELP_URL)
            set_status("help page opened in browser", ok=True)
        except Exception as e:
            set_status(f"cannot open help page: {e}", ok=False)

    def guarded_send(payload: str):
        payload = (payload or "").strip()
        if not payload:
            set_status("payload is empty", ok=False)
            return

        dst = varDst.get().strip() or args.dst
        full = make_send_cmd(dst, 1, payload)

        if send_cli_line(full):
            set_status(f"sent (ACK=1) to {dst}: {payload}", ok=True)
        else:
            set_status("TX failed (serial not ready?)", ok=False)

    ttk.Button(frmMid, text="SEND (ACK=1)", command=lambda: guarded_send(varPay.get())).grid(
        row=0, column=4, sticky="e", padx=(0, 6)
    )

    ttk.Button(frmMid, text="HELP", command=open_help).grid(row=0, column=5, sticky="e")

    def mkbtn(txt, payload):
        return ttk.Button(frmBot, text=txt, command=lambda: guarded_send(payload))

    # Quick-Buttons bleiben absichtlich unverändert
    for i, b in enumerate(
        [
            mkbtn("RELAY ON", "CMD:RELAY ON"),
            mkbtn("RELAY OFF", "CMD:RELAY OFF"),
            mkbtn("RELAY TOGGLE", "CMD:RELAY TOGGLE"),
            mkbtn("STATUS?", "CMD:STATUS?"),
        ]
    ):
        b.grid(row=0, column=i, sticky="ew", padx=5)
        frmBot.columnconfigure(i, weight=1)

    AUTO_COMMANDS = [
        "CMD:STATUS?",
        "CMD:DISPLAY ON",
        "CMD:DISPLAY OFF",
        "CMD:WIFI ON",
        "CMD:WIFI OFF",
        "CMD:GPS",
        "CMD:BATT",
        "CMD:VERSION",
        "CMD:REBOOT",
        "CMD:RELAY ON",
        "CMD:RELAY OFF",
        "CMD:RELAY TOGGLE",
        "CMD:CRYPTO ON",
        "CMD:CRYPTO OFF",
    ]

    varAuto = tk.StringVar(value=AUTO_COMMANDS[0])

    cmbAuto = ttk.Combobox(
        frmAuto,
        textvariable=varAuto,
        values=AUTO_COMMANDS,
        state="readonly",
        width=30,
    )
    cmbAuto.grid(row=0, column=0, sticky="ew", padx=(0, 10))
    frmAuto.columnconfigure(0, weight=1)

    lblAutoState = tk.Label(frmAuto, text="DISARMED", bg="#aa3333", fg="white", width=26)
    lblAutoState.grid(row=0, column=1, sticky="e")

    lblDbg = ttk.Label(frmAuto, text="debug: auto_armed=0")
    lblDbg.grid(row=2, column=0, columnspan=2, sticky="w", pady=(8, 0))

    lblLog = ttk.Label(frmAuto, text="log: (none)")
    lblLog.grid(row=3, column=0, columnspan=2, sticky="w", pady=(2, 0))

    frmLink = ttk.Frame(root, padding=10)
    frmLink.grid(row=4, column=0, sticky="ew")

    ttk.Label(frmLink, text="Link:").grid(row=0, column=0, sticky="w")
    lblLink = ttk.Label(frmLink, text="LQ=—  RSSI=—", font=("Segoe UI", 11, "bold"))
    lblLink.grid(row=0, column=1, sticky="w", padx=(6, 0))
    frmLink.columnconfigure(1, weight=1)

    frmNodes = ttk.LabelFrame(root, text="Nodes (last seen)", padding=10)
    frmNodes.grid(row=5, column=0, sticky="nsew", padx=10, pady=(0, 10))

    frmRx = ttk.LabelFrame(root, text="DST Monitor Log", padding=10)
    frmRx.grid(row=6, column=0, sticky="nsew", padx=10, pady=(0, 10))

    root.grid_rowconfigure(5, weight=1)
    root.grid_rowconfigure(6, weight=1)
    root.grid_columnconfigure(0, weight=1)

    cols = ("call", "last_seen", "rssi", "lq")
    tv = ttk.Treeview(frmNodes, columns=cols, show="headings", height=7)

    tv.heading("call", text="Node")
    tv.heading("last_seen", text="Last seen")
    tv.heading("rssi", text="RSSI")
    tv.heading("lq", text="LQ")

    tv.column("call", width=180, anchor="w", stretch=True)
    tv.column("last_seen", width=90, anchor="e")
    tv.column("rssi", width=90, anchor="e")
    tv.column("lq", width=180, anchor="w")

    vsb = ttk.Scrollbar(frmNodes, orient="vertical", command=tv.yview)
    tv.configure(yscrollcommand=vsb.set)

    tv.grid(row=0, column=0, sticky="nsew")
    vsb.grid(row=0, column=1, sticky="ns")
    frmNodes.columnconfigure(0, weight=1)
    frmNodes.rowconfigure(0, weight=1)

    txtRx = tk.Text(frmRx, height=10, wrap="none", font=("Consolas", 10))
    txtRx.grid(row=0, column=0, sticky="nsew")
    txtRx.configure(state="disabled")

    rx_vsb = ttk.Scrollbar(frmRx, orient="vertical", command=txtRx.yview)
    rx_hsb = ttk.Scrollbar(frmRx, orient="horizontal", command=txtRx.xview)
    txtRx.configure(yscrollcommand=rx_vsb.set, xscrollcommand=rx_hsb.set)

    rx_vsb.grid(row=0, column=1, sticky="ns")
    rx_hsb.grid(row=1, column=0, sticky="ew")
    frmRx.columnconfigure(0, weight=1)
    frmRx.rowconfigure(0, weight=1)

    last_rx_render = {"text": ""}
    sent_until = {"ts": None}

    def ui_set_disarmed():
        lblAutoState.config(text="DISARMED", bg="#aa3333")
        sent_until["ts"] = None

    def ui_set_armed(dst: str, payload: str):
        short_dst = dst.strip()
        short_payload = payload.strip()

        if len(short_dst) > 12:
            short_dst = short_dst[:12] + "…"
        if len(short_payload) > 14:
            short_payload = short_payload[:14] + "…"

        lblAutoState.config(text=f"ARMED {short_dst} ← {short_payload}", bg="#228822")
        sent_until["ts"] = None

    def ui_set_sent(info: str):
        short = info.strip()
        if len(short) > 22:
            short = short[:22] + "…"
        lblAutoState.config(text=f"SENT ✓ ({short})", bg="#caa200")
        sent_until["ts"] = dt.datetime.now() + dt.timedelta(seconds=3)

    def arm_auto():
        selected = varAuto.get().strip()
        current_dst = varDst.get().strip() or args.dst

        lblLog.config(text=f"log: ARM clicked, dst='{current_dst}', selected='{selected}'")
        print(f"[UI] ARM click, dst='{current_dst}', selected='{selected}'")

        if not selected:
            lblDbg.config(text="debug: auto_armed=0 (no cmd)")
            set_status("auto command is empty", ok=False)
            return

        with S.lock:
            S.auto_payload = selected
            S.auto_dst = current_dst
            S.auto_armed = True
            S.auto_last_sent_ts = None
            S.auto_last_sent_payload = ""

        ui_set_armed(current_dst, selected)
        lblDbg.config(text=f"debug: auto_armed=1 dst='{current_dst}'")
        set_status(f"ARMED: next AWAKE sends to {current_dst} → '{selected}'", ok=True)

    def disarm_auto():
        lblLog.config(text="log: DISARM clicked")
        print("[UI] DISARM click")

        with S.lock:
            S.auto_payload = ""
            S.auto_dst = ""
            S.auto_armed = False

        ui_set_disarmed()
        lblDbg.config(text="debug: auto_armed=0 (DISARM OK)")
        set_status("DISARMED", ok=True)

    btnArm = ttk.Button(frmAuto, text="ARM (next AWAKE)", command=arm_auto)
    btnArm.grid(row=1, column=0, sticky="w", pady=(8, 0))

    btnDisarm = ttk.Button(frmAuto, text="DISARM", command=disarm_auto)
    btnDisarm.grid(row=1, column=1, sticky="e", pady=(8, 0))

    def refresh_nodes_table(nodes_snapshot: dict):
        now = dt.datetime.now()
        rows = []

        for call, ent in nodes_snapshot.items():
            last_seen = ent.get("last_seen")
            rssi = ent.get("rssi")

            age_s = 10**9
            if isinstance(last_seen, dt.datetime):
                age_s = int((now - last_seen).total_seconds())
                last_txt = f"{age_s}s"
            else:
                last_txt = "—"

            q, bar = rssi_to_quality(rssi if isinstance(rssi, int) else None)
            rssi_txt = f"{rssi}dBm" if isinstance(rssi, int) else "—"
            lq_txt = f"{q}% {bar}" if q is not None else "—"

            rows.append((call, last_txt, rssi_txt, lq_txt, age_s))

        rows.sort(key=lambda x: x[4])

        existing = set(tv.get_children(""))
        wanted = set()

        for call, last_txt, rssi_txt, lq_txt, _age in rows:
            iid = call
            wanted.add(iid)
            if iid in existing:
                tv.item(iid, values=(call, last_txt, rssi_txt, lq_txt))
            else:
                tv.insert("", "end", iid=iid, values=(call, last_txt, rssi_txt, lq_txt))

        for iid in existing - wanted:
            try:
                tv.delete(iid)
            except Exception:
                pass

    def refresh_rx_log(rx_lines: list[str]):
        text = "\n".join(rx_lines)
        if text == last_rx_render["text"]:
            return

        txtRx.configure(state="normal")
        txtRx.delete("1.0", "end")
        if text:
            txtRx.insert("1.0", text)
        txtRx.see("end")
        txtRx.configure(state="disabled")
        last_rx_render["text"] = text

    def tick():
        ok, left = tx_window_ok(args.window)
        if ok:
            lamp_canvas.itemconfig(lamp, fill="green")
            lblCnt.config(text=f"{left}s (auto only)")
        else:
            lamp_canvas.itemconfig(lamp, fill="grey")
            lblCnt.config(text="closed (auto only)")

        with S.lock:
            armed = S.auto_armed
            auto_dst = S.auto_dst
            auto_payload = S.auto_payload
            sent_ts = S.auto_last_sent_ts
            sent_pl = S.auto_last_sent_payload
            lrssi = S.last_rssi
            nodes_snapshot = dict(S.nodes)
            rx_lines = list(S.rx_log)
            monitor_dst = S.monitor_dst or args.dst

        q, bar = rssi_to_quality(lrssi)
        if q is None:
            lblLink.config(text="LQ=—  RSSI=—")
        else:
            lblLink.config(text=f"LQ={q}% {bar}  RSSI={lrssi}dBm")

        try:
            refresh_nodes_table(nodes_snapshot)
            refresh_rx_log(rx_lines)
        except Exception:
            pass

        lblDbg.config(text=f"debug: auto_armed={1 if armed else 0} monitor_dst='{monitor_dst}'")

        if sent_ts is not None:
            ui_set_sent(sent_pl)
            set_status(f"auto sent once → '{sent_pl}'", ok=True)
            with S.lock:
                S.auto_last_sent_ts = None
                S.auto_last_sent_payload = ""

        if sent_until["ts"] is not None and dt.datetime.now() >= sent_until["ts"]:
            sent_until["ts"] = None
            ui_set_disarmed()

        if sent_until["ts"] is None:
            if armed:
                ui_set_armed(auto_dst or args.dst, auto_payload or "")
            else:
                lblAutoState.config(text="DISARMED", bg="#aa3333")

        root.after(200, tick)

    tick()

    def on_close():
        with S.lock:
            S.running = False
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    return root


# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------

def main():
    global S
    args = parse_args()
    S = Shared(max_points=args.max_points, rx_log_lines=args.rx_log_lines)

    root = tk.Tk()
    root.geometry("800x690") # 1200x980") 

    th = threading.Thread(target=serial_reader, args=(args,), daemon=True)
    th.start()

    start_control_window(args, root)

    plot_win, plot_update = start_plot_window(root)

    def plot_tick():
        with S.lock:
            running = S.running

        if not running:
            try:
                if plot_win.winfo_exists():
                    plot_win.destroy()
            except Exception:
                pass
            return

        try:
            if plot_win.winfo_exists():
                plot_update()
        except Exception:
            pass

        root.after(200, plot_tick)

    plot_tick()
    root.mainloop()


if __name__ == "__main__":
    main()
