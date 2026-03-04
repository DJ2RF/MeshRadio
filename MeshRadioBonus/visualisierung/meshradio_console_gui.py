#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MeshRadio Console GUI
---------------------

Live monitoring and control console for MeshRadio nodes.

Features
--------
Live WX plot
Battery monitoring
Command console
Auto command on next AWAKE
Link Quality display (based on RSSI)
Node list with last seen timestamps
TX window protection

Author
------
Friedrich Riedhammer DJ2RF

Copyright
---------
(c) 2026 Friedrich Riedhammer
fritz@nerdverlag.com
https://nerdverlag.com

For educational / amateur radio use
"""

import argparse
import datetime as dt
import re
import threading
import time
from collections import deque

import serial

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

import tkinter as tk
from tkinter import ttk

VERSION = "MRVIS v1.0 (c) nerdverlag.com"

WX_RE = re.compile(
    r"SENSOR:AWAKE\s+WX\s+"
    r"t=(?P<t>-?\d+(?:\.\d+)?)C\s+"
    r"p=(?P<p>\d+)hPa\s+"
    r"rh=(?P<rh>\d+(?:\.\d+)?)%\s+"
    r"bat=(?P<bat_mv>\d+)mV\s+"
    r"bat=(?P<bat_pct>\d+)%",
    re.IGNORECASE,
)

# Robust RSSI anywhere in the line
# Example: "from=DL1ABCF rssi=-37 ..." etc.
RSSI_RE = re.compile(r"\b(?:RSSI|rssi)\s*[:=]\s*(?P<rssi>-?\d+)\b")

# Sender callsign for node list (supports: from=DL1ABCF, from:DL1ABCF)
FROM_RE = re.compile(r"\bfrom\s*[:=]\s*(?P<call>[A-Za-z0-9/]+)\b")


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM16")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=0.2)
    ap.add_argument("--window", type=int, default=30)
    ap.add_argument("--dst", default="DL1ABCF")
    ap.add_argument("--max-points", type=int, default=300)
    ap.add_argument("--always-allow-tx", action="store_true")
    return ap.parse_args()


class Shared:
    def __init__(self, max_points: int):
        self.lock = threading.Lock()
        self.running = True
        self.ser = None

        self.samples = deque(maxlen=max_points)
        self.last_sample = None
        self.last_awake_ts = None

        self.auto_armed = False
        self.auto_payload = ""

        self.auto_last_sent_ts = None
        self.auto_last_sent_payload = ""

        # Link metrics (last known)
        self.last_rssi = None

        # Node list: call -> {"last_seen": datetime, "rssi": int|None, "last_line": str}
        self.nodes = {}


S: Shared | None = None


def make_send_cmd(dst: str, ack: int, payload: str) -> str:
    dst = (dst or "").strip() or "DL1ABCF"
    ack = 1 if ack else 0
    payload = (payload or "").strip()
    return f"send {dst} {ack} {payload}"


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
    try:
        ser.write(line.encode("utf-8"))
        ser.flush()
        print(f"[TX] {line.strip()}")
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
        return (False, 0)
    age = (dt.datetime.now() - t0).total_seconds()
    left = max(0, int(window_s - age))
    return (age <= window_s, left)


def rssi_to_quality(rssi: int | None) -> tuple[int | None, str]:
    """
    Map RSSI (dBm) to a 0..100 quality percentage + simple bar.
    Conservative defaults:
      -120 dBm -> 0%
      -30  dBm -> 100%
    """
    if rssi is None:
        return None, "—"
    lo, hi = -120.0, -30.0
    q = int(round((float(rssi) - lo) * 100.0 / (hi - lo)))
    q = max(0, min(100, q))
    blocks = 10
    filled = int(round(q / 100 * blocks))
    bar = "█" * filled + "░" * (blocks - filled)
    return q, bar


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
        m = WX_RE.search(line)
        if not m:
            continue

        # Optional RSSI anywhere in the line
        rssi = None
        rm = RSSI_RE.search(line)
        if rm:
            try:
                rssi = int(rm.group("rssi"))
            except Exception:
                rssi = None

        # Optional node callsign for node list
        call = None
        fm = FROM_RE.search(line)
        if fm:
            call = (fm.group("call") or "").strip() or None

        ts = dt.datetime.now()
        t_c = float(m.group("t"))
        p_hpa = int(m.group("p"))
        rh = float(m.group("rh"))
        mv = int(m.group("bat_mv"))
        pct = int(m.group("bat_pct"))

        do_auto = False
        auto_payload = ""

        with S.lock:
            S.last_sample = (ts, t_c, p_hpa, rh, mv, pct, line)
            S.samples.append((ts, t_c, p_hpa, rh, mv, pct))
            S.last_awake_ts = ts

            # Only update if present; otherwise keep last known value
            if rssi is not None:
                S.last_rssi = rssi

            # Update node list (only if we have a callsign)
            if call:
                ent = S.nodes.get(call)
                if ent is None:
                    ent = {"last_seen": ts, "rssi": None, "last_line": ""}
                    S.nodes[call] = ent
                ent["last_seen"] = ts
                if rssi is not None:
                    ent["rssi"] = rssi
                ent["last_line"] = line

            if S.auto_armed and S.auto_payload.strip():
                do_auto = True
                auto_payload = S.auto_payload.strip()
                S.auto_armed = False
                S.auto_payload = ""

        if do_auto:
            full = make_send_cmd(args.dst, 1, auto_payload)
            ok = send_cli_line(full)
            if ok:
                with S.lock:
                    S.auto_last_sent_ts = dt.datetime.now()
                    S.auto_last_sent_payload = auto_payload

    try:
        ser.close()
    except Exception:
        pass


def start_plot_window():
    plt.ion()
    fig = plt.figure(figsize=(11.5, 6.5))
    fig.canvas.manager.set_window_title(f"MeshRadio WX Plot — {VERSION}")

    ax1 = fig.add_subplot(2, 1, 1)
    ax2 = fig.add_subplot(2, 1, 2)

    (ln_t,) = ax1.plot([], [], label="Temp (°C)")
    (ln_p,) = ax1.plot([], [], label="Pressure (hPa)")
    ax1.grid(True)
    ax1.legend(loc="upper left")

    (ln_rh,) = ax2.plot([], [], label="RH (%)")
    (ln_mv,) = ax2.plot([], [], label="Battery (mV)")
    ax2.grid(True)

    ax2r = ax2.twinx()
    (ln_pct,) = ax2r.plot([], [], label="Battery (%)")
    ax2r.set_ylim(0, 100)

    ax2.legend(
        [ln_rh, ln_mv, ln_pct],
        [ln_rh.get_label(), ln_mv.get_label(), ln_pct.get_label()],
        loc="upper left",
    )

    def update():
        global S
        assert S is not None
        with S.lock:
            sam = list(S.samples)
            last = S.last_sample
            lrssi = S.last_rssi

        if sam:
            t0 = sam[0][0]
            xs = [(x[0] - t0).total_seconds() for x in sam]

            ln_t.set_data(xs, [x[1] for x in sam])
            ln_p.set_data(xs, [x[2] for x in sam])
            ln_rh.set_data(xs, [x[3] for x in sam])
            ln_mv.set_data(xs, [x[4] for x in sam])
            ln_pct.set_data(xs, [x[5] for x in sam])

            ax1.relim()
            ax1.autoscale_view()
            ax2.relim()
            ax2.autoscale_view()
            ax2.set_xlabel("Time (s)")

            if last:
                q, bar = rssi_to_quality(lrssi)
                link_txt = ""
                if q is not None:
                    link_txt = f"  LQ={q}% {bar}  RSSI={lrssi}dBm"

                fig.suptitle(
                    f"{last[0].strftime('%H:%M:%S')}  "
                    f"T={last[1]:.2f}°C  P={last[2]}hPa  RH={last[3]:.2f}%  "
                    f"BAT={last[4]}mV ({last[5]}%)"
                    f"{link_txt}",
                    fontsize=12,
                )

        fig.canvas.draw_idle()
        plt.pause(0.001)

    return fig, update


def start_control_window(args):
    global S
    assert S is not None

    root = tk.Tk()
    root.title(f"MeshRadio Control Panel — {VERSION}")

    frmTop = ttk.Frame(root, padding=10)
    frmTop.grid(row=0, column=0, sticky="ew")

    canvas = tk.Canvas(frmTop, width=40, height=40, highlightthickness=0)
    canvas.grid(row=0, column=0, rowspan=2, padx=(0, 10))
    lamp = canvas.create_oval(5, 5, 35, 35, fill="grey", outline="black")

    ttk.Label(frmTop, text=f"TX window ({args.window}s):", font=("Segoe UI", 11)).grid(
        row=0, column=1, sticky="w"
    )
    lblCnt = ttk.Label(frmTop, text="—", font=("Segoe UI", 12, "bold"))
    lblCnt.grid(row=1, column=1, sticky="w")

    frmMid = ttk.Frame(root, padding=10)
    frmMid.grid(row=1, column=0, sticky="ew")

    ttk.Label(frmMid, text="DST:").grid(row=0, column=0, sticky="w")
    varDst = tk.StringVar(value=args.dst)
    ttk.Entry(frmMid, textvariable=varDst, width=12).grid(
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

    # Status line (no popups)
    lblStatus = ttk.Label(frmAuto, text="status: ready", foreground="#444")
    lblStatus.grid(row=4, column=0, columnspan=2, sticky="w", pady=(4, 0))

    def set_status(msg: str, ok: bool = True):
        lblStatus.config(
            text=f"status: {msg}", foreground=("#228822" if ok else "#aa3333")
        )

    def guarded_send(payload: str):
        payload = (payload or "").strip()
        if not payload:
            set_status("payload is empty", ok=False)
            return

        dst = varDst.get().strip() or args.dst
        full = make_send_cmd(dst, 1, payload)

        if args.always_allow_tx:
            send_cli_line(full)
            set_status(f"sent (ACK=1) to {dst}: {payload}", ok=True)
            return

        ok, _left = tx_window_ok(args.window)
        if not ok:
            set_status("TX window closed (wait for next AWAKE)", ok=False)
            return

        if send_cli_line(full):
            set_status(f"sent (ACK=1) to {dst}: {payload}", ok=True)
        else:
            set_status("TX failed (serial not ready?)", ok=False)

    ttk.Button(frmMid, text="SEND (ACK=1)", command=lambda: guarded_send(varPay.get())).grid(
        row=0, column=4, sticky="e"
    )

    def mkbtn(txt, payload):
        return ttk.Button(frmBot, text=txt, command=lambda: guarded_send(payload))

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

    # Auto command: keep StringVar, but READ from Entry directly (bulletproof)
    varAuto = tk.StringVar(value="")
    entAuto = ttk.Entry(frmAuto, textvariable=varAuto, width=55)
    entAuto.grid(row=0, column=0, sticky="ew", padx=(0, 10))
    frmAuto.columnconfigure(0, weight=1)

    lblAutoState = tk.Label(frmAuto, text="DISARMED", bg="#aa3333", fg="white", width=26)
    lblAutoState.grid(row=0, column=1, sticky="e")

    lblDbg = ttk.Label(frmAuto, text="debug: auto_armed=0")
    lblDbg.grid(row=2, column=0, columnspan=2, sticky="w", pady=(8, 0))

    lblLog = ttk.Label(frmAuto, text="log: (none)")
    lblLog.grid(row=3, column=0, columnspan=2, sticky="w", pady=(2, 0))

    # ---- Link Quality display (based on RSSI) ----
    frmLink = ttk.Frame(root, padding=10)
    frmLink.grid(row=4, column=0, sticky="ew")

    ttk.Label(frmLink, text="Link:").grid(row=0, column=0, sticky="w")
    lblLink = ttk.Label(frmLink, text="LQ=—  RSSI=—", font=("Segoe UI", 11, "bold"))
    lblLink.grid(row=0, column=1, sticky="w", padx=(6, 0))
    frmLink.columnconfigure(1, weight=1)
    # ---------------------------------------------

    # ---- Node list (multiple Mesh nodes) ----
    frmNodes = ttk.LabelFrame(root, text="Nodes (last seen)", padding=10)
    frmNodes.grid(row=5, column=0, sticky="nsew", padx=10, pady=(0, 10))
    root.grid_rowconfigure(5, weight=1)
    root.grid_columnconfigure(0, weight=1)

    cols = ("call", "last_seen", "rssi", "lq")
    tv = ttk.Treeview(frmNodes, columns=cols, show="headings", height=7)
    tv.heading("call", text="Node")
    tv.heading("last_seen", text="Last seen")
    tv.heading("rssi", text="RSSI")
    tv.heading("lq", text="LQ")

    tv.column("call", width=120, anchor="w")
    tv.column("last_seen", width=90, anchor="e")
    tv.column("rssi", width=80, anchor="e")
    tv.column("lq", width=180, anchor="w")

    vsb = ttk.Scrollbar(frmNodes, orient="vertical", command=tv.yview)
    tv.configure(yscrollcommand=vsb.set)

    tv.grid(row=0, column=0, sticky="nsew")
    vsb.grid(row=0, column=1, sticky="ns")
    frmNodes.columnconfigure(0, weight=1)
    frmNodes.rowconfigure(0, weight=1)
    # ---------------------------------------

    sent_until = {"ts": None}

    def ui_set_disarmed():
        lblAutoState.config(text="DISARMED", bg="#aa3333")
        sent_until["ts"] = None

    def ui_set_armed():
        lblAutoState.config(text="ARMED (waiting AWAKE)", bg="#228822")
        sent_until["ts"] = None

    def ui_set_sent(payload: str):
        short = payload.strip()
        if len(short) > 18:
            short = short[:18] + "…"
        lblAutoState.config(text=f"SENT ✓ ({short})", bg="#caa200")
        sent_until["ts"] = dt.datetime.now() + dt.timedelta(seconds=3)

    def arm_auto():
        raw_entry = entAuto.get()
        raw_var = varAuto.get()
        cmd = (raw_entry or "").strip()

        lblLog.config(text=f"log: ARM clicked, ent='{raw_entry}', var='{raw_var}'")
        print(f"[UI] ARM click, ent='{raw_entry}', var='{raw_var}'")

        if not cmd:
            lblDbg.config(text="debug: auto_armed=0 (no cmd)")
            set_status("auto command is empty (enter e.g. CMD:RELAY ON)", ok=False)
            return

        with S.lock:
            S.auto_payload = cmd
            S.auto_armed = True
            S.auto_last_sent_ts = None
            S.auto_last_sent_payload = ""

        ui_set_armed()
        lblDbg.config(text="debug: auto_armed=1 (ARM OK)")
        set_status(f"ARMED: will send once on next AWAKE → '{cmd}'", ok=True)

    def disarm_auto():
        lblLog.config(text="log: DISARM clicked")
        print("[UI] DISARM click")

        with S.lock:
            S.auto_payload = ""
            S.auto_armed = False

        ui_set_disarmed()
        lblDbg.config(text="debug: auto_armed=0 (DISARM OK)")
        set_status("DISARMED", ok=True)

    btnArm = ttk.Button(frmAuto, text="ARM (next AWAKE)")
    btnArm.grid(row=1, column=0, sticky="w", pady=(8, 0))

    btnDisarm = ttk.Button(frmAuto, text="DISARM")
    btnDisarm.grid(row=1, column=1, sticky="e", pady=(8, 0))

    # Both command + bind, to guarantee click handling
    btnArm.configure(command=arm_auto)
    btnDisarm.configure(command=disarm_auto)
    btnArm.bind("<Button-1>", lambda e: arm_auto())
    btnDisarm.bind("<Button-1>", lambda e: disarm_auto())

    ui_set_disarmed()

    def refresh_nodes_table(nodes_snapshot: dict):
        # nodes_snapshot: call -> {"last_seen": dt, "rssi": int|None, ...}
        now = dt.datetime.now()

        rows = []
        for call, ent in nodes_snapshot.items():
            last_seen = ent.get("last_seen")
            rssi = ent.get("rssi")
            if isinstance(last_seen, dt.datetime):
                age_s = int((now - last_seen).total_seconds())
                last_txt = f"{age_s}s"
            else:
                last_txt = "—"
            q, bar = rssi_to_quality(rssi if isinstance(rssi, int) else None)
            rssi_txt = f"{rssi}dBm" if isinstance(rssi, int) else "—"
            lq_txt = f"{q}% {bar}" if q is not None else "—"
            rows.append((call, last_txt, rssi_txt, lq_txt, age_s if "age_s" in locals() else 10**9))

        # sort: most recently seen first (smallest age)
        rows.sort(key=lambda x: x[4])

        # Update treeview in-place by call as iid
        existing = set(tv.get_children(""))
        wanted = set()

        for call, last_txt, rssi_txt, lq_txt, _age in rows:
            iid = call
            wanted.add(iid)
            if iid in existing:
                tv.item(iid, values=(call, last_txt, rssi_txt, lq_txt))
            else:
                tv.insert("", "end", iid=iid, values=(call, last_txt, rssi_txt, lq_txt))

        # Remove nodes no longer present
        for iid in existing - wanted:
            try:
                tv.delete(iid)
            except Exception:
                pass

    def tick():
        ok, left = tx_window_ok(args.window)
        if ok:
            canvas.itemconfig(lamp, fill="green")
            lblCnt.config(text=f"{left}s (open)")
        else:
            canvas.itemconfig(lamp, fill="grey")
            lblCnt.config(text="closed")

        with S.lock:
            armed = S.auto_armed
            sent_ts = S.auto_last_sent_ts
            sent_pl = S.auto_last_sent_payload
            lrssi = S.last_rssi
            nodes_snapshot = dict(S.nodes)  # shallow copy

        # Update link quality label
        q, bar = rssi_to_quality(lrssi)
        if q is None:
            lblLink.config(text="LQ=—  RSSI=—")
        else:
            lblLink.config(text=f"LQ={q}% {bar}  RSSI={lrssi}dBm")

        # Update node list
        try:
            refresh_nodes_table(nodes_snapshot)
        except Exception:
            pass

        lblDbg.config(text=f"debug: auto_armed={1 if armed else 0}")

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
                lblAutoState.config(text="ARMED (waiting AWAKE)", bg="#228822")
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


def main():
    global S
    args = parse_args()
    S = Shared(max_points=args.max_points)

    th = threading.Thread(target=serial_reader, args=(args,), daemon=True)
    th.start()

    fig, plot_update = start_plot_window()
    root = start_control_window(args)

    def plot_tick():
        with S.lock:
            running = S.running
        if not running:
            try:
                plt.close(fig)
            except Exception:
                pass
            return
        plot_update()
        root.after(200, plot_tick)

    plot_tick()
    root.mainloop()


if __name__ == "__main__":
    main()