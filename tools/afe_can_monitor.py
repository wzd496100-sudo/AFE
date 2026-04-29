#!/usr/bin/env python3
"""Simple CAN monitor GUI for the AFE BQ76940 firmware."""

from __future__ import annotations

import queue
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field
from tkinter import messagebox, ttk

try:
    import can
except ImportError:  # pragma: no cover - handled at runtime
    can = None


APP_TITLE = "AFE CAN Monitor"
DEFAULT_BITRATE = "500000"
DEFAULT_INTERFACE = "slcan"
DEFAULT_CHANNEL = "COM3"
DEFAULT_SERIAL_BAUD = "115200"
MAX_LOG_LINES = 300
FRAME_IDS = (0x500, 0x501, 0x502, 0x503, 0x504, 0x505, 0x506)


BLOCK_REASON_TEXT = {
    0: "none",
    1: "comm_error",
    2: "fault",
    3: "ovrd_alert",
    4: "disabled",
    5: "cell_high",
}


SYS_STAT_FLAGS = (
    (0x01, "OCD"),
    (0x02, "SCD"),
    (0x04, "OV"),
    (0x08, "UV"),
    (0x10, "OVRD_ALERT"),
    (0x20, "DEVICE_XREADY"),
    (0x80, "CC_READY"),
)


DEBUG_FLAGS = (
    (0x01, "plain_fallback"),
    (0x02, "alert_afe_low"),
    (0x04, "alert_gauge_low"),
    (0x08, "chg_on"),
    (0x10, "dsg_on"),
    (0x20, "balancing"),
)


@dataclass
class AfeState:
    pack_mv: int | None = None
    sys_stat: int | None = None
    sys_ctrl2: int | None = None
    balance_active: int | None = None
    balance_candidate: int | None = None
    cells_mv: list[int | None] = field(default_factory=lambda: [None] * 13)
    balance_min_mv: int | None = None
    balance_max_mv: int | None = None
    balance_delta_mv: int | None = None
    dsg_block_reason: int | None = None
    chg_block_reason: int | None = None
    i2c_error: int | None = None
    can_tx_status: int | None = None
    debug_flags: int | None = None
    led_bar_mask: int | None = None
    cc_raw: int | None = None
    max_cell_mv: int | None = None
    chg_recover_count: int | None = None
    dsg_recover_count: int | None = None
    can_rx_count_low: int | None = None
    can_tx_fail_count_low: int | None = None
    frame_counts: dict[int, int] = field(default_factory=dict)
    last_frame_timestamp: float | None = None

    def mark_frame(self, arbitration_id: int, timestamp: float) -> None:
        self.frame_counts[arbitration_id] = self.frame_counts.get(arbitration_id, 0) + 1
        self.last_frame_timestamp = timestamp


def le_u16(data: bytes, offset: int) -> int:
    return data[offset] | (data[offset + 1] << 8)


def le_s16(data: bytes, offset: int) -> int:
    value = le_u16(data, offset)
    return value - 0x10000 if value & 0x8000 else value


def le_u32(data: bytes, offset: int) -> int:
    return (
        data[offset]
        | (data[offset + 1] << 8)
        | (data[offset + 2] << 16)
        | (data[offset + 3] << 24)
    )


def describe_flags(value: int | None, mapping: tuple[tuple[int, str], ...]) -> str:
    if value is None:
        return "-"
    names = [name for bit, name in mapping if value & bit]
    return ", ".join(names) if names else "none"


def describe_block_reason(value: int | None) -> str:
    if value is None:
        return "-"
    return BLOCK_REASON_TEXT.get(value, f"unknown({value})")


def format_mv(value: int | None) -> str:
    if value is None:
        return "-"
    return f"{value} mV"


def format_hex_byte(value: int | None) -> str:
    if value is None:
        return "-"
    return f"0x{value:02X}"


def format_hex_id(value: int) -> str:
    return f"0x{value:03X}"


class CanWorker:
    def __init__(self, interface: str, channel: str, bitrate: int, serial_baud: int):
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.serial_baud = serial_baud
        self.bus = None
        self.reader_thread = None
        self.stop_event = threading.Event()
        self.queue: queue.Queue[tuple[str, object]] = queue.Queue()

    def start(self) -> None:
        if can is None:
            raise RuntimeError("python-can is not installed")

        bus_kwargs = {
            "interface": self.interface,
            "channel": self.channel,
            "bitrate": self.bitrate,
        }
        if self.interface in {"slcan", "serial"}:
            bus_kwargs["ttyBaudrate"] = self.serial_baud

        self.bus = can.Bus(**bus_kwargs)
        self.stop_event.clear()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.reader_thread is not None:
            self.reader_thread.join(timeout=1.0)
        if self.bus is not None:
            self.bus.shutdown()
            self.bus = None

    def _reader_loop(self) -> None:
        assert self.bus is not None
        while not self.stop_event.is_set():
            try:
                msg = self.bus.recv(timeout=0.2)
            except Exception as exc:  # pragma: no cover - runtime hardware path
                self.queue.put(("error", exc))
                return

            if msg is not None:
                self.queue.put(("frame", msg))


class AfeCanMonitorApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title(APP_TITLE)
        self.root.geometry("1180x760")

        self.worker: CanWorker | None = None
        self.state = AfeState()

        self.interface_var = tk.StringVar(value=DEFAULT_INTERFACE)
        self.channel_var = tk.StringVar(value=DEFAULT_CHANNEL)
        self.bitrate_var = tk.StringVar(value=DEFAULT_BITRATE)
        self.serial_baud_var = tk.StringVar(value=DEFAULT_SERIAL_BAUD)
        self.status_var = tk.StringVar(value="Disconnected")
        self.last_update_var = tk.StringVar(value="No CAN frames yet")
        self.filter_var = tk.StringVar(value="")

        self.value_vars: dict[str, tk.StringVar] = {}
        self.frame_count_vars: dict[int, tk.StringVar] = {
            frame_id: tk.StringVar(value="0") for frame_id in FRAME_IDS
        }

        self._build_ui()
        self.root.after(100, self._poll_worker)
        self.root.after(500, self._refresh_age_text)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(2, weight=1)

        top = ttk.Frame(self.root, padding=10)
        top.grid(row=0, column=0, sticky="ew")
        for idx in range(10):
            top.columnconfigure(idx, weight=1 if idx in {1, 3, 5, 7} else 0)

        ttk.Label(top, text="Interface").grid(row=0, column=0, sticky="w", padx=(0, 6))
        interface_combo = ttk.Combobox(
            top,
            textvariable=self.interface_var,
            values=("slcan", "pcan", "vector", "kvaser", "socketcan", "serial"),
            state="normal",
        )
        interface_combo.grid(row=0, column=1, sticky="ew", padx=(0, 10))

        ttk.Label(top, text="Channel").grid(row=0, column=2, sticky="w", padx=(0, 6))
        ttk.Entry(top, textvariable=self.channel_var).grid(row=0, column=3, sticky="ew", padx=(0, 10))

        ttk.Label(top, text="Bitrate").grid(row=0, column=4, sticky="w", padx=(0, 6))
        ttk.Entry(top, textvariable=self.bitrate_var).grid(row=0, column=5, sticky="ew", padx=(0, 10))

        ttk.Label(top, text="Serial Baud").grid(row=0, column=6, sticky="w", padx=(0, 6))
        ttk.Entry(top, textvariable=self.serial_baud_var).grid(row=0, column=7, sticky="ew", padx=(0, 10))

        ttk.Button(top, text="Connect", command=self.connect).grid(row=0, column=8, sticky="ew", padx=(0, 6))
        ttk.Button(top, text="Disconnect", command=self.disconnect).grid(row=0, column=9, sticky="ew")

        info = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        info.grid(row=1, column=0, sticky="ew")
        info.columnconfigure(1, weight=1)
        info.columnconfigure(3, weight=1)

        ttk.Label(info, text="Status").grid(row=0, column=0, sticky="w", padx=(0, 6))
        ttk.Label(info, textvariable=self.status_var).grid(row=0, column=1, sticky="w")
        ttk.Label(info, text="Last Update").grid(row=0, column=2, sticky="w", padx=(20, 6))
        ttk.Label(info, textvariable=self.last_update_var).grid(row=0, column=3, sticky="w")

        main = ttk.Panedwindow(self.root, orient=tk.HORIZONTAL)
        main.grid(row=2, column=0, sticky="nsew", padx=10, pady=(0, 10))

        left = ttk.Frame(main, padding=4)
        right = ttk.Frame(main, padding=4)
        main.add(left, weight=3)
        main.add(right, weight=2)

        left.columnconfigure(0, weight=1)
        left.rowconfigure(1, weight=1)
        right.columnconfigure(0, weight=1)

        summary = ttk.LabelFrame(left, text="Decoded Status", padding=10)
        summary.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        for col in range(4):
            summary.columnconfigure(col, weight=1)

        summary_items = (
            ("pack_mv", "Pack Voltage"),
            ("sys_stat", "SYS_STAT"),
            ("sys_stat_flags", "SYS_STAT Flags"),
            ("sys_ctrl2", "SYS_CTRL2"),
            ("chg_on", "Charge FET"),
            ("dsg_on", "Discharge FET"),
            ("balance_active", "Balance Active"),
            ("balance_candidate", "Balance Candidate"),
            ("balance_min_mv", "Balance Min"),
            ("balance_max_mv", "Balance Max"),
            ("balance_delta_mv", "Balance Delta"),
            ("max_cell_mv", "Max Cell"),
            ("cc_raw", "CC Raw"),
            ("chg_block", "Charge Block"),
            ("dsg_block", "Discharge Block"),
            ("debug_flags", "Debug Flags"),
            ("i2c_error", "I2C Error"),
            ("can_tx_status", "CAN TX Status"),
            ("led_bar_mask", "LED Bar Mask"),
        )

        for idx, (key, label) in enumerate(summary_items):
            row = idx // 2
            col = (idx % 2) * 2
            ttk.Label(summary, text=label).grid(row=row, column=col, sticky="w", padx=(0, 6), pady=2)
            var = tk.StringVar(value="-")
            self.value_vars[key] = var
            ttk.Label(summary, textvariable=var).grid(row=row, column=col + 1, sticky="w", pady=2)

        cells = ttk.LabelFrame(left, text="Cell Voltages", padding=10)
        cells.grid(row=1, column=0, sticky="nsew")
        for col in range(4):
            cells.columnconfigure(col, weight=1)

        for idx in range(13):
            row = idx // 4
            col = (idx % 4) * 2
            ttk.Label(cells, text=f"Cell {idx + 1}").grid(row=row, column=col, sticky="w", padx=(0, 6), pady=2)
            var = tk.StringVar(value="-")
            self.value_vars[f"cell_{idx + 1}"] = var
            ttk.Label(cells, textvariable=var).grid(row=row, column=col + 1, sticky="w", pady=2)

        frame_stats = ttk.LabelFrame(right, text="Frame Counters", padding=10)
        frame_stats.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        frame_stats.columnconfigure(1, weight=1)
        for idx, frame_id in enumerate(FRAME_IDS):
            ttk.Label(frame_stats, text=format_hex_id(frame_id)).grid(row=idx, column=0, sticky="w", padx=(0, 8), pady=2)
            ttk.Label(frame_stats, textvariable=self.frame_count_vars[frame_id]).grid(row=idx, column=1, sticky="w", pady=2)

        raw_box = ttk.LabelFrame(right, text="Raw CAN Log", padding=10)
        raw_box.grid(row=1, column=0, sticky="nsew")
        right.rowconfigure(1, weight=1)
        raw_box.columnconfigure(0, weight=1)
        raw_box.rowconfigure(1, weight=1)

        filter_bar = ttk.Frame(raw_box)
        filter_bar.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        filter_bar.columnconfigure(1, weight=1)
        ttk.Label(filter_bar, text="ID Filter").grid(row=0, column=0, sticky="w", padx=(0, 6))
        ttk.Entry(filter_bar, textvariable=self.filter_var).grid(row=0, column=1, sticky="ew")

        self.log_widget = tk.Text(raw_box, height=24, wrap="none", state="disabled")
        self.log_widget.grid(row=1, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(raw_box, orient="vertical", command=self.log_widget.yview)
        scroll.grid(row=1, column=1, sticky="ns")
        self.log_widget.configure(yscrollcommand=scroll.set)

    def connect(self) -> None:
        if self.worker is not None:
            return

        if can is None:
            messagebox.showerror("Missing dependency", "python-can is not installed.\nRun install_can_monitor.bat first.")
            return

        try:
            bitrate = int(self.bitrate_var.get().strip())
            serial_baud = int(self.serial_baud_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid input", "Bitrate and Serial Baud must be integers.")
            return

        interface = self.interface_var.get().strip()
        channel = self.channel_var.get().strip()
        if not interface or not channel:
            messagebox.showerror("Invalid input", "Interface and Channel are required.")
            return

        self.state = AfeState()
        self._reset_log()
        for var in self.frame_count_vars.values():
            var.set("0")
        self._update_view()

        worker = CanWorker(interface=interface, channel=channel, bitrate=bitrate, serial_baud=serial_baud)
        try:
            worker.start()
        except Exception as exc:  # pragma: no cover - runtime hardware path
            messagebox.showerror("Connect failed", str(exc))
            return

        self.worker = worker
        self.status_var.set(f"Connected: {interface} {channel} @ {bitrate}")

    def disconnect(self) -> None:
        if self.worker is not None:
            self.worker.stop()
            self.worker = None
        self.status_var.set("Disconnected")

    def _poll_worker(self) -> None:
        worker = self.worker
        if worker is not None:
            while True:
                try:
                    item_type, payload = worker.queue.get_nowait()
                except queue.Empty:
                    break

                if item_type == "error":
                    self.status_var.set(f"CAN error: {payload}")
                    self.disconnect()
                    break

                self._handle_frame(payload)

        self.root.after(100, self._poll_worker)

    def _handle_frame(self, msg) -> None:
        data = bytes(msg.data)
        arbitration_id = int(msg.arbitration_id)
        timestamp = float(getattr(msg, "timestamp", time.time()))

        self.state.mark_frame(arbitration_id, timestamp)
        if arbitration_id == 0x500 and len(data) >= 8:
            self.state.pack_mv = le_u32(data, 0)
            self.state.sys_stat = data[4]
            self.state.sys_ctrl2 = data[5]
            self.state.balance_active = data[6]
            self.state.balance_candidate = data[7]
        elif arbitration_id == 0x501 and len(data) >= 8:
            self._update_cells(0, data)
        elif arbitration_id == 0x502 and len(data) >= 8:
            self._update_cells(4, data)
        elif arbitration_id == 0x503 and len(data) >= 8:
            self._update_cells(8, data)
        elif arbitration_id == 0x504 and len(data) >= 8:
            self.state.cells_mv[12] = le_u16(data, 0)
            self.state.balance_min_mv = le_u16(data, 2)
            self.state.balance_max_mv = le_u16(data, 4)
            self.state.balance_delta_mv = le_u16(data, 6)
        elif arbitration_id == 0x505 and len(data) >= 8:
            self.state.sys_stat = data[0]
            self.state.sys_ctrl2 = data[1]
            self.state.dsg_block_reason = data[2]
            self.state.chg_block_reason = data[3]
            self.state.i2c_error = data[4]
            self.state.can_tx_status = data[5]
            self.state.debug_flags = data[6]
            self.state.led_bar_mask = data[7]
        elif arbitration_id == 0x506 and len(data) >= 8:
            self.state.cc_raw = le_s16(data, 0)
            self.state.max_cell_mv = le_u16(data, 2)
            self.state.chg_recover_count = data[4]
            self.state.dsg_recover_count = data[5]
            self.state.can_rx_count_low = data[6]
            self.state.can_tx_fail_count_low = data[7]

        self.frame_count_vars.setdefault(arbitration_id, tk.StringVar()).set(str(self.state.frame_counts[arbitration_id]))
        self._append_log_line(msg)
        self._update_view()

    def _update_cells(self, start_idx: int, data: bytes) -> None:
        for idx in range(4):
            cell_index = start_idx + idx
            if cell_index < len(self.state.cells_mv):
                self.state.cells_mv[cell_index] = le_u16(data, idx * 2)

    def _append_log_line(self, msg) -> None:
        arbitration_id = int(msg.arbitration_id)
        filter_text = self.filter_var.get().strip().lower()
        if filter_text:
            frame_text = f"{arbitration_id:03x}"
            if filter_text not in frame_text and filter_text not in f"0x{frame_text}":
                return

        data_hex = " ".join(f"{byte:02X}" for byte in msg.data)
        timestamp = getattr(msg, "timestamp", time.time())
        line = f"{timestamp:12.3f}  ID={format_hex_id(arbitration_id)}  DLC={msg.dlc}  DATA={data_hex}\n"

        self.log_widget.configure(state="normal")
        self.log_widget.insert("end", line)
        line_count = int(float(self.log_widget.index("end-1c").split(".")[0]))
        if line_count > MAX_LOG_LINES:
            self.log_widget.delete("1.0", f"{line_count - MAX_LOG_LINES + 1}.0")
        self.log_widget.see("end")
        self.log_widget.configure(state="disabled")

    def _reset_log(self) -> None:
        self.log_widget.configure(state="normal")
        self.log_widget.delete("1.0", "end")
        self.log_widget.configure(state="disabled")

    def _update_view(self) -> None:
        state = self.state
        self.value_vars["pack_mv"].set(format_mv(state.pack_mv))
        self.value_vars["sys_stat"].set(format_hex_byte(state.sys_stat))
        self.value_vars["sys_stat_flags"].set(describe_flags(state.sys_stat, SYS_STAT_FLAGS))
        self.value_vars["sys_ctrl2"].set(format_hex_byte(state.sys_ctrl2))
        self.value_vars["chg_on"].set("ON" if ((state.sys_ctrl2 or 0) & 0x01) else "OFF")
        self.value_vars["dsg_on"].set("ON" if ((state.sys_ctrl2 or 0) & 0x02) else "OFF")
        self.value_vars["balance_active"].set(str(state.balance_active) if state.balance_active is not None else "-")
        self.value_vars["balance_candidate"].set(str(state.balance_candidate) if state.balance_candidate is not None else "-")
        self.value_vars["balance_min_mv"].set(format_mv(state.balance_min_mv))
        self.value_vars["balance_max_mv"].set(format_mv(state.balance_max_mv))
        self.value_vars["balance_delta_mv"].set(format_mv(state.balance_delta_mv))
        self.value_vars["max_cell_mv"].set(format_mv(state.max_cell_mv))
        self.value_vars["cc_raw"].set(str(state.cc_raw) if state.cc_raw is not None else "-")
        self.value_vars["chg_block"].set(describe_block_reason(state.chg_block_reason))
        self.value_vars["dsg_block"].set(describe_block_reason(state.dsg_block_reason))
        self.value_vars["debug_flags"].set(describe_flags(state.debug_flags, DEBUG_FLAGS))
        self.value_vars["i2c_error"].set(format_hex_byte(state.i2c_error))
        self.value_vars["can_tx_status"].set(format_hex_byte(state.can_tx_status))
        self.value_vars["led_bar_mask"].set(format_hex_byte(state.led_bar_mask))

        for idx, cell_value in enumerate(state.cells_mv, start=1):
            self.value_vars[f"cell_{idx}"].set(format_mv(cell_value))

        for frame_id in FRAME_IDS:
            self.frame_count_vars[frame_id].set(str(state.frame_counts.get(frame_id, 0)))

    def _refresh_age_text(self) -> None:
        last_ts = self.state.last_frame_timestamp
        if last_ts is None:
            self.last_update_var.set("No CAN frames yet")
        else:
            age = time.time() - last_ts
            self.last_update_var.set(f"{age:.1f} s ago")
        self.root.after(500, self._refresh_age_text)

    def _on_close(self) -> None:
        self.disconnect()
        self.root.destroy()


def main() -> int:
    root = tk.Tk()
    style = ttk.Style(root)
    try:
        style.theme_use("vista")
    except tk.TclError:
        pass
    app = AfeCanMonitorApp(root)
    app.status_var.set("Disconnected. Set interface/channel and click Connect.")
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
