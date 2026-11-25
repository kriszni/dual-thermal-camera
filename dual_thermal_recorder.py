#!/usr/bin/env python3
"""
Dual thermal recorder + LIVE DISPLAY + diagnostics.

Shows both camera feeds in windows while telling you exactly what each port
is doing (bytes received, protocol detected, device messages).

Controls (with either video window focused):
    r      start/stop recording BOTH cameras
    c      flat-field calibrate the MLX90640 (point at a blank wall)
    q/ESC  quit

Outputs: ~/Downloads/mlx_MMDDYYYY.mp4/.csv and lep_MMDDYYYY.mp4/.csv
(auto _2, _3 suffix if today's files exist)

Run:  python dual_thermal_recorder.py
"""

import csv
import os
import struct
import sys
import threading
import time

import cv2
import numpy as np
import serial
from serial.tools import list_ports

MLX_PORT = "/dev/cu.usbmodem101"
LEP_PORT = "/dev/cu.usbmodem2101"
BAUD = 921600
OUT_DIR = os.path.expanduser("~/Downloads")

MLX_MAGIC = b"\xAA\x55\xA5\x5A"
MLX_W, MLX_H = 32, 24
MLX_NPIX = MLX_W * MLX_H
MLX_PAYLOAD = 4 + MLX_NPIX * 4
MLX_PACKET = 4 + MLX_PAYLOAD + 2
MLX_SCALE = 20

LEP_HEADER = b"\xDE\xAD\xBE\xEF"
LEP_W, LEP_H = 160, 120
LEP_FRAME_BYTES = LEP_W * LEP_H * 2
LEP_SCALE = 6
LEP_FLIP = True
MLX_FLIP = True

FPS_FALLBACK = 8.0


def output_paths(prefix):
    datestr = time.strftime("%m%d%Y")
    base = os.path.join(OUT_DIR, f"{prefix}_{datestr}")
    candidate, n = base, 1
    while os.path.exists(candidate + ".mp4") or os.path.exists(candidate + ".csv"):
        n += 1
        candidate = f"{base}_{n}"
    return candidate + ".mp4", candidate + ".csv"


class CameraThread(threading.Thread):
    name_prefix = "cam"
    video_size = (640, 480)

    def __init__(self, port):
        super().__init__(daemon=True)
        self.port = port
        self.ser = None
        self.running = True
        self.recording = False
        self.lock = threading.Lock()
        self.frames, self.times, self.rows = [], [], []
        self.error = None
        # live/debug state
        self.latest_color = None          # most recent colorized frame
        self.frame_count = 0
        self.bytes_seen = 0
        self.diag_note = "waiting for data..."

    def open(self):
        self.ser = serial.Serial(self.port, BAUD, timeout=0.3)
        time.sleep(0.3)
        self.ser.reset_input_buffer()

    def read_one(self):
        raise NotImplementedError

    def colorize(self, raw):
        raise NotImplementedError

    def csv_header(self):
        raise NotImplementedError

    def close(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass

    def start_recording(self):
        with self.lock:
            self.frames, self.times, self.rows = [], [], []
            self.recording = True

    def stop_recording(self):
        with self.lock:
            self.recording = False
            frames, times, rows = self.frames, self.times, self.rows
            self.frames, self.times, self.rows = [], [], []
        if not frames:
            print(f"[{self.name_prefix}] no frames captured — nothing saved.")
            return
        mp4_path, csv_path = output_paths(self.name_prefix)
        elapsed = times[-1] - times[0]
        fps = (len(frames) - 1) / elapsed if elapsed > 0 and len(frames) > 1 else FPS_FALLBACK
        writer = cv2.VideoWriter(mp4_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, self.video_size)
        for fr in frames:
            writer.write(fr)
        writer.release()
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(self.csv_header())
            w.writerows(rows)
        print(f"[{self.name_prefix}] saved {len(frames)} frames ({elapsed:.1f}s @ {fps:.2f} fps)")
        print(f"[{self.name_prefix}]   -> {mp4_path}")
        print(f"[{self.name_prefix}]   -> {csv_path}")

    def run(self):
        try:
            self.open()
        except serial.SerialException as e:
            self.error = str(e)
            self.diag_note = f"PORT FAILED: {e}"
            print(f"[{self.name_prefix}] ERROR opening {self.port}: {e}")
            return
        print(f"[{self.name_prefix}] connected on {self.port}")
        while self.running:
            try:
                result = self.read_one()
            except Exception as e:
                if self.running:
                    self.diag_note = f"read error: {e}"
                    time.sleep(0.2)
                continue
            if result is None:
                continue
            raw, row_tail = result
            t = time.time()
            color = self.colorize(raw)
            self.latest_color = color
            self.frame_count += 1
            with self.lock:
                if self.recording:
                    self.frames.append(color)
                    self.times.append(t)
                    self.rows.append([f"{t:.3f}"] + row_tail)
        self.close()

    def shutdown(self):
        self.running = False


class MlxThread(CameraThread):
    """Auto-detects the MLX90640 firmware's protocol:
      - BINARY:  [AA 55 A5 5A][u32 id][768 f32][u16 checksum]   (our firmware)
      - ASCII:   timestamp,ambient,min,max,avg,<768 pixels>\\n    (lab firmware)
    """
    name_prefix = "mlx"
    video_size = (MLX_W * MLX_SCALE, MLX_H * MLX_SCALE)

    def __init__(self, port):
        super().__init__(port)
        self.buf = bytearray()
        self.protocol = None            # None -> undetected, "binary", "ascii"
        self.last_start_sent = 0.0
        self.checksum_fails = 0
        self.offset = None        # per-pixel flat-field offsets
        self._cal_frames = None   # collecting calibration frames when not None

    def open(self):
        super().open()
        self.ser.write(b"S")            # harmless no-op for the ASCII firmware
        self.last_start_sent = time.time()

    def close(self):
        try:
            if self.ser and self.protocol == "binary":
                self.ser.write(b"P")
        except Exception:
            pass
        super().close()

    # --- ASCII protocol -----------------------------------------------------
    def _try_parse_ascii_line(self, line):
        """Parse 'timestamp,ambient,min,max,avg,<768 px>' -> (raw, row_tail) or None."""
        parts = line.split(",")
        if len(parts) < 5 + MLX_NPIX:
            return None
        try:
            dev_ts = parts[0].strip()
            pixels = np.array([float(v) for v in parts[5:5 + MLX_NPIX]],
                              dtype=np.float32)
        except ValueError:
            return None
        raw = pixels.reshape(MLX_H, MLX_W)
        row_tail = [dev_ts, f"{raw.min():.2f}", f"{raw.max():.2f}",
                    f"{raw.mean():.2f}"] + [f"{v:.2f}" for v in pixels]
        return raw, row_tail

    def _read_ascii(self):
        nl = self.buf.find(b"\n")
        if nl < 0:
            if len(self.buf) > 60000:   # runaway garbage guard
                del self.buf[:30000]
            return None
        line = bytes(self.buf[:nl]).decode("ascii", errors="ignore").strip()
        del self.buf[:nl + 1]
        if not line or line.startswith("#"):
            return None
        result = self._try_parse_ascii_line(line)
        if result is None:
            # partial first line after connect, or a status line - skip it
            return None
        self.diag_note = "OK (ascii csv firmware)"
        return result

    # --- binary protocol ----------------------------------------------------
    def _read_binary(self):
        while True:
            idx = self.buf.find(MLX_MAGIC)
            if idx < 0:
                del self.buf[:-3]
                return None
            if idx > 0:
                del self.buf[:idx]
            if len(self.buf) < MLX_PACKET:
                return None
            payload = self.buf[4:4 + MLX_PAYLOAD]
            (checksum,) = struct.unpack_from("<H", self.buf, 4 + MLX_PAYLOAD)
            if (sum(payload) & 0xFFFF) != checksum:
                self.checksum_fails += 1
                self.diag_note = f"checksum fails: {self.checksum_fails}"
                del self.buf[:4]
                continue
            (frame_id,) = struct.unpack_from("<I", payload, 0)
            temps = np.frombuffer(payload, dtype="<f4", count=MLX_NPIX, offset=4)
            raw = temps.reshape(MLX_H, MLX_W).copy()
            if MLX_FLIP:
                raw = np.rot90(raw, 2).copy()
            row_tail = [frame_id, f"{raw.min():.2f}", f"{raw.max():.2f}",
                        f"{raw.mean():.2f}"] + [f"{v:.2f}" for v in raw.flatten()]
            del self.buf[:MLX_PACKET]
            self.diag_note = "OK (binary firmware)"
            return raw, row_tail

    # --- protocol detection + dispatch ---------------------------------------
    def read_one(self):
        chunk = self.ser.read(4096)
        self.bytes_seen += len(chunk)
        self.buf += chunk

        if self.protocol is None:
            if MLX_MAGIC in self.buf:
                self.protocol = "binary"
                print("[mlx] detected BINARY protocol firmware")
            else:
                # A full ASCII frame line is ~4-5 KB of comma-separated floats.
                nl = self.buf.find(b"\n")
                if nl > 0:
                    line = bytes(self.buf[:nl]).decode("ascii", errors="ignore")
                    if line.count(",") >= 5 + MLX_NPIX - 1:
                        self.protocol = "ascii"
                        print("[mlx] detected ASCII CSV protocol firmware")
                    else:
                        del self.buf[:nl + 1]   # header/partial line, keep looking
            if self.protocol is None:
                if self.bytes_seen == 0 and time.time() - self.last_start_sent > 2.0:
                    self.ser.write(b"S")
                    self.last_start_sent = time.time()
                    self.diag_note = ("no bytes at all — board silent. "
                                      "Tap RESET on the MLX board / check port number.")
                elif self.bytes_seen > 0:
                    self.diag_note = f"detecting protocol... ({self.bytes_seen} bytes seen)"
                return None

        return self._read_binary() if self.protocol == "binary" else self._read_ascii()

    def colorize(self, raw):
        # flat-field calibration capture (collect 30 frames of a uniform scene)
        if self._cal_frames is not None:
            self._cal_frames.append(raw.astype(np.float32))
            self.diag_note = f"calibrating... {len(self._cal_frames)}/30"
            if len(self._cal_frames) >= 30:
                mean = np.mean(self._cal_frames, axis=0)
                self.offset = mean - mean.mean()   # per-pixel deviation from scene avg
                self._cal_frames = None
                self._ema = None
                print(f"[mlx] flat-field calibrated (offset range "
                      f"{self.offset.min():+.2f}..{self.offset.max():+.2f} C)")
        corrected = raw - self.offset if self.offset is not None else raw

        # temporal smoothing (EMA)
        if getattr(self, "_ema", None) is None:
            self._ema = corrected.astype(np.float32).copy()
        else:
            self._ema = 0.7 * self._ema + 0.3 * corrected
        # light spatial smoothing on the small image
        sm = cv2.GaussianBlur(self._ema, (3, 3), 0)

        vmin, vmax = np.percentile(sm, [1, 99])
        if vmax - vmin < 3.0:
            mid = 0.5 * (vmin + vmax)
            vmin, vmax = mid - 1.5, mid + 1.5
        norm = np.clip((sm - vmin) / (vmax - vmin), 0, 1)
        img8 = (norm * 255).astype(np.uint8)
        img8 = cv2.resize(img8, self.video_size, interpolation=cv2.INTER_CUBIC)
        return cv2.applyColorMap(img8, cv2.COLORMAP_JET)

    def start_calibration(self):
        self._cal_frames = []

    def csv_header(self):
        # 2nd column: frame counter (binary firmware) or device ms timestamp (ascii firmware)
        return (["unix_time", "frame_id_or_device_ms", "t_min", "t_max", "t_avg"]
                + [f"px_r{r}_c{c}" for r in range(MLX_H) for c in range(MLX_W)])


class LepThread(CameraThread):
    name_prefix = "lep"
    video_size = (LEP_W * LEP_SCALE, LEP_H * LEP_SCALE)

    def __init__(self, port):
        super().__init__(port)
        self.sniff = bytearray()   # rolling buffer to detect wrong-firmware

    def read_one(self):
        # sync to header, watching what actually arrives
        w = b""
        deadline = time.time() + 2.0
        while self.running and time.time() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            self.bytes_seen += 1
            self.sniff += b
            if len(self.sniff) > 512:
                del self.sniff[:-512]
            w = (w + b)[-4:]
            if w == LEP_HEADER:
                self.diag_note = "OK"
                break
        else:
            # timed out without finding header — diagnose
            if self.bytes_seen == 0:
                self.diag_note = ("no bytes at all — board silent. "
                                  "Tap RESET / check port.")
            elif MLX_MAGIC in self.sniff:
                self.diag_note = ("MLX packets detected on this port! "
                                  "This board is running MLX firmware — reflash Lepton fw.")
                print(f"[lep] !! {self.diag_note}")
            else:
                txt = self.sniff.decode("ascii", errors="ignore")
                printable = "".join(ch for ch in txt if ch.isprintable())
                if len(printable) > len(self.sniff) * 0.7 and printable.strip():
                    self.diag_note = f"getting TEXT not frames: '{printable.strip()[:50]}'"
                    print(f"[lep] device text: {printable.strip()[:120]}")
                else:
                    self.diag_note = (f"bytes flowing ({self.bytes_seen}) but no DEADBEEF "
                                      "header — wrong firmware or wrong port?")
            return None

        buf = b""
        while len(buf) < LEP_FRAME_BYTES:
            chunk = self.ser.read(LEP_FRAME_BYTES - len(buf))
            if not chunk:
                return None
            buf += chunk
        self.bytes_seen += len(buf)
        raw = np.frombuffer(buf, dtype=">u2").astype(np.float32).reshape(LEP_H, LEP_W)
        if LEP_FLIP:
            raw = np.flipud(raw)
        return raw, [f"{v:.0f}" for v in raw.flatten()]

    def colorize(self, raw):
        mn, mx = float(raw.min()), float(raw.max())
        span = max(mx - mn, 1.0)
        norm = ((raw - mn) / span * 255).clip(0, 255).astype(np.uint8)
        big = cv2.resize(norm, self.video_size, interpolation=cv2.INTER_NEAREST)
        return cv2.applyColorMap(big, cv2.COLORMAP_JET)

    def csv_header(self):
        return ["unix_time"] + [f"px_r{r}_c{c}" for r in range(LEP_H) for c in range(LEP_W)]


def placeholder(size, lines):
    w, h = size
    img = np.full((h, w, 3), 30, np.uint8)
    y = h // 2 - 12 * len(lines)
    for line in lines:
        cv2.putText(img, line, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (200, 200, 200), 1, cv2.LINE_AA)
        y += 26
    return img


def wrap(text, width=60):
    words, lines, cur = text.split(), [], ""
    for wd in words:
        if len(cur) + len(wd) + 1 > width:
            lines.append(cur)
            cur = wd
        else:
            cur = f"{cur} {wd}".strip()
    if cur:
        lines.append(cur)
    return lines or [""]


def main():
    print("Ports currently visible to the system:")
    for p in list_ports.comports():
        print(f"  {p.device}  ({p.description})")
    print()
    print(f"Using MLX={MLX_PORT}  LEP={LEP_PORT}")
    print("If those don't match the list above, edit the two lines at the top of this file.\n")

    mlx = MlxThread(MLX_PORT)
    lep = LepThread(LEP_PORT)
    mlx.start()
    lep.start()

    cv2.namedWindow("MLX90640", cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow("Lepton", cv2.WINDOW_AUTOSIZE)
    print("Controls: r=record | c=calibrate mlx flat-field | q=quit\n")
    

    recording = False
    try:
        while True:
            for cam, win, small in ((mlx, "MLX90640", (640, 480)),
                                    (lep, "Lepton", (640, 480))):
                if cam.latest_color is not None:
                    disp = cam.latest_color.copy()
                    status = f"{cam.frame_count} frames | {cam.bytes_seen//1024} KB"
                    cv2.putText(disp, status, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                                0.55, (255, 255, 255), 1, cv2.LINE_AA)
                    if recording:
                        cv2.circle(disp, (disp.shape[1] - 22, 22), 9, (0, 0, 255), -1)
                    # shrink for screen real estate
                    disp = cv2.resize(disp, small, interpolation=cv2.INTER_AREA)
                else:
                    lines = [f"{cam.name_prefix.upper()}: no frames yet",
                             f"port: {cam.port}",
                             f"bytes seen: {cam.bytes_seen}"] + wrap(cam.diag_note, 52)
                    disp = placeholder(small, lines)
                cv2.imshow(win, disp)

            key = cv2.waitKey(30) & 0xFF
            if key in (ord("q"), 27):
                break
            elif key == ord("r"):
                if not recording:
                    mlx.start_recording()
                    lep.start_recording()
                    recording = True
                    print(f"Recording BOTH (t0={time.time():.3f})...")
                else:
                    mlx.stop_recording()
                    lep.stop_recording()
                    recording = False
                    print()
            elif key == ord("c"):
                mlx.start_calibration()
                print("[mlx] point at a UNIFORM scene (blank wall), hold still...")
    finally:
        for c in (mlx, lep):
            if c.recording:
                c.stop_recording()
            c.shutdown()
        for c in (mlx, lep):
            c.join(timeout=2)
        cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()