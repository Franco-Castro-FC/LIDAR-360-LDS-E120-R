from __future__ import annotations

import argparse
import math
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path

try:
    from PIL import Image, ImageTk, ImageEnhance
except ImportError:
    Image = None  # type: ignore[assignment]
    ImageTk = None  # type: ignore[assignment]
    ImageEnhance = None  # type: ignore[assignment]

try:
    import serial
    from serial import SerialException
except ImportError:
    serial = None  # type: ignore[assignment]
    SerialException = Exception  # type: ignore[misc,assignment]


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from parsers import parser_cffa  # noqa: E402
from parsers.parser_cffa import CffaPacket, LidarPoint  # noqa: E402


@dataclass
class DisplayPoint:
    angle_deg: float
    distance_mm: int
    quality: int
    valid: bool
    updated_at: float


class ScanState:
    def __init__(
        self,
        angle_increment_deg: float,
        max_age_seconds: float,
        min_quality: int,
        angle_offset_deg: float,
    ) -> None:
        self.angle_increment_deg = angle_increment_deg
        self.max_age_seconds = max_age_seconds
        self.min_quality = min_quality
        self.angle_offset_deg = angle_offset_deg
        self.bin_count = max(1, round(360.0 / angle_increment_deg))
        self.points: list[DisplayPoint | None] = [None] * self.bin_count
        self.packet_count = 0
        self.point_count = 0
        self.rotation_count = 0
        self.last_start_angle: float | None = None
        self.last_update = 0.0
        self.error: str | None = None
        self.lock = threading.Lock()

    def adjust_angle_offset(self, delta_deg: float) -> float:
        with self.lock:
            self.angle_offset_deg = (self.angle_offset_deg + delta_deg) % 360.0
            self.points = [None] * self.bin_count
            return self.angle_offset_deg

    def reset_angle_offset(self) -> float:
        with self.lock:
            self.angle_offset_deg = 0.0
            self.points = [None] * self.bin_count
            return self.angle_offset_deg

    def ingest_packets(self, packets: list[CffaPacket]) -> None:
        now = time.monotonic()
        with self.lock:
            for packet in packets:
                self.packet_count += 1
                if (
                    self.last_start_angle is not None
                    and packet.start_angle_deg < self.last_start_angle - 180.0
                ):
                    self.rotation_count += 1
                self.last_start_angle = packet.start_angle_deg
                self._ingest_points(packet.all_points, now)
            if packets:
                self.last_update = now

    def _ingest_points(self, points: list[LidarPoint], now: float) -> None:
        for point in points:
            if point.valid and point.quality < self.min_quality:
                continue
            angle = (point.angle_deg + self.angle_offset_deg) % 360.0
            index = int(angle / self.angle_increment_deg) % self.bin_count
            self.points[index] = DisplayPoint(
                angle_deg=angle,
                distance_mm=point.distance_mm,
                quality=point.quality,
                valid=point.valid,
                updated_at=now,
            )
            if point.valid:
                self.point_count += 1

    def snapshot(self) -> tuple[list[DisplayPoint], dict[str, object]]:
        now = time.monotonic()
        with self.lock:
            points = [
                point
                for point in self.points
                if point is not None and now - point.updated_at <= self.max_age_seconds
            ]
            valid_points = [point for point in points if point.valid]
            closest = min(valid_points, key=lambda point: point.distance_mm) if valid_points else None
            status = {
                "packets": self.packet_count,
                "points_total": self.point_count,
                "points_visible": len(valid_points),
                "invalid_visible": len(points) - len(valid_points),
                "rotations": self.rotation_count,
                "closest": closest,
                "last_update_age": now - self.last_update if self.last_update else None,
                "angle_offset": self.angle_offset_deg,
                "error": self.error,
            }
        return points, status

    def set_error(self, message: str) -> None:
        with self.lock:
            self.error = message


class SerialReader(threading.Thread):
    def __init__(self, state: ScanState, port: str, baud: int, stop_event: threading.Event) -> None:
        super().__init__(daemon=True)
        self.state = state
        self.port = port
        self.baud = baud
        self.stop_event = stop_event

    def run(self) -> None:
        if serial is None:
            self.state.set_error("pyserial no esta instalado")
            return

        buffer = b""
        try:
            with serial.Serial(port=self.port, baudrate=self.baud, timeout=0.05) as ser:
                ser.reset_input_buffer()
                while not self.stop_event.is_set():
                    chunk = ser.read(ser.in_waiting or 1)
                    if chunk:
                        buffer += chunk
                        packets, buffer = parser_cffa.parse_buffer(buffer)
                        if packets:
                            self.state.ingest_packets(packets)
                    else:
                        time.sleep(0.005)
        except SerialException as exc:
            self.state.set_error(f"Error serial: {exc}")


class RawReplayReader(threading.Thread):
    def __init__(
        self,
        state: ScanState,
        path: Path,
        stop_event: threading.Event,
        loop: bool,
        bytes_per_second: int,
    ) -> None:
        super().__init__(daemon=True)
        self.state = state
        self.path = path
        self.stop_event = stop_event
        self.loop = loop
        self.bytes_per_second = bytes_per_second

    def run(self) -> None:
        if not self.path.exists():
            self.state.set_error(f"No existe el RAW: {self.path}")
            return

        data = self.path.read_bytes()
        chunk_size = 512
        delay = max(chunk_size / self.bytes_per_second, 0.001)
        buffer = b""

        while not self.stop_event.is_set():
            for offset in range(0, len(data), chunk_size):
                if self.stop_event.is_set():
                    return
                buffer += data[offset : offset + chunk_size]
                packets, buffer = parser_cffa.parse_buffer(buffer)
                if packets:
                    self.state.ingest_packets(packets)
                time.sleep(delay)
            if not self.loop:
                return
            buffer = b""


class LidarMapApp:
    def __init__(self, root: tk.Tk, state: ScanState, args: argparse.Namespace) -> None:
        self.root = root
        self.state = state
        self.args = args
        source = "RAW replay" if args.raw_file else f"LIVE {args.port} @ {args.baud}"
        self.source_label = source
        self.root.title(f"PACECAT LDS-E120-R Map - {source}")
        self.root.geometry("1040x760")
        self.root.minsize(780, 560)

        self.status_var = tk.StringVar(value="Inicializando...")
        self.canvas = tk.Canvas(root, bg="#070b10", highlightthickness=0)
        self.status = tk.Label(
            root,
            textvariable=self.status_var,
            anchor="w",
            bg="#20262d",
            fg="#edf2f7",
            padx=10,
            pady=8,
            font=("Segoe UI", 10),
        )
        self.status.pack(side=tk.TOP, fill=tk.X)
        self.canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.bind("<Left>", lambda _event: self.state.adjust_angle_offset(-1.0))
        self.root.bind("<Right>", lambda _event: self.state.adjust_angle_offset(1.0))
        self.root.bind("a", lambda _event: self.state.adjust_angle_offset(-1.0))
        self.root.bind("d", lambda _event: self.state.adjust_angle_offset(1.0))
        self.root.bind("q", lambda _event: self.state.adjust_angle_offset(-10.0))
        self.root.bind("e", lambda _event: self.state.adjust_angle_offset(10.0))
        self.root.bind("r", lambda _event: self.state.reset_angle_offset())
        self.root.bind("j", lambda _event: self.adjust_front_angle(-5.0))
        self.root.bind("l", lambda _event: self.adjust_front_angle(5.0))
        self.stop_event: threading.Event | None = None
        self.logo_image_path = args.logo_file
        self.logo_image: Image.Image | None = None
        self.logo_image_tk: tk.PhotoImage | None = None
        self.logo_render_cache: tuple[int, int, tk.PhotoImage] | None = None
        self._load_logo_image()
        self.draw()

    def attach_stop_event(self, stop_event: threading.Event) -> None:
        self.stop_event = stop_event

    def adjust_front_angle(self, delta_deg: float) -> None:
        self.args.front_angle = (self.args.front_angle + delta_deg) % 360.0

    def on_close(self) -> None:
        if self.stop_event is not None:
            self.stop_event.set()
        self.root.destroy()

    def draw(self) -> None:
        width = max(self.canvas.winfo_width(), 1)
        height = max(self.canvas.winfo_height(), 1)
        points, status = self.state.snapshot()

        self.canvas.delete("all")
        self._draw_scan_backdrop(width, height)
        self._draw_background_logo(width, height)
        self._draw_grid(width, height)
        self._draw_points(width, height, points)
        self._draw_robot(width, height)
        self._update_status(status)

        self.root.after(self.args.refresh_ms, self.draw)

    def _center_and_scale(self, width: int, height: int) -> tuple[float, float, float]:
        center_x = width / 2.0
        center_y = height / 2.0
        radius_px = min(width, height) * 0.44
        scale = radius_px / self.args.range_mm
        return center_x, center_y, scale

    def _draw_scan_backdrop(self, width: int, height: int) -> None:
        self.canvas.create_rectangle(0, 0, width, height, fill="#05080f", outline="")
        center_x, center_y, scale = self._center_and_scale(width, height)
        max_radius = self.args.range_mm * scale
        field_width = min(width * 0.80, max_radius * 1.65)
        field_height = min(height * 0.70, max_radius * 1.35)
        left = center_x - field_width / 2.0
        right = center_x + field_width / 2.0
        top = center_y - field_height / 2.0
        bottom = center_y + field_height / 2.0

        self.canvas.create_rectangle(left, top, right, bottom, fill="#0b1219", outline="")
        for x in range(round(left), round(right) + 1, 4):
            self.canvas.create_line(x, top, x, bottom, fill="#1f2d35", dash=(1, 3))

    def _draw_grid(self, width: int, height: int) -> None:
        center_x, center_y, scale = self._center_and_scale(width, height)
        max_radius = self.args.range_mm * scale

        self.canvas.create_oval(
            center_x - max_radius,
            center_y - max_radius,
            center_x + max_radius,
            center_y + max_radius,
            outline="#344352",
            width=2,
        )

        ring_step_mm = 1000
        for distance in range(ring_step_mm, self.args.range_mm + 1, ring_step_mm):
            radius = distance * scale
            self.canvas.create_oval(
                center_x - radius,
                center_y - radius,
                center_x + radius,
                center_y + radius,
                outline="#25323d",
            )
            self.canvas.create_text(
                center_x + 6,
                center_y - radius - 2,
                text=f"{distance // 1000}m",
                fill="#7f8c96",
                anchor="w",
                font=("Segoe UI", 8),
            )

        self.canvas.create_line(center_x, center_y - max_radius, center_x, center_y + max_radius, fill="#2b3944")
        self.canvas.create_line(center_x - max_radius, center_y, center_x + max_radius, center_y, fill="#2b3944")
        self.canvas.create_line(center_x, center_y, center_x, center_y - max_radius, fill="#435462", dash=(4, 4))
        self.canvas.create_text(
            center_x,
            center_y - max_radius - 14,
            text="0 deg datos",
            fill="#7f8c96",
            font=("Segoe UI", 9),
        )
        front_x, front_y = self._polar_to_canvas(
            center_x,
            center_y,
            self.args.front_angle,
            max_radius,
        )
        self.canvas.create_line(center_x, center_y, front_x, front_y, fill="#4cc7ff", width=3, arrow=tk.LAST)
        label_x, label_y = self._polar_to_canvas(
            center_x,
            center_y,
            self.args.front_angle,
            max_radius + 18,
        )
        self.canvas.create_text(
            label_x,
            label_y,
            text="frente fisico",
            fill="#9fdcff",
            font=("Segoe UI", 9),
        )
        self.canvas.create_text(
            center_x,
            center_y + max_radius + 14,
            text="180 deg",
            fill="#7f8c96",
            font=("Segoe UI", 9),
        )
        self.canvas.create_text(
            center_x + max_radius + 28,
            center_y,
            text="90",
            fill="#7f8c96",
            font=("Segoe UI", 9),
        )
        self.canvas.create_text(
            center_x - max_radius - 28,
            center_y,
            text="270",
            fill="#7f8c96",
            font=("Segoe UI", 9),
        )

    def _draw_points(self, width: int, height: int, points: list[DisplayPoint]) -> None:
        center_x, center_y, scale = self._center_and_scale(width, height)
        point_radius = self.args.point_size

        for point in points:
            draw_distance = point.distance_mm
            if not point.valid:
                draw_distance = self.args.invalid_radius_mm
            draw_distance = max(0, min(draw_distance, self.args.range_mm))
            x, y = self._polar_to_canvas(center_x, center_y, point.angle_deg, draw_distance * scale)
            color = "#3f4b55" if not point.valid else self._point_color(point.distance_mm)
            radius = max(1, point_radius - 1) if not point.valid else point_radius
            self.canvas.create_oval(
                x - radius,
                y - radius,
                x + radius,
                y + radius,
                fill=color,
                outline="",
            )

    def _draw_robot(self, width: int, height: int) -> None:
        center_x, center_y, _scale = self._center_and_scale(width, height)
        size = 14
        points = self._triangle_points(center_x, center_y, size, self.args.front_angle)
        self.canvas.create_polygon(points, fill="#edf2f7", outline="#101418")

    def _polar_to_canvas(
        self,
        center_x: float,
        center_y: float,
        angle_deg: float,
        radius: float,
    ) -> tuple[float, float]:
        radians = math.radians(angle_deg)
        return (
            center_x + math.sin(radians) * radius,
            center_y - math.cos(radians) * radius,
        )

    def _triangle_points(
        self,
        center_x: float,
        center_y: float,
        size: float,
        angle_deg: float,
    ) -> list[float]:
        tip = self._polar_to_canvas(center_x, center_y, angle_deg, size)
        left = self._polar_to_canvas(center_x, center_y, angle_deg + 140.0, size * 0.75)
        right = self._polar_to_canvas(center_x, center_y, angle_deg - 140.0, size * 0.75)
        return [tip[0], tip[1], left[0], left[1], right[0], right[1]]

    def _point_color(self, distance_mm: int) -> str:
        if distance_mm <= 500:
            return "#ff4d4d"
        if distance_mm <= 1500:
            return "#ffb84d"
        if distance_mm <= 3000:
            return "#f3e85b"
        return "#5ee0a0"

    def _load_logo_image(self) -> None:
        if self.logo_image_path is None:
            return

        try:
            if Image is not None and ImageTk is not None:
                self.logo_image = Image.open(self.logo_image_path).convert("RGBA")
                self.logo_image_tk = None
            else:
                self.logo_image_tk = tk.PhotoImage(file=str(self.logo_image_path))
        except Exception as exc:
            self.state.set_error(f"No se pudo cargar logo: {exc}")
            self.logo_image = None
            self.logo_image_tk = None

    def _draw_background_logo(self, width: int, height: int) -> None:
        if self.logo_image_path is None:
            return
        if width < 64 or height < 64:
            return

        if self.logo_image is not None and Image is not None and ImageTk is not None:
            cached = self.logo_render_cache
            if cached is not None and cached[0] == width and cached[1] == height:
                self.canvas.create_image(width / 2, height / 2, image=cached[2], anchor=tk.CENTER)
                return

            img = self.logo_image
            max_width = int(width * 0.55)
            max_height = int(height * 0.55)
            image_ratio = img.width / img.height
            if max_width / max_height > image_ratio:
                new_height = max_height
                new_width = round(image_ratio * max_height)
            else:
                new_width = max_width
                new_height = round(max_width / image_ratio)

            resampling = getattr(getattr(Image, "Resampling", Image), "LANCZOS")
            logo_image = img.resize((new_width, new_height), resampling).convert("RGBA")

            if ImageEnhance is not None:
                logo_image = ImageEnhance.Brightness(logo_image).enhance(0.70)
                logo_image = ImageEnhance.Color(logo_image).enhance(0.75)

            alpha_mask = logo_image.split()[3].point(lambda p: int(p * 0.35))
            logo_image.putalpha(alpha_mask)

            background = Image.new("RGBA", (width, height), (6, 8, 14, 255))
            paste_x = (width - new_width) // 2
            paste_y = (height - new_height) // 2
            background.paste(logo_image, (paste_x, paste_y), logo_image)

            self.logo_image_tk = ImageTk.PhotoImage(background)
            self.logo_render_cache = (width, height, self.logo_image_tk)
            self.canvas.create_image(width / 2, height / 2, image=self.logo_image_tk, anchor=tk.CENTER)
        else:
            self.canvas.create_image(width / 2, height / 2, image=self.logo_image_tk, anchor=tk.CENTER)

    def _prepare_logo_for_background(self, image: Image.Image) -> Image.Image:
        dark_background = Image.new("RGBA", image.size, (8, 10, 14, 255))
        faded = Image.blend(dark_background, image, alpha=0.10)
        if ImageEnhance is not None:
            faded = ImageEnhance.Brightness(faded).enhance(0.18)
            faded = ImageEnhance.Color(faded).enhance(0.65)
        return faded

    def _update_status(self, status: dict[str, object]) -> None:
        if status["error"]:
            self.status_var.set(str(status["error"]))
            return

        closest = status["closest"]
        closest_text = "Closest: --"
        if isinstance(closest, DisplayPoint):
            closest_text = f"Closest: {closest.distance_mm} mm @ {closest.angle_deg:.1f} deg"

        age = status["last_update_age"]
        age_text = "sin datos"
        if isinstance(age, float):
            age_text = f"{age:.2f}s desde ultimo dato"

        self.status_var.set(
            f"{self.source_label} | "
            f"Packets: {status['packets']} | "
            f"Vueltas: {status['rotations']} | "
            f"Validos: {status['points_visible']}/{self.state.bin_count} | "
            f"Sin retorno: {status['invalid_visible']} | "
            f"{closest_text} | "
            f"Offset: {float(status['angle_offset']):.1f} deg | "
            f"Frente: {self.args.front_angle:.1f} deg | "
            f"Rango: {self.args.range_mm / 1000:.1f} m | "
            f"{age_text} | "
            "A/D: gira datos, J/L: gira frente"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mapa 2D en vivo para PACECAT LDS-E120-R.")
    parser.add_argument("--port", default="COM9", help="Puerto COM. Por defecto: COM9.")
    parser.add_argument("--baud", type=int, default=230400, help="Baudrate. Por defecto: 230400.")
    parser.add_argument("--raw-file", type=Path, help="Reproduce un RAW .bin en vez de abrir el puerto serial.")
    parser.add_argument("--no-loop", action="store_true", help="No repetir el RAW al terminar.")
    parser.add_argument("--range-mm", type=int, default=6000, help="Radio visible del mapa en mm.")
    parser.add_argument("--quality-min", type=int, default=0, help="Calidad minima aceptada.")
    parser.add_argument("--angle-offset", type=float, default=0.0, help="Correccion angular en grados.")
    parser.add_argument(
        "--front-angle",
        type=float,
        default=0.0,
        help="Angulo visual del frente fisico. 0=arriba, 90=derecha, 180=abajo, 270=izquierda.",
    )
    parser.add_argument("--angle-step", type=float, default=0.6, help="Resolucion angular por bin en grados.")
    parser.add_argument("--max-age", type=float, default=1.0, help="Segundos que un punto queda visible.")
    parser.add_argument("--point-size", type=int, default=3, help="Radio del punto dibujado en pixeles.")
    parser.add_argument("--refresh-ms", type=int, default=50, help="Periodo de refresco del mapa.")
    parser.add_argument(
        "--invalid-radius-mm",
        type=int,
        default=6000,
        help="Radio donde se dibujan retornos invalidos o sin distancia.",
    )
    parser.add_argument(
        "--logo-file",
        type=Path,
        default=Path(__file__).resolve().with_name("logo.png"),
        help="Ruta al archivo de logo PNG que se muestra de fondo.",
    )
    parser.add_argument(
        "--replay-bytes-per-second",
        type=int,
        default=13360,
        help="Velocidad de reproduccion de RAW. Por defecto: similar a la captura real.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    state = ScanState(
        angle_increment_deg=args.angle_step,
        max_age_seconds=args.max_age,
        min_quality=args.quality_min,
        angle_offset_deg=args.angle_offset,
    )
    stop_event = threading.Event()

    if args.raw_file:
        reader = RawReplayReader(
            state=state,
            path=args.raw_file,
            stop_event=stop_event,
            loop=not args.no_loop,
            bytes_per_second=args.replay_bytes_per_second,
        )
    else:
        reader = SerialReader(state=state, port=args.port, baud=args.baud, stop_event=stop_event)

    root = tk.Tk()
    app = LidarMapApp(root, state, args)
    app.attach_stop_event(stop_event)
    reader.start()
    root.mainloop()
    stop_event.set()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
