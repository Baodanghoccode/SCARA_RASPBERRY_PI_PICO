import sys
import math
import time
import serial
import serial.tools.list_ports
import numpy as np
from svgpathtools import svg2paths

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QGroupBox, QLabel, QComboBox, QPushButton, QTextEdit,
    QDoubleSpinBox, QSpinBox, QSlider, QFileDialog, QSplitter,
    QMessageBox, QProgressBar, QCheckBox
)
from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal, QPointF
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QFont, QPainterPath


# =====================================================================
# CẤU HÌNH ĐỘNG HỌC & GIỚI HẠN GÓC CHẾT ROBOT SCARA
# =====================================================================
class RobotKinematics:
    def __init__(self):
        self.L1 = 85.0
        self.L2 = 143.3
        self.L3 = 100.0

        self.motor_steps = 200
        self.microsteps = 16

        self.ratio_j1 = 110.0 / 20.0  # 5.5
        self.ratio_j2 = 90.0 / 20.0   # 4.5
        self.ratio_j3 = 92.0 / 20.0   # 4.6

        self.j2_min_deg = 52.5
        self.j2_max_deg = 302.5
        self.j3_min_deg = 52.5
        self.j3_max_deg = 302.5

        # Nhấc lên 4000 xung (~10mm ở vi bước 1/4 T8-2) | Hạ xuống 5000 xung (bù 1000 xung)
        self.z_lift_steps = 4600
        self.z_lower_steps = 5000
        self.ready_pose = [30.0, 120.0, 210.0]
        self._last_valid_phi = 0.0

    def is_joint_valid(self, j2_deg, j3_deg):
        return (self.j2_min_deg <= j2_deg <= self.j2_max_deg) and (self.j3_min_deg <= j3_deg <= self.j3_max_deg)

    def deg_to_steps(self, axis, deg):
        base = self.motor_steps * self.microsteps / 360.0
        if axis == 1:
            return int(round(deg * base * self.ratio_j1))
        elif axis == 2:
            return int(round((deg - 180.0) * base * self.ratio_j2))
        elif axis == 4:
            return int(round((deg - 180.0) * base * self.ratio_j3))
        return 0

    def steps_to_deg(self, axis, steps):
        base = self.motor_steps * self.microsteps / 360.0
        if axis == 1:
            return steps / (base * self.ratio_j1)
        elif axis == 2:
            return 180.0 + (steps / (base * self.ratio_j2))
        elif axis == 4:
            return 180.0 + (steps / (base * self.ratio_j3))
        return 0.0

    def forward_kinematics(self, j1_deg, j2_deg, j3_deg):
        rel_j2 = j2_deg - 180.0
        rel_j3 = j3_deg - 180.0
        t1 = math.radians(j1_deg)
        t2 = math.radians(j1_deg + rel_j2)
        t3 = math.radians(j1_deg + rel_j2 + rel_j3)

        x0, y0 = 0.0, 0.0
        x1 = x0 + self.L1 * math.cos(t1)
        y1 = y0 + self.L1 * math.sin(t1)
        x2 = x1 + self.L2 * math.cos(t2)
        y2 = y1 + self.L2 * math.sin(t2)
        x3 = x2 + self.L3 * math.cos(t3)
        y3 = y2 + self.L3 * math.sin(t3)
        return (x0, y0), (x1, y1), (x2, y2), (x3, y3)

    def _solve_ik_branch(self, wx, wy, phi, elbow_up):
        d2 = wx * wx + wy * wy
        max_r = self.L1 + self.L2
        min_r = abs(self.L1 - self.L2)
        if d2 > max_r * max_r or d2 < min_r * min_r:
            return None

        cos_t2 = (d2 - self.L1**2 - self.L2**2) / (2.0 * self.L1 * self.L2)
        cos_t2 = max(-1.0, min(1.0, cos_t2))
        rel_t2 = math.acos(cos_t2)
        if not elbow_up:
            rel_t2 = -rel_t2

        k1 = self.L1 + self.L2 * math.cos(rel_t2)
        k2 = self.L2 * math.sin(rel_t2)
        t1 = math.atan2(wy, wx) - math.atan2(k2, k1)
        rel_t3 = phi - (t1 + rel_t2)

        def norm_180(rad):
            deg = math.degrees(rad)
            while deg > 180: deg -= 360
            while deg < -180: deg += 360
            return deg

        def norm_360_centered(rad):
            deg = 180.0 + norm_180(rad)
            return deg % 360.0

        j1 = norm_180(t1)
        j2 = norm_360_centered(rel_t2)
        j3 = norm_360_centered(rel_t3)
        return j1, j2, j3

    def inverse_kinematics(self, x, y, phi_deg=0.0, current_joints=None):
        base_phi = self._last_valid_phi if phi_deg == 0.0 else phi_deg
        # Quét góc mịn 5 độ quanh góc cổ tay hiện tại để hạn chế tối đa việc xoay quanh bút
        offsets = [0.0]
        for step in range(5, 185, 5):
            offsets.extend([float(step), float(-step)])

        best_sol = None
        best_cost = float("inf")
        best_phi = base_phi

        for offset in offsets:
            test_phi_deg = base_phi + offset
            phi = math.radians(test_phi_deg)
            wx = x - self.L3 * math.cos(phi)
            wy = y - self.L3 * math.sin(phi)

            for elbow_up in (False, True):
                sol = self._solve_ik_branch(wx, wy, phi, elbow_up=elbow_up)
                if sol and self.is_joint_valid(sol[1], sol[2]):
                    if current_joints is None:
                        self._last_valid_phi = test_phi_deg
                        return sol

                    # Tính chi phí dịch chuyển khớp: phạt nặng nếu đảo khuỷu tay hoặc quay J1, J2 lớn
                    dj1 = abs(sol[0] - current_joints[0])
                    if dj1 > 180.0:
                        dj1 = 360.0 - dj1
                    dj2 = abs(sol[1] - current_joints[1])
                    dj3 = abs(sol[2] - current_joints[2])

                    # Kiểm tra xem có bị lật khuỷu tay J2 qua 180 độ không
                    flip_penalty = 500.0 if ((sol[1] - 180.0) * (current_joints[1] - 180.0) < 0) else 0.0
                    cost = dj1 * 2.0 + dj2 * 1.5 + dj3 * 1.0 + abs(offset) * 0.3 + flip_penalty

                    if cost < best_cost:
                        best_cost = cost
                        best_sol = sol
                        best_phi = test_phi_deg

            # Nếu đã tìm được nghiệm êm (không lật khuỷu và tổng lệch nhỏ) thì chốt luôn cho nhanh
            if best_sol is not None and best_cost < 25.0:
                break

        if best_sol is not None:
            self._last_valid_phi = best_phi
        return best_sol


# =====================================================================
# LUỒNG ĐỌC SERIAL CHUNG CHO CẢ PICO 2W VÀ ARDUINO UNO (TRỤC Z)
# =====================================================================
class SerialWorker(QThread):
    rx_signal = pyqtSignal(str)
    status_signal = pyqtSignal(list, int, int, int, int, int)
    z_pos_signal = pyqtSignal(int)
    event_signal = pyqtSignal(str)
    error_signal = pyqtSignal(str)

    def __init__(self, is_arduino_z=False):
        super().__init__()
        self.ser = None
        self.running = False
        self.is_arduino_z = is_arduino_z

    def connect_port(self, port, baudrate):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.05)
            self.running = True
            self.start()
            return True, "Đã kết nối thành công."
        except Exception as e:
            return False, str(e)

    def disconnect_port(self):
        self.running = False
        self.wait(300)
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def send_cmd(self, cmd_str):
        if self.ser and self.ser.is_open:
            if not cmd_str.startswith("#"):
                cmd_str = "#" + cmd_str
            if not cmd_str.endswith("\n"):
                cmd_str += "\n"
            try:
                self.ser.write(cmd_str.encode('utf-8'))
                return True
            except Exception as e:
                self.error_signal.emit(f"Lỗi gửi: {e}")
        return False

    def run(self):
        buf = ""
        while self.running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                    buf += data
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        line = line.strip()
                        if line:
                            self.parse_line(line)
                else:
                    time.sleep(0.005)
            except Exception as e:
                self.error_signal.emit(f"Mất kết nối Serial: {e}")
                self.running = False
                break

    def parse_line(self, line):
        if "#VALUE:" in line:
            idx = line.index("#VALUE:")
            parts = line[idx + 7:].split(",")
            if len(parts) == 9:
                vals = [int(x) for x in parts]
                self.status_signal.emit(vals[:4], vals[4], vals[5], vals[6], vals[7], vals[8])
            return
        elif "#ZVAL:" in line:
            idx = line.index("#ZVAL:")
            try:
                self.z_pos_signal.emit(int(line[idx + 6:].strip()))
            except ValueError:
                pass
            return

        self.rx_signal.emit(line)
        if "#EVT:" in line:
            idx = line.index("#EVT:")
            self.event_signal.emit(line[idx + 5:])
        elif "#ERR:" in line:
            idx = line.index("#ERR:")
            self.error_signal.emit(line[idx + 5:])


# =====================================================================
# WIDGET MÔ PHỎNG 2D NHÌN TỪ TRÊN XUỐNG (TOP-DOWN VIEW)
# =====================================================================
class RobotCanvas2D(QWidget):
    def __init__(self, kin: RobotKinematics):
        super().__init__()
        self.kin = kin
        self.setMinimumSize(480, 480)
        self.j1 = 0.0
        self.z_steps = 0
        self.j2 = 180.0
        self.j3 = 180.0
        self.pen_is_down = False

        self.svg_polylines = []
        self.drawn_traces = []

    def update_state(self, j1, z_steps, j2, j3, pen_down=False):
        self.j1, self.z_steps, self.j2, self.j3 = j1, z_steps, j2, j3
        self.pen_is_down = pen_down

        _, _, _, (px, py) = self.kin.forward_kinematics(j1, j2, j3)
        if self.pen_is_down:
            if not self.drawn_traces or self.drawn_traces[-1] is None:
                self.drawn_traces.append([(px, py)])
            else:
                last = self.drawn_traces[-1][-1]
                if math.hypot(px - last[0], py - last[1]) > 0.2:
                    self.drawn_traces[-1].append((px, py))
        else:
            if self.drawn_traces and self.drawn_traces[-1] is not None:
                self.drawn_traces.append(None)
        self.update()

    def clear_traces(self):
        self.drawn_traces.clear()
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#181A1F"))

        w, h = self.width(), self.height()
        max_reach = self.kin.L1 + self.kin.L2 + self.kin.L3
        scale = min(w, h) / (2.3 * max_reach)

        painter.translate(w / 2.0, h / 2.0)
        painter.scale(scale, -scale)

        grid_pen = QPen(QColor("#282C34"), 1.0 / scale)
        painter.setPen(grid_pen)
        limit = int(max_reach + 50)
        for g in range(-limit, limit + 1, 50):
            painter.drawLine(g, -limit, g, limit)
            painter.drawLine(-limit, g, limit, g)

        work_pen = QPen(QColor("#3E4451"), 1.5 / scale, Qt.PenStyle.DashLine)
        painter.setPen(work_pen)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawEllipse(QPointF(0, 0), max_reach, max_reach)
        min_reach = abs(self.kin.L1 - self.kin.L2)
        painter.drawEllipse(QPointF(0, 0), min_reach, min_reach)

        if self.svg_polylines:
            svg_pen = QPen(QColor("#56B6C2"), 1.2 / scale)
            svg_pen.setStyle(Qt.PenStyle.DotLine)
            painter.setPen(svg_pen)
            for poly in self.svg_polylines:
                if len(poly) > 1:
                    path = QPainterPath()
                    path.moveTo(poly[0][0], poly[0][1])
                    for pt in poly[1:]:
                        path.lineTo(pt[0], pt[1])
                    painter.drawPath(path)

        trace_pen = QPen(QColor("#98C379"), 2.2 / scale)
        painter.setPen(trace_pen)
        for seg in self.drawn_traces:
            if seg and len(seg) > 1:
                path = QPainterPath()
                path.moveTo(seg[0][0], seg[0][1])
                for pt in seg[1:]:
                    path.lineTo(pt[0], pt[1])
                painter.drawPath(path)

        p0, p1, p2, p3 = self.kin.forward_kinematics(self.j1, self.j2, self.j3)

        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(QColor(224, 108, 117, 50)))
        deadzone_span = (360.0 - self.kin.j2_max_deg) + self.kin.j2_min_deg
        r_dz2 = 28.0
        start_j2 = int((self.j1 + 180.0 - deadzone_span / 2.0) * 16)
        span_j2 = int(deadzone_span * 16)
        painter.drawPie(int(p1[0] - r_dz2), int(p1[1] - r_dz2), int(2 * r_dz2), int(2 * r_dz2), -start_j2, -span_j2)

        r_dz3 = 22.0
        arm2_angle = self.j1 + (self.j2 - 180.0)
        start_j3 = int((arm2_angle + 180.0 - deadzone_span / 2.0) * 16)
        painter.drawPie(int(p2[0] - r_dz3), int(p2[1] - r_dz3), int(2 * r_dz3), int(2 * r_dz3), -start_j3, -span_j2)

        painter.setPen(QPen(QColor("#E5C07B"), 10.0 / scale, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        painter.drawLine(QPointF(*p0), QPointF(*p1))

        painter.setPen(QPen(QColor("#61AFEF"), 8.0 / scale, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        painter.drawLine(QPointF(*p1), QPointF(*p2))

        painter.setPen(QPen(QColor("#C678DD"), 5.0 / scale, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        painter.drawLine(QPointF(*p2), QPointF(*p3))

        painter.setPen(QPen(QColor("#FFFFFF"), 1.5 / scale))
        painter.setBrush(QBrush(QColor("#E06C75")))
        painter.drawEllipse(QPointF(*p0), 9.0 / scale, 9.0 / scale)
        painter.setBrush(QBrush(QColor("#D19A66")))
        painter.drawEllipse(QPointF(*p1), 7.0 / scale, 7.0 / scale)
        painter.setBrush(QBrush(QColor("#56B6C2")))
        painter.drawEllipse(QPointF(*p2), 5.5 / scale, 5.5 / scale)

        painter.setBrush(QBrush(QColor("#FF3333") if self.pen_is_down else QColor("#00FF88")))
        painter.drawEllipse(QPointF(*p3), 4.5 / scale, 4.5 / scale)

        painter.resetTransform()
        painter.setPen(QColor("#ABB2BF"))
        painter.setFont(QFont("Consolas", 10))
        info = (
            f"DUAL MCU (Pico2W + Arduino Z) | Bút: X={p3[0]:.1f} mm, Y={p3[1]:.1f} mm | "
            f"J2={self.j2:.1f}°, J3={self.j3:.1f}° | Z={self.z_steps} xung"
        )
        painter.drawText(15, 25, info)


# =====================================================================
# WIDGET HIỂN THỊ BẢN VẼ SVG PHỤ (SVG PREVIEW)
# =====================================================================
class SvgPreviewWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(240, 195)
        self.raw_polylines = []
        self.bounds = (0, 100, 0, 100)

    def set_polylines(self, polylines):
        self.raw_polylines = polylines
        if polylines:
            xs = [pt[0] for poly in polylines for pt in poly]
            ys = [pt[1] for poly in polylines for pt in poly]
            if xs and ys:
                self.bounds = (min(xs), max(xs), min(ys), max(ys))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#FFFFFF"))

        if not self.raw_polylines:
            painter.setPen(QColor("#777777"))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "Chưa nạp file SVG\n(Nhấn 'Nạp file SVG')")
            return

        min_x, max_x, min_y, max_y = self.bounds
        bw = max(1e-3, max_x - min_x)
        bh = max(1e-3, max_y - min_y)

        margin = 20.0
        scale = min((self.width() - 2 * margin) / bw, (self.height() - 2 * margin) / bh)

        pen = QPen(QColor("#111111"), 1.5)
        painter.setPen(pen)

        for poly in self.raw_polylines:
            if len(poly) > 1:
                path = QPainterPath()
                x0 = margin + (poly[0][0] - min_x) * scale
                y0 = margin + (poly[0][1] - min_y) * scale
                path.moveTo(x0, y0)
                for pt in poly[1:]:
                    x = margin + (pt[0] - min_x) * scale
                    y = margin + (pt[1] - min_y) * scale
                    path.lineTo(x, y)
                painter.drawPath(path)


# =====================================================================
# CỬA SỔ CHÍNH ĐIỀU KHIỂN SCARA (GỘP PICO 2W + ARDUINO CNC SHIELD Z)
# =====================================================================
class ScaraMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Điều khiển ROBOT SCARA 4 BẬC")
        self.resize(1380, 880)

        self.kin = RobotKinematics()
        self.serial_worker = SerialWorker(is_arduino_z=False)
        self.z_worker = SerialWorker(is_arduino_z=True)

        self.cur_steps = [0, 0, 0, 0]  # [J1, J2, Z_arduino, J3]
        self.is_moving = False
        self.is_enabled = False
        self.is_estop = False

        self.pen_ready_on_paper = False
        self.waiting_for_manual_pen = False
        self.draw_queue = []
        self.total_queue_len = 0
        self.is_drawing_svg = False
        self.is_paused = False
        self.raw_svg_paths = []

        self.init_ui()
        self.connect_signals()

        self.stat_timer = QTimer(self)
        self.stat_timer.timeout.connect(self.poll_status)
        self.stat_timer.start(350)

        self.offline_jog_timer = QTimer(self)
        self.offline_jog_timer.timeout.connect(self.step_offline_jog)
        self.active_jog = None

        self.refresh_ports()
        self.sync_kinematics_to_ui()

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        main_layout.addWidget(splitter)

        # ==================== CỘT TRÁI ====================
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)

        # 1. Kết nối 2 bo mạch
        conn_group = QGroupBox("1. KẾT NỐI Pico 2W & Arduino + CNC Shield)")
        conn_grid = QGridLayout(conn_group)

        self.cb_port = QComboBox()
        self.btn_connect = QPushButton("Kết nối Pico")
        self.btn_connect.setStyleSheet("background-color: #2E7D32; color: white; font-weight: bold;")

        self.cb_z_port = QComboBox()
        self.btn_z_connect = QPushButton("Kết nối Arduino")
        self.btn_z_connect.setStyleSheet("background-color: #1565C0; color: white; font-weight: bold;")

        self.cb_baud = QComboBox()
        self.cb_baud.addItems(["115200", "230400", "9600", "57600", "921600"])
        self.btn_refresh = QPushButton("Làm mới COM")

        conn_grid.addWidget(QLabel("Pico (J1,J2,J3):"), 0, 0)
        conn_grid.addWidget(self.cb_port, 0, 1)
        conn_grid.addWidget(self.btn_connect, 0, 2)

        conn_grid.addWidget(QLabel("Arduino (Trục Z):"), 1, 0)
        conn_grid.addWidget(self.cb_z_port, 1, 1)
        conn_grid.addWidget(self.btn_z_connect, 1, 2)

        conn_grid.addWidget(QLabel("Baudrate chung:"), 2, 0)
        conn_grid.addWidget(self.cb_baud, 2, 1)
        conn_grid.addWidget(self.btn_refresh, 2, 2)
        left_layout.addWidget(conn_group)

        # 2. Bản vẽ SVG
        svg_group = QGroupBox("2. Bản vẽ SVG & Vùng vẽ")
        svg_layout = QVBoxLayout(svg_group)
        self.svg_preview = SvgPreviewWidget()
        svg_layout.addWidget(self.svg_preview)

        svg_cfg = QGridLayout()
        self.spin_svg_scale = QDoubleSpinBox()
        self.spin_svg_scale.setRange(0.05, 20.0)
        self.spin_svg_scale.setValue(1.0)
        self.spin_svg_scale.setSingleStep(0.1)

        self.spin_off_x = QDoubleSpinBox()
        self.spin_off_x.setRange(-400, 400)
        self.spin_off_x.setValue(180.0)

        self.spin_off_y = QDoubleSpinBox()
        self.spin_off_y.setRange(-400, 400)
        self.spin_off_y.setValue(-50.0)

        self.spin_step_mm = QDoubleSpinBox()
        self.spin_step_mm.setRange(0.2, 10.0)
        self.spin_step_mm.setValue(0.8)

        svg_cfg.addWidget(QLabel("Tỉ lệ (Scale):"), 0, 0)
        svg_cfg.addWidget(self.spin_svg_scale, 0, 1)
        svg_cfg.addWidget(QLabel("Bước mịn (mm):"), 0, 2)
        svg_cfg.addWidget(self.spin_step_mm, 0, 3)
        svg_cfg.addWidget(QLabel("Gốc X (mm):"), 1, 0)
        svg_cfg.addWidget(self.spin_off_x, 1, 1)
        svg_cfg.addWidget(QLabel("Gốc Y (mm):"), 1, 2)
        svg_cfg.addWidget(self.spin_off_y, 1, 3)
        svg_layout.addLayout(svg_cfg)

        svg_btns = QHBoxLayout()
        self.btn_load_svg = QPushButton("Nạp file SVG")
        self.btn_start_draw = QPushButton("Bắt đầu vẽ")
        self.btn_start_draw.setStyleSheet("background-color: #1565C0; color: white; font-weight: bold;")
        self.btn_pause_draw = QPushButton("Tạm dừng")
        self.btn_stop_draw = QPushButton("Hủy")
        svg_btns.addWidget(self.btn_load_svg)
        svg_btns.addWidget(self.btn_start_draw)
        svg_btns.addWidget(self.btn_pause_draw)
        svg_btns.addWidget(self.btn_stop_draw)
        svg_layout.addLayout(svg_btns)

        self.progress_bar = QProgressBar()
        svg_layout.addWidget(self.progress_bar)
        left_layout.addWidget(svg_group)

        # 3. Log hệ thống
        log_group = QGroupBox("3. Serial Log")
        log_layout = QVBoxLayout(log_group)
        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setFont(QFont("Consolas", 9))
        self.txt_log.setStyleSheet("background-color: #121212; color: #00E676;")
        btn_clear_log = QPushButton("Xóa Log")
        btn_clear_log.clicked.connect(self.txt_log.clear)
        log_layout.addWidget(self.txt_log)
        log_layout.addWidget(btn_clear_log)
        left_layout.addWidget(log_group)

        # ==================== CỘT GIỮA ====================
        center_panel = QWidget()
        center_layout = QVBoxLayout(center_panel)
        sim_group = QGroupBox("Mô phỏng quỹ đạo & Cánh tay SCARA 2D (Vùng đỏ = Góc chết)")
        sim_layout = QVBoxLayout(sim_group)
        self.canvas_2d = RobotCanvas2D(self.kin)
        sim_layout.addWidget(self.canvas_2d)

        sim_toolbar = QHBoxLayout()
        self.btn_clear_trace = QPushButton("Xóa nét vẽ mô phỏng")
        self.btn_clear_trace.clicked.connect(self.canvas_2d.clear_traces)
        self.lbl_robot_state = QLabel("Trạng thái: CHƯA KẾT NỐI | Driver: TẮT")
        self.lbl_robot_state.setStyleSheet("font-weight: bold; color: #D32F2F;")
        sim_toolbar.addWidget(self.lbl_robot_state)
        sim_toolbar.addStretch()
        sim_toolbar.addWidget(self.btn_clear_trace)
        sim_layout.addLayout(sim_toolbar)
        center_layout.addWidget(sim_group)

        # ==================== CỘT PHẢI ====================
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)

        power_group = QGroupBox("4. ĐIỀU KHIỂN")
        power_grid = QGridLayout(power_group)

        self.btn_en_on = QPushButton("Bật Driver")
        self.btn_en_off = QPushButton("Tắt Driver")
        self.btn_estop_reset = QPushButton("Reset")
        self.btn_estop_stop = QPushButton("E-STOP")
        self.btn_estop_stop.setStyleSheet("background-color: #C62828; color: white; font-weight: bold; padding: 6px;")

        self.btn_straight_zero = QPushButton("1. Gán Duỗi Thẳng (0, 180°, 180°)")
        self.btn_go_straight = QPushButton("Về gốc (J1,J2,J3)")
        self.btn_go_ready = QPushButton("2. Về vị trí bắt đầu")
        self.btn_go_ready.setStyleSheet("background-color: #EF6C00; color: white; font-weight: bold;")

        self.btn_pen_ready = QPushButton("3. ĐẶT BÚT XONG")
        self.btn_pen_ready.setStyleSheet("background-color: #2E7D32; color: white; font-weight: bold; padding: 7px;")

        self.btn_pen_up = QPushButton("Nhấc bút lên")
        self.chk_invert_z = QCheckBox("Đảo chiều nhấc(+/-)")
        self.chk_invert_z.setChecked(False)
        self.chk_invert_z.setToolTip("Bỏ tích: nhấc bút đi về -4000 xung. Tích vào: nhấc bút đi về +4000 xung.")

        power_grid.addWidget(self.btn_en_on, 0, 0)
        power_grid.addWidget(self.btn_en_off, 0, 1)
        power_grid.addWidget(self.btn_estop_reset, 1, 0)
        power_grid.addWidget(self.btn_estop_stop, 1, 1)
        power_grid.addWidget(self.btn_straight_zero, 2, 0)
        power_grid.addWidget(self.btn_go_straight, 2, 1)
        power_grid.addWidget(self.btn_go_ready, 3, 0, 1, 2)
        power_grid.addWidget(self.btn_pen_ready, 4, 0, 1, 2)
        power_grid.addWidget(self.btn_pen_up, 5, 0)
        power_grid.addWidget(self.chk_invert_z, 5, 1)
        right_layout.addWidget(power_group)

        # 5. Vị trí 4 Trục, MOVA & JOG
        axis_group = QGroupBox("5. ĐIỀU KHIỂN 4 TRỤC")
        axis_grid = QGridLayout(axis_group)

        headers = ["Trục", "Hiện tại", "Xung", "Đích đến", "JOG -", "JOG +"]
        for col, text in enumerate(headers):
            lbl = QLabel(text)
            lbl.setStyleSheet("font-weight: bold;")
            axis_grid.addWidget(lbl, 0, col)

        axis_names = [
            "Trục 1 (J1 °)",
            "Trục 2 (J2 °)",
            "Trục 3 (Z Ard)",
            "Trục 4 (J3 °)"
        ]
        self.lbl_pos_val = []
        self.lbl_pos_step = []
        self.spin_target = []

        for idx in range(4):
            row = idx + 1
            axis_grid.addWidget(QLabel(axis_names[idx]), row, 0)

            l_val = QLabel("0.00")
            l_val.setStyleSheet("color: #1976D2; font-weight: bold;")
            self.lbl_pos_val.append(l_val)
            axis_grid.addWidget(l_val, row, 1)

            l_stp = QLabel("0")
            self.lbl_pos_step.append(l_stp)
            axis_grid.addWidget(l_stp, row, 2)

            sp = QDoubleSpinBox()
            if idx == 2:
                sp.setRange(-15000.0, 15000.0)
                sp.setDecimals(0)
                sp.setSingleStep(50.0)
            elif idx in (1, 3):
                sp.setRange(52.5, 302.5)
                sp.setValue(180.0)
                sp.setDecimals(2)
            else:
                sp.setRange(-360.0, 360.0)
                sp.setDecimals(2)
            self.spin_target.append(sp)
            axis_grid.addWidget(sp, row, 3)

            btn_jm = QPushButton("◀")
            btn_jm.setFixedWidth(40)
            btn_jm.pressed.connect(lambda a=idx + 1: self.start_jog(a, -1))
            btn_jm.released.connect(self.stop_jog)
            axis_grid.addWidget(btn_jm, row, 4)

            btn_jp = QPushButton("▶")
            btn_jp.setFixedWidth(40)
            btn_jp.pressed.connect(lambda a=idx + 1: self.start_jog(a, 1))
            btn_jp.released.connect(self.stop_jog)
            axis_grid.addWidget(btn_jp, row, 5)

        self.btn_move_target = QPushButton("Chạy tới Đích")
        self.btn_move_target.setStyleSheet("background-color: #00838F; color: white; font-weight: bold; padding: 5px;")
        axis_grid.addWidget(self.btn_move_target, 5, 0, 1, 6)
        right_layout.addWidget(axis_group)

        speed_group = QGroupBox("6. Vận tốc & Thông số động cơ")
        speed_grid = QGridLayout(speed_group)

        self.slider_speed = QSlider(Qt.Orientation.Horizontal)
        self.slider_speed.setRange(5, 100)
        self.slider_speed.setValue(50)
        self.lbl_speed_pct = QLabel("50%")
        self.slider_speed.valueChanged.connect(lambda v: self.lbl_speed_pct.setText(f"{v}%"))

        self.spin_vmax = QSpinBox()
        self.spin_vmax.setRange(100, 15000)
        self.spin_vmax.setValue(3000)

        self.spin_acc = QSpinBox()
        self.spin_acc.setRange(100, 30000)
        self.spin_acc.setValue(2500)

        self.btn_apply_cfg = QPushButton("Nạp Vmax & Acc xuống Pico")

        speed_grid.addWidget(QLabel("Tỉ lệ vận tốc (f):"), 0, 0)
        speed_grid.addWidget(self.slider_speed, 0, 1)
        speed_grid.addWidget(self.lbl_speed_pct, 0, 2)
        speed_grid.addWidget(QLabel("Vmax (xung/s):"), 1, 0)
        speed_grid.addWidget(self.spin_vmax, 1, 1, 1, 2)
        speed_grid.addWidget(QLabel("Gia tốc (xung/s²):"), 2, 0)
        speed_grid.addWidget(self.spin_acc, 2, 1, 1, 2)
        speed_grid.addWidget(self.btn_apply_cfg, 3, 0, 1, 3)
        right_layout.addWidget(speed_group)

        mech_group = QGroupBox("7. Kích thước tay máy(mm) & Thông số Trục Z")
        mech_grid = QGridLayout(mech_group)
        self.spin_L1 = QDoubleSpinBox(); self.spin_L1.setRange(10, 500)
        self.spin_L2 = QDoubleSpinBox(); self.spin_L2.setRange(10, 500)
        self.spin_L3 = QDoubleSpinBox(); self.spin_L3.setRange(0, 200)
        self.spin_z_lift_steps = QSpinBox(); self.spin_z_lift_steps.setRange(10, 15000)

        self.spin_z_speed = QSpinBox()
        self.spin_z_speed.setRange(100, 6000)
        self.spin_z_speed.setValue(1000)

        mech_grid.addWidget(QLabel("L1 (mm):"), 0, 0); mech_grid.addWidget(self.spin_L1, 0, 1)
        mech_grid.addWidget(QLabel("L2 (mm):"), 0, 2); mech_grid.addWidget(self.spin_L2, 0, 3)
        mech_grid.addWidget(QLabel("L3 (mm):"), 1, 0); mech_grid.addWidget(self.spin_L3, 1, 1)
        mech_grid.addWidget(QLabel("Xung nhấc bút:"), 1, 2); mech_grid.addWidget(self.spin_z_lift_steps, 1, 3)
        mech_grid.addWidget(QLabel("Tốc độ Z (xung/s):"), 2, 0); mech_grid.addWidget(self.spin_z_speed, 2, 1)
        mech_grid.addWidget(QLabel("Góc J2,J3:"), 2, 2); mech_grid.addWidget(QLabel("52.5°..302.5°"), 2, 3)

        btn_update_mech = QPushButton("Cập nhật thông số cơ khí")
        btn_update_mech.clicked.connect(self.update_kinematics_from_ui)
        mech_grid.addWidget(btn_update_mech, 3, 0, 1, 4)
        right_layout.addWidget(mech_group)

        splitter.addWidget(left_panel)
        splitter.addWidget(center_panel)
        splitter.addWidget(right_panel)
        splitter.setSizes([340, 660, 380])

    # =================================================================
    # KẾT NỐI TÍN HIỆU & LOGIC ĐIỀU KHIỂN 2 BO MẠCH
    # =================================================================
    def connect_signals(self):
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_connect.clicked.connect(self.toggle_connection)
        self.btn_z_connect.clicked.connect(self.toggle_z_connection)

        self.serial_worker.rx_signal.connect(lambda s: self.log(f"[Pico] RX: {s}"))
        self.serial_worker.status_signal.connect(self.on_status_update)
        self.serial_worker.event_signal.connect(self.on_robot_event)
        self.serial_worker.error_signal.connect(self.on_robot_error)

        self.z_worker.rx_signal.connect(lambda s: self.log(f"[ArdZ] RX: {s}"))
        self.z_worker.z_pos_signal.connect(self.on_z_pos_update)
        self.z_worker.event_signal.connect(self.on_z_event)
        self.z_worker.error_signal.connect(self.on_robot_error)

        self.btn_en_on.clicked.connect(self.enable_all_drivers)
        self.btn_en_off.clicked.connect(self.disable_all_drivers)
        self.btn_estop_reset.clicked.connect(lambda: self.send_cmd("START"))
        self.btn_estop_stop.clicked.connect(self.emergency_stop)

        self.btn_straight_zero.clicked.connect(self.set_straight_zero)
        self.btn_go_straight.clicked.connect(self.move_to_straight)
        self.btn_go_ready.clicked.connect(self.move_to_ready)
        self.btn_pen_ready.clicked.connect(self.confirm_pen_ready)
        self.btn_pen_up.clicked.connect(self.pen_up)
        self.btn_move_target.clicked.connect(self.move_to_targets)
        self.btn_apply_cfg.clicked.connect(self.apply_motor_config)

        self.btn_load_svg.clicked.connect(self.load_svg_file)
        self.btn_start_draw.clicked.connect(self.start_svg_drawing)
        self.btn_pause_draw.clicked.connect(self.toggle_pause_drawing)
        self.btn_stop_draw.clicked.connect(self.cancel_svg_drawing)

        self.spin_svg_scale.valueChanged.connect(self.recompute_svg_workspace)
        self.spin_off_x.valueChanged.connect(self.recompute_svg_workspace)
        self.spin_off_y.valueChanged.connect(self.recompute_svg_workspace)

    def get_signed_z_lift(self):
        mag = abs(self.spin_z_lift_steps.value())
        return mag if self.chk_invert_z.isChecked() else -mag

    def get_ready_pose_steps(self):
        """
        Tư thế sẵn sàng:
        - Nếu đã nạp bản vẽ SVG -> trả về bước của điểm đầu tiên trên path thứ 1.
        - Nếu chưa nạp bản vẽ -> trả về bước của góc ready_pose mặc định.
        """
        if self.canvas_2d.svg_polylines:
            self.kin._last_valid_phi = 0.0
            for poly in self.canvas_2d.svg_polylines:
                for (x, y) in poly:
                    ik = self.kin.inverse_kinematics(x, y, phi_deg=0.0)
                    if ik is not None:
                        s1 = self.kin.deg_to_steps(1, ik[0])
                        s2 = self.kin.deg_to_steps(2, ik[1])
                        s4 = self.kin.deg_to_steps(4, ik[2])
                        return (s1, s2, s4), True
        rj1, rj2, rj3 = self.kin.ready_pose
        return (
            self.kin.deg_to_steps(1, rj1),
            self.kin.deg_to_steps(2, rj2),
            self.kin.deg_to_steps(4, rj3),
        ), False

    def log(self, msg):
        self.txt_log.append(msg)

    def send_cmd(self, cmd):
        if self.serial_worker.send_cmd(cmd):
            if not cmd.startswith("STAT") and not cmd.startswith("JOG"):
                self.log(f"[Pico] TX: #{cmd}")
        else:
            self.simulate_offline_cmd(cmd)

    def send_z_cmd(self, cmd):
        if self.z_worker.send_cmd(cmd):
            if not cmd.startswith("JOG") and not cmd.startswith("STAT"):
                self.log(f"[ArdZ] TX: #{cmd}")
        else:
            if cmd.startswith("MOVA:"):
                parts = cmd[5:].split(",")
                target_z = int(parts[0])
                zero_after = (len(parts) >= 3 and parts[2].strip() == "1")
                self.cur_steps[2] = 0 if zero_after else target_z
                self.update_ui_positions()
                if self.is_drawing_svg and not self.is_paused and not self.waiting_for_manual_pen:
                    QTimer.singleShot(1400, self.process_next_draw_step)
            elif cmd == "ZERO":
                self.cur_steps[2] = 0
                self.update_ui_positions()

    def simulate_offline_cmd(self, cmd):
        if cmd.startswith("MOVA:"):
            parts = cmd[5:].split(",")
            for i in (0, 1, 3):
                if parts[i] not in ("x", "X", ""):
                    self.cur_steps[i] = int(parts[i])
            self.update_ui_positions()
            if self.is_drawing_svg and not self.is_paused and not self.waiting_for_manual_pen:
                QTimer.singleShot(20, self.process_next_draw_step)
        elif cmd.startswith("ZERO:"):
            ax = int(cmd[5:])
            if ax == 0:
                self.cur_steps[0] = self.cur_steps[1] = self.cur_steps[3] = 0
            elif ax != 3:
                self.cur_steps[ax - 1] = 0
            self.update_ui_positions()

    def refresh_ports(self):
        self.cb_port.clear()
        self.cb_z_port.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            label = f"{p.device} - {p.description}"
            self.cb_port.addItem(label, p.device)
            self.cb_z_port.addItem(label, p.device)
        if self.cb_port.count() == 0:
            self.cb_port.addItem("Không tìm thấy cổng COM", "")
            self.cb_z_port.addItem("Không tìm thấy cổng COM", "")

    def toggle_connection(self):
        if self.serial_worker.running:
            self.serial_worker.disconnect_port()
            self.btn_connect.setText("Kết nối Pico")
            self.btn_connect.setStyleSheet("background-color: #2E7D32; color: white; font-weight: bold;")
            self.log("Đã ngắt kết nối Pico 2W.")
        else:
            port = self.cb_port.currentData()
            if not port:
                QMessageBox.warning(self, "Lỗi", "Vui lòng chọn cổng COM của Pico 2W!")
                return
            baud = int(self.cb_baud.currentText())
            ok, msg = self.serial_worker.connect_port(port, baud)
            if ok:
                self.btn_connect.setText("Ngắt Pico")
                self.btn_connect.setStyleSheet("background-color: #C62828; color: white; font-weight: bold;")
                self.log(f"Kết nối Pico 2W tại {port} @ {baud} bps.")
                self.send_cmd("PING")
                self.apply_motor_config()
            else:
                QMessageBox.critical(self, "Lỗi kết nối Pico", msg)

    def toggle_z_connection(self):
        if self.z_worker.running:
            self.z_worker.disconnect_port()
            self.btn_z_connect.setText("Kết nối Arduino Z")
            self.btn_z_connect.setStyleSheet("background-color: #1565C0; color: white; font-weight: bold;")
            self.log("Đã ngắt kết nối Arduino Z.")
        else:
            port = self.cb_z_port.currentData()
            if not port:
                QMessageBox.warning(self, "Lỗi", "Vui lòng chọn cổng COM của Arduino Uno!")
                return
            if self.serial_worker.running and port == self.cb_port.currentData():
                QMessageBox.warning(self, "Trùng cổng COM", "Cổng COM của Arduino Z phải khác cổng COM của Pico 2W!")
                return
            baud = int(self.cb_baud.currentText())
            ok, msg = self.z_worker.connect_port(port, baud)
            if ok:
                self.btn_z_connect.setText("Ngắt Arduino Z")
                self.btn_z_connect.setStyleSheet("background-color: #C62828; color: white; font-weight: bold;")
                self.log(f"Kết nối Arduino Z tại {port} @ {baud} bps (đang chờ 2.2s Arduino khởi động)...")
                QTimer.singleShot(2200, lambda: self.send_z_cmd("PING"))
            else:
                QMessageBox.critical(self, "Lỗi kết nối Arduino Z", msg)

    def enable_all_drivers(self):
        self.send_cmd("EN:1")
        self.send_z_cmd("EN:1")

    def disable_all_drivers(self):
        self.send_cmd("EN:0")
        self.send_z_cmd("EN:0")

    def poll_status(self):
        if self.active_jog is not None:
            return
        if self.serial_worker.running:
            self.serial_worker.send_cmd("STAT")

    def on_status_update(self, pos, moving, enabled, home_mask, estop, homed_mask):
        self.cur_steps[0] = pos[0]
        self.cur_steps[1] = pos[1]
        self.cur_steps[3] = pos[3]
        self.is_moving = bool(moving)
        self.is_enabled = bool(enabled)
        self.is_estop = bool(estop)
        self.update_ui_positions()

        if self.waiting_for_manual_pen:
            status_txt = "⏸ CHỜ HẠ BÚT BẰNG JOG TRỤC 3 -> BẤM 'BÚT ĐÃ SẴN SÀNG'"
            self.lbl_robot_state.setStyleSheet("font-weight: bold; color: #FF6F00;")
        else:
            z_conn_txt = "ArdZ: ON" if self.z_worker.running else "ArdZ: OFF"
            status_txt = f"Pico: {'CHẠY' if moving else 'SẴN SÀNG'} | Driver: {'BẬT' if enabled else 'TẮT'} | {z_conn_txt}"
            if estop:
                status_txt += " | ⚠️ E-STOP!"
                self.lbl_robot_state.setStyleSheet("font-weight: bold; color: #FF1744;")
            elif enabled:
                self.lbl_robot_state.setStyleSheet("font-weight: bold; color: #2E7D32;")
            else:
                self.lbl_robot_state.setStyleSheet("font-weight: bold; color: #EF6C00;")
        self.lbl_robot_state.setText(status_txt)

    def on_z_pos_update(self, z_steps):
        self.cur_steps[2] = z_steps
        self.update_ui_positions()

    def update_ui_positions(self):
        j1 = self.kin.steps_to_deg(1, self.cur_steps[0])
        j2 = self.kin.steps_to_deg(2, self.cur_steps[1])
        z_steps = self.cur_steps[2]
        j3 = self.kin.steps_to_deg(4, self.cur_steps[3])

        self.lbl_pos_val[0].setText(f"{j1:.2f}°")
        self.lbl_pos_val[1].setText(f"{j2:.2f}°")
        self.lbl_pos_val[2].setText(f"{z_steps} (VẼ)" if self.pen_ready_on_paper else f"{z_steps}")
        self.lbl_pos_val[3].setText(f"{j3:.2f}°")

        for i in range(4):
            self.lbl_pos_step[i].setText(str(self.cur_steps[i]))

        self.canvas_2d.update_state(j1, z_steps, j2, j3, pen_down=self.pen_ready_on_paper)

    # =================================================================
    # CÁC CHỨC NĂNG CHUYỂN ĐỘNG & XÁC NHẬN HẠ BÚT BẰNG TAY
    # =================================================================
    def emergency_stop(self):
        self.is_drawing_svg = False
        self.waiting_for_manual_pen = False
        self.pen_ready_on_paper = False
        self.draw_queue.clear()
        self.send_cmd("STOP")
        self.send_z_cmd("EN:0")

    def set_straight_zero(self):
        self.pen_ready_on_paper = False
        self.waiting_for_manual_pen = False
        self.send_cmd("ZERO:0")
        self.send_z_cmd("ZERO")
        self.cur_steps[2] = 0
        self.update_ui_positions()
        self.canvas_2d.clear_traces()
        self.log("Đã gán tư thế duỗi thẳng: J1=0°, J2=180°, J3=180° và Z=0.")

    def pen_up(self):
        self.pen_ready_on_paper = False
        z_up = self.get_signed_z_lift()
        z_spd = self.spin_z_speed.value()
        self.log(f"Nhấc bút {abs(z_up)} xung -> MOVA:{z_up},{z_spd}")
        self.send_z_cmd(f"MOVA:{z_up},{z_spd}")

    def move_to_straight(self):
        self.waiting_for_manual_pen = False
        f = self.slider_speed.value() / 100.0
        self.log("Về Duỗi Thẳng (chỉ đưa J1, J2, J3 về gốc, giữ nguyên Trục Z).")
        self.send_cmd(f"MOVA:0,0,x,0,{f:.2f}")

    def move_to_ready(self):
        """Đưa đầu bút tới ngay vị trí bắt đầu vẽ của bản vẽ SVG (hoặc góc Ready nếu chưa nạp SVG)"""
        self.pen_ready_on_paper = False
        self.waiting_for_manual_pen = True
        f = self.slider_speed.value() / 100.0
        (s1, s2, s4), is_svg_start = self.get_ready_pose_steps()

        if is_svg_start:
            self.log("📍 Đang đưa đầu bút tới ĐIỂM ĐẦU TIÊN của bản vẽ! Hãy JOG Trục 3 hạ bút chạm giấy rồi bấm '3. BÚT ĐÃ SẴN SÀNG'.")
        else:
            self.log("⚠️ Chưa nạp SVG -> Về góc Sẵn Sàng mặc định. Hãy nạp SVG, hạ bút rồi bấm '3. BÚT ĐÃ SẴN SÀNG'.")
        self.send_cmd(f"MOVA:{s1},{s2},x,{s4},{f:.2f}")

    def confirm_pen_ready(self):
        """Xác nhận bút đã hạ chạm giấy tại điểm đầu tiên -> Gán Z=0 và bắt đầu vẽ ngay"""
        self.stop_jog()
        self.send_z_cmd("ZERO")
        self.cur_steps[2] = 0
        self.pen_ready_on_paper = True
        self.waiting_for_manual_pen = False
        self.is_moving = False
        self.update_ui_positions()
        self.log("✅ Đã gán Z=0 tại điểm xuất phát -> Bắt đầu vẽ toàn bộ bản vẽ!")

        if self.is_drawing_svg and self.is_paused:
            self.is_paused = False
            QTimer.singleShot(50, self.process_next_draw_step)
            return

        if self.canvas_2d.svg_polylines:
            QTimer.singleShot(50, self.start_svg_drawing)
        else:
            QMessageBox.information(self, "Thông báo", "Đã gán Z=0. Hãy nạp file SVG rồi bấm 'Bắt đầu vẽ'!")

    def move_to_targets(self):
        j2_t = self.spin_target[1].value()
        j3_t = self.spin_target[3].value()
        if not self.kin.is_joint_valid(j2_t, j3_t):
            QMessageBox.warning(
                self, "Góc chết",
                "Góc J2 hoặc J3 rơi vào vùng góc chết (302.5° qua 360° đến 52.5°).\nChỉ nhập trong khoảng [52.5°, 302.5°]!"
            )
            return
        f = self.slider_speed.value() / 100.0
        s1 = self.kin.deg_to_steps(1, self.spin_target[0].value())
        s2 = self.kin.deg_to_steps(2, j2_t)
        s3 = int(round(self.spin_target[2].value()))
        s4 = self.kin.deg_to_steps(4, j3_t)
        z_spd = self.spin_z_speed.value()

        self.send_z_cmd(f"MOVA:{s3},{z_spd}")
        self.send_cmd(f"MOVA:{s1},{s2},x,{s4},{f:.2f}")

    def start_jog(self, axis, direction):
        if axis == 3:
            self.pen_ready_on_paper = False
            z_spd = self.spin_z_speed.value()
            self.active_jog = (3, direction, z_spd)
            if self.z_worker.running:
                self.z_worker.send_cmd(f"JOG:{direction},{z_spd}")
            else:
                self.step_offline_jog()
                self.offline_jog_timer.start(80)
            return

        if self.is_moving:
            self.send_cmd("HALT")
            return
        f = self.slider_speed.value() / 100.0
        self.active_jog = (axis, direction, f)
        if self.serial_worker.running:
            self.serial_worker.send_cmd(f"JOG:{axis},{direction},{f:.2f}")
        else:
            self.step_offline_jog()
            self.offline_jog_timer.start(80)

    def stop_jog(self):
        if self.active_jog:
            axis = self.active_jog[0]
            self.offline_jog_timer.stop()
            self.active_jog = None
            if axis == 3:
                if self.z_worker.running:
                    self.z_worker.send_cmd(f"JOG:0,{self.spin_z_speed.value()}")
            else:
                if self.serial_worker.running:
                    self.serial_worker.send_cmd(f"JOG:{axis},0,0.1")

    def step_offline_jog(self):
        if not self.active_jog:
            return
        axis, direction, val = self.active_jog
        if axis == 3:
            self.cur_steps[2] += direction * max(1, int(val * 0.05))
        else:
            self.cur_steps[axis - 1] += direction * 40
        self.update_ui_positions()

    def apply_motor_config(self):
        vmax = self.spin_vmax.value()
        acc = self.spin_acc.value()
        for ax in range(1, 5):
            if ax == 1:
                self.send_cmd(f"CFG:{ax},{vmax},{acc},1")
            elif ax == 2:
                self.send_cmd(f"CFG:{ax},{int(vmax * 1.6)},{int(acc * 1.6)},0")
            elif ax == 4:
                self.send_cmd(f"CFG:{ax},{vmax},{acc},1")

    def sync_kinematics_to_ui(self):
        self.spin_L1.setValue(self.kin.L1)
        self.spin_L2.setValue(self.kin.L2)
        self.spin_L3.setValue(self.kin.L3)
        self.spin_z_lift_steps.setValue(abs(self.kin.z_lift_steps))

    def update_kinematics_from_ui(self):
        self.kin.L1 = self.spin_L1.value()
        self.kin.L2 = self.spin_L2.value()
        self.kin.L3 = self.spin_L3.value()
        self.kin.z_lift_steps = abs(self.spin_z_lift_steps.value())
        self.kin.z_lower_steps = self.kin.z_lift_steps + 400
        self.canvas_2d.update()
        self.recompute_svg_workspace()
        self.log(
            f"Đã cập nhật cơ khí & Trục Z (nhấc {self.kin.z_lift_steps} xung, "
            f"hạ {self.kin.z_lower_steps} xung)."
        )

    # =================================================================
    # XỬ LÝ FILE SVG INKSCAPE & CHU TRÌNH NÂNG / HẠ BÚT
    # =================================================================
    def load_svg_file(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Chọn file bản vẽ SVG (Inkscape)", "", "SVG Files (*.svg)"
        )
        if not file_path:
            return
        try:
            paths, _ = svg2paths(file_path)
            step_mm = self.spin_step_mm.value()
            raw_polys = []

            for path in paths:
                subpaths = path.continuous_subpaths()
                for sub in subpaths:
                    length = sub.length()
                    if length < 0.5:
                        continue
                    num_samples = max(2, int(math.ceil(length / step_mm)))
                    poly = []
                    for k in range(num_samples + 1):
                        pt = sub.point(k / num_samples)
                        poly.append((pt.real, -pt.imag))
                    if len(poly) >= 3:
                        raw_polys.append(poly)

            if not raw_polys:
                QMessageBox.warning(self, "Cảnh báo", "Không tìm thấy đường nét (Path) hợp lệ trong file SVG!")
                return

            min_x = min(pt[0] for poly in raw_polys for pt in poly)
            min_y = min(pt[1] for poly in raw_polys for pt in poly)
            self.raw_svg_paths = [[(x - min_x, y - min_y) for (x, y) in poly] for poly in raw_polys]

            self.svg_preview.set_polylines(
                [[(x, -y) for (x, y) in poly] for poly in self.raw_svg_paths]
            )
            self.recompute_svg_workspace()
            self.log(f"Đã nạp SVG: {len(self.raw_svg_paths)} nét vẽ liên tục. Bấm '2. Về tư thế Sẵn Sàng' để tới điểm đặt bút đầu tiên.")
        except Exception as e:
            QMessageBox.critical(self, "Lỗi đọc file SVG", str(e))

    def recompute_svg_workspace(self):
        if not self.raw_svg_paths:
            return
        sc = self.spin_svg_scale.value()
        ox = self.spin_off_x.value()
        oy = self.spin_off_y.value()

        scaled_polys = []
        for poly in self.raw_svg_paths:
            scaled_polys.append([(ox + x * sc, oy + y * sc) for (x, y) in poly])

        self.canvas_2d.svg_polylines = scaled_polys
        self.canvas_2d.update()

    def build_draw_queue(self):
        self.draw_queue.clear()
        f_draw = self.slider_speed.value() / 100.0
        f_travel = min(1.0, f_draw * 1.2)
        z_up = self.get_signed_z_lift()
        extra_down = self.kin.z_lower_steps - abs(z_up)
        z_down_target = extra_down if z_up < 0 else -extra_down
        z_spd = self.spin_z_speed.value()
        self.kin._last_valid_phi = 0.0

        # Lấy góc khớp hiện tại làm mốc để IK chọn đường đi ít xoay nhất
        cur_j = (
            self.kin.steps_to_deg(1, self.cur_steps[0]),
            self.kin.steps_to_deg(2, self.cur_steps[1]),
            self.kin.steps_to_deg(4, self.cur_steps[3]),
        )

        valid_polys = []
        skipped_points = 0
        for poly in self.canvas_2d.svg_polylines:
            pts_ik = []
            for (x, y) in poly:
                ik = self.kin.inverse_kinematics(x, y, phi_deg=0.0, current_joints=cur_j)
                if ik is None:
                    skipped_points += 1
                else:
                    pts_ik.append(ik)
                    cur_j = ik
            if len(pts_ik) >= 3:
                valid_polys.append(pts_ik)

        if not valid_polys:
            return

        # Bù độ rơ dây đai GT2 (số xung bù thêm mỗi khi khớp đảo chiều quay)
        # Nếu thấy nét nối vẫn hơi hụt -> tăng lên 18..25; nếu hơi lố -> giảm về 6..10
        BACKLASH_STEPS = [12, 12, 10]  # [J1, J2, J3]
        last_dir = [0, 0, 0]
        last_raw = [self.cur_steps[0], self.cur_steps[1], self.cur_steps[3]]

        def apply_backlash(raw_s1, raw_s2, raw_s4):
            raws = [raw_s1, raw_s2, raw_s4]
            comp = []
            for idx in range(3):
                diff = raws[idx] - last_raw[idx]
                if diff > 0:
                    last_dir[idx] = 1
                elif diff < 0:
                    last_dir[idx] = -1
                last_raw[idx] = raws[idx]
                comp.append(raws[idx] + last_dir[idx] * BACKLASH_STEPS[idx])
            return comp[0], comp[1], comp[2]

        REORIENT_JUMP_DEG = 10.0
        reorient_count = 0

        for stroke_idx, pts_ik in enumerate(valid_polys):
            r1_0 = self.kin.deg_to_steps(1, pts_ik[0][0])
            r2_0 = self.kin.deg_to_steps(2, pts_ik[0][1])
            r4_0 = self.kin.deg_to_steps(4, pts_ik[0][2])
            s1_0, s2_0, s4_0 = apply_backlash(r1_0, r2_0, r4_0)

            if stroke_idx == 0:
                if (abs(r1_0 - self.cur_steps[0]) > 2) or (abs(r2_0 - self.cur_steps[1]) > 2) or (abs(r4_0 - self.cur_steps[3]) > 2):
                    self.draw_queue.append(("PICO_CMD", f"MOVA:{s1_0},{s2_0},x,{s4_0},{f_draw:.2f}"))
            else:
                self.draw_queue.append(("Z_PEN_UP", f"MOVA:{z_up},{z_spd}"))
                self.draw_queue.append(("PICO_CMD", f"MOVA:{s1_0},{s2_0},x,{s4_0},{f_travel:.2f}"))
                self.draw_queue.append(("Z_PEN_DOWN", f"MOVA:{z_down_target},{z_spd},1"))

            prev_ik = pts_ik[0]
            for ik in pts_ik[1:]:
                r1 = self.kin.deg_to_steps(1, ik[0])
                r2 = self.kin.deg_to_steps(2, ik[1])
                r4 = self.kin.deg_to_steps(4, ik[2])
                s1, s2, s4 = apply_backlash(r1, r2, r4)

                dj1 = abs(ik[0] - prev_ik[0])
                if dj1 > 180.0:
                    dj1 = 360.0 - dj1
                dj2 = abs(ik[1] - prev_ik[1])
                dj3 = abs(ik[2] - prev_ik[2])

                if max(dj1, dj2, dj3) > REORIENT_JUMP_DEG:
                    reorient_count += 1
                    self.draw_queue.append(("Z_PEN_UP", f"MOVA:{z_up},{z_spd}"))
                    self.draw_queue.append(("PICO_CMD", f"MOVA:{s1},{s2},x,{s4},{f_travel:.2f}"))
                    self.draw_queue.append(("Z_PEN_DOWN", f"MOVA:{z_down_target},{z_spd},1"))
                else:
                    self.draw_queue.append(("PICO_CMD", f"MOVA:{s1},{s2},x,{s4},{f_draw:.2f}"))

                prev_ik = ik

        (ready_s1, ready_s2, ready_s4), _ = self.get_ready_pose_steps()
        self.draw_queue.append(("Z_PEN_UP", f"MOVA:{z_up},{z_spd}"))
        self.draw_queue.append(("PICO_CMD", f"MOVA:{ready_s1},{ready_s2},x,{ready_s4},{f_travel:.2f}"))

        if reorient_count > 0:
            self.log(f"🔄 Tự động chèn {reorient_count} nhịp nhấc bút để tránh góc chết / đổi hướng tay máy.")
        if skipped_points > 0:
            self.log(f"⚠️ Cảnh báo: Có {skipped_points} điểm nằm ngoài tầm với hoặc chạm góc chết (302.5°..52.5°) đã bị bỏ qua.")
        self.total_queue_len = len(self.draw_queue)
        self.progress_bar.setMaximum(max(1, self.total_queue_len))
        self.progress_bar.setValue(0)

    def start_svg_drawing(self):
        try:
            if not self.canvas_2d.svg_polylines:
                QMessageBox.warning(self, "Chưa có bản vẽ", "Vui lòng nạp file SVG trước khi vẽ!")
                return

            if self.is_drawing_svg and self.is_paused and not self.waiting_for_manual_pen:
                self.is_paused = False
                self.log("Tiếp tục quá trình vẽ...")
                self.process_next_draw_step()
                return

            if not self.pen_ready_on_paper:
                self.send_z_cmd("ZERO")
                self.cur_steps[2] = 0
                self.pen_ready_on_paper = True
                self.update_ui_positions()

            self.canvas_2d.clear_traces()
            self.build_draw_queue()
            if not self.draw_queue:
                return

            self.is_drawing_svg = True
            self.is_paused = False
            self.waiting_for_manual_pen = False
            self.log(
                f"Bắt đầu vẽ ({self.total_queue_len} bước | Nhấc {abs(self.get_signed_z_lift())} xung, "
                f"hạ {self.kin.z_lower_steps} xung)..."
            )
            self.process_next_draw_step()
        except Exception as e:
            self.is_drawing_svg = False
            self.log(f"❌ Lỗi khởi chạy vẽ: {e}")
            QMessageBox.critical(self, "Lỗi", f"Không thể bắt đầu vẽ:\n{e}")

    def toggle_pause_drawing(self):
        if self.is_drawing_svg and not self.waiting_for_manual_pen:
            self.is_paused = not self.is_paused
            if self.is_paused:
                self.send_cmd("HALT")
                self.log("Đã tạm dừng vẽ (HALT).")
            else:
                self.log("Tiếp tục vẽ...")
                self.process_next_draw_step()

    def cancel_svg_drawing(self):
        self.is_drawing_svg = False
        self.is_paused = False
        self.waiting_for_manual_pen = False
        self.draw_queue.clear()
        self.send_cmd("HALT")
        self.progress_bar.setValue(0)
        self.log("Đã hủy chu trình vẽ.")

    def process_next_draw_step(self):
        if not self.is_drawing_svg or self.is_paused or self.waiting_for_manual_pen:
            return
        if not self.draw_queue:
            self.is_drawing_svg = False
            self.pen_ready_on_paper = False
            self.progress_bar.setValue(self.total_queue_len)
            self.log("✅ HOÀN THÀNH BẢN VẼ & ĐÃ VỀ LẠI VỊ TRÍ SẴN SÀNG!")
            return

        action, payload = self.draw_queue.pop(0)
        done_count = self.total_queue_len - len(self.draw_queue)
        self.progress_bar.setValue(done_count)

        if action == "Z_PEN_UP":
            self.pen_ready_on_paper = False
            self.update_ui_positions()
            self.log(f"⬆️ Nhấc bút {abs(self.get_signed_z_lift())} xung: #{payload}")
            self.send_z_cmd(payload)
            return

        if action == "Z_PEN_DOWN":
            self.pen_ready_on_paper = True
            self.update_ui_positions()
            self.log(f"⬇️ Hạ bút {self.kin.z_lower_steps} xung: #{payload}")
            self.send_z_cmd(payload)
            return

        if action == "PICO_CMD":
            self.send_cmd(payload)

    def on_robot_event(self, evt):
        if evt == "DONE" and self.is_drawing_svg and not self.is_paused and not self.waiting_for_manual_pen:
            self.process_next_draw_step()

    def on_z_event(self, evt):
        if evt == "Z_DONE" and self.is_drawing_svg and not self.is_paused and not self.waiting_for_manual_pen:
            self.process_next_draw_step()

    def on_robot_error(self, err):
        self.waiting_for_manual_pen = False
        self.log(f"❌ LỖI HỆ THỐNG: {err}")
        if self.is_drawing_svg:
            self.is_drawing_svg = False
            self.log("Đã tự động dừng vẽ do phát hiện lỗi từ bộ điều khiển.")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = ScaraMainWindow()
    window.show()
    sys.exit(app.exec())