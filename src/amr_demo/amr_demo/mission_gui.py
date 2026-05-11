#!/usr/bin/env python3

import os
import re
import subprocess
import sys
import tempfile

from PyQt6.QtCore import QThread, pyqtSignal, Qt
from PyQt6.QtGui import QColor, QImage, QPainter, QPen
from PyQt6.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QPushButton,
    QComboBox,
    QTextEdit,
    QVBoxLayout,
    QHBoxLayout,
    QFrame,
    QSizePolicy,
)


MISSION_TYPES = {
    "Grocery delivery": "grocery",
    "Food delivery": "food",
    "Fire emergency": "fire",
    "Medical help": "medical",
}

HOME_LABEL_TO_ID = {
    "House 1": "HOUSE_1",
    "House 2": "HOUSE_2",
    "House 3": "HOUSE_3",
    "House 4": "HOUSE_4",
    "House 5": "HOUSE_5",
}

PICKUP_LABELS = {
    "grocery": "Supermarket",
    "food": "Restaurant",
    "fire": "Fire Station",
    "medical": "Pharmacy",
}


class MissionWorker(QThread):
    line = pyqtSignal(str)
    finished_ok = pyqtSignal(bool)

    def __init__(self, mission_type, home):
        super().__init__()
        self.mission_type = mission_type
        self.home = home

    def run(self):
        env = os.environ.copy()
        env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"

        cmd = [
            "ros2", "run", "amr_demo", "run_mission",
            "--type", self.mission_type,
            "--home", self.home,
        ]

        self.line.emit(f"$ {' '.join(cmd)}\n")

        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env=env,
        )

        for output in process.stdout:
            self.line.emit(output.rstrip())

        code = process.wait()
        self.finished_ok.emit(code == 0)


class MissionGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.worker = None
        self.setWindowTitle("")
        self.setMinimumSize(860, 680)
        self.build_ui()

    def _combo_arrow_icon_path(self):
        icon_path = os.path.join(tempfile.gettempdir(), "amr_demo_combo_down_arrow.png")
        if not os.path.exists(icon_path):
            image = QImage(16, 16, QImage.Format.Format_ARGB32)
            image.fill(Qt.GlobalColor.transparent)
            painter = QPainter(image)
            painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
            pen = QPen(QColor("#FFFFFF"))
            pen.setWidth(2)
            pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
            painter.setPen(pen)
            painter.drawLine(4, 6, 8, 10)
            painter.drawLine(8, 10, 12, 6)
            painter.end()
            image.save(icon_path, "PNG")
        return icon_path.replace("\\", "/")

    def build_ui(self):
        combo_arrow = self._combo_arrow_icon_path()
        stylesheet = """
            QWidget {
                background-color: #DCEBFF;
                color: #0F172A;
                font-family: Arial, Helvetica, sans-serif;
                font-size: 16px;
            }

            QFrame#HeaderBar {
                background-color: #0A4EA3;
                border: none;
            }

            QLabel#HeaderInstitution {
                background-color: transparent;
                color: #F8FBFF;
                font-size: 15px;
                font-weight: 800;
            }

            QLabel#HeaderDetails {
                background-color: transparent;
                color: #D9E9FF;
                font-size: 12px;
                font-weight: 600;
            }

            QFrame#FooterBar {
                background-color: #F6F9FC;
                border-top: 1px solid #BCCDE4;
            }

            QLabel#FooterText {
                background-color: transparent;
                color: #5B6B82;
                font-size: 13px;
                font-weight: 600;
            }

            QLabel#Title {
                font-size: 36px;
                font-weight: 800;
                color: #0B2D5B;
                background-color: transparent;
            }

            QLabel#Subtitle {
                font-size: 18px;
                color: #4F6D98;
                background-color: transparent;
            }

            QLabel#SectionTitle {
                font-size: 20px;
                font-weight: 700;
                color: #0F2E59;
                background-color: transparent;
            }

            QLabel#FieldLabel {
                font-size: 17px;
                font-weight: 700;
                color: #0F2E59;
                background-color: transparent;
            }

            QLabel#HelperText {
                font-size: 14px;
                color: #6B7D93;
                background-color: transparent;
            }

            QLabel#PreviewRoute {
                font-size: 19px;
                font-weight: 700;
                color: #0A4EA3;
                background-color: transparent;
            }

            QFrame#Card {
                background-color: #F9FBFE;
                border: 1px solid #C8D4E2;
                border-radius: 18px;
            }

            QFrame#StatusCard {
                background-color: #F9FBFE;
                border: 1px solid #C8D4E2;
                border-radius: 18px;
            }

            QLabel#StatusBadge {
                background-color: #F4F8FC;
                color: #1F2937;
                border: 1px solid #C8D4E2;
                border-left: 3px solid #94A3B8;
                border-radius: 6px;
                padding: 4px 8px;
                font-size: 14px;
                font-weight: 700;
                min-width: 112px;
            }

            QLabel#StatusBadge[status="ready"] {
                background-color: #ECFDF5;
                color: #047857;
                border: 1px solid #A7F3D0;
                border-left: 3px solid #10B981;
            }

            QLabel#StatusBadge[status="running"] {
                background-color: #FFF8DC;
                color: #B45309;
                border: 1px solid #FCD34D;
                border-left: 3px solid #EAB308;
            }

            QLabel#StatusBadge[status="success"] {
                background-color: #ECFDF5;
                color: #047857;
                border: 1px solid #A7F3D0;
                border-left: 3px solid #10B981;
            }

            QLabel#StatusBadge[status="error"] {
                background-color: #FEF2F2;
                color: #B91C1C;
                border: 1px solid #FECACA;
                border-left: 3px solid #EF4444;
            }

            QComboBox {
                background-color: #FFFFFF;
                border: 1px solid #C8D4E2;
                border-radius: 12px;
                padding: 13px 48px 13px 16px;
                color: #102A43;
                min-height: 30px;
                font-size: 16px;
            }

            QComboBox:hover {
                border: 1px solid #0A4EA3;
            }

            QComboBox:focus {
                border: 2px solid #0A4EA3;
            }

            QComboBox::drop-down {
                width: 46px;
                border-left: 1px solid #C8D4E2;
                background-color: #0A4EA3;
                border-top-right-radius: 12px;
                border-bottom-right-radius: 12px;
            }

            QComboBox::down-arrow {
                image: url("__COMBO_ARROW__");
                width: 14px;
                height: 14px;
                margin: 0px;
            }

            QComboBox QAbstractItemView {
                background-color: #FFFFFF;
                color: #102A43;
                selection-background-color: #0A4EA3;
                selection-color: #FFFFFF;
                border: 1px solid #C8D4E2;
                padding: 8px;
                outline: 0;
            }

            QPushButton#StartButton {
                background-color: qlineargradient(
                    x1:0, y1:0, x2:1, y2:0,
                    stop:0 #FFE26A,
                    stop:1 #FACC15
                );
                color: #1F2937;
                border: 2px solid #D4A800;
                border-radius: 16px;
                padding: 18px 38px;
                font-size: 20px;
                font-weight: 800;
                min-height: 32px;
                letter-spacing: 0px;
            }

            QPushButton#StartButton:hover {
                background-color: qlineargradient(
                    x1:0, y1:0, x2:1, y2:0,
                    stop:0 #FFED8A,
                    stop:1 #FDE047
                );
                border: 2px solid #B88D00;
            }

            QPushButton#StartButton:pressed {
                background-color: #EAB308;
                border: 2px solid #987300;
                padding-top: 19px;
                padding-bottom: 17px;
            }

            QPushButton#StartButton:disabled {
                background-color: #CDD7E3;
                color: #6B7D93;
                border: 2px solid #CDD7E3;
            }

            QTextEdit {
                background-color: #FFFFFF;
                border: 1px solid #C8D4E2;
                border-radius: 14px;
                padding: 14px;
                color: #102A43;
                font-family: Consolas, "Courier New", monospace;
                font-size: 13px;
                selection-background-color: #0A4EA3;
            }
        """
        self.setStyleSheet(stylesheet.replace("__COMBO_ARROW__", combo_arrow))

        root = QVBoxLayout()
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        header_bar = QFrame()
        header_bar.setObjectName("HeaderBar")
        header_layout = QHBoxLayout()
        header_layout.setContentsMargins(28, 10, 28, 10)
        header_layout.setSpacing(20)

        institution = QLabel("American University of Beirut (AUB)")
        institution.setObjectName("HeaderInstitution")
        institution.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        institution.setWordWrap(True)

        project_details = QLabel("MSFEA  |  MECH 650 / EECE 698 - Autonomous Mobile Robotics")
        project_details.setObjectName("HeaderDetails")
        project_details.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        project_details.setWordWrap(True)

        header_layout.addWidget(institution, stretch=1)
        header_layout.addWidget(project_details, stretch=1)
        header_bar.setLayout(header_layout)

        content = QVBoxLayout()
        content.setContentsMargins(42, 26, 42, 20)
        content.setSpacing(16)

        title = QLabel("Autonomous Service Robot")
        title.setObjectName("Title")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)

        subtitle = QLabel("Mission Control Interface")
        subtitle.setObjectName("Subtitle")
        subtitle.setAlignment(Qt.AlignmentFlag.AlignCenter)

        header = QVBoxLayout()
        header.setSpacing(4)
        header.addWidget(title)
        header.addWidget(subtitle)

        controls_card = QFrame()
        controls_card.setObjectName("Card")
        controls_card.setMinimumWidth(760)
        controls_card.setMaximumWidth(1080)
        controls_card.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        controls_layout = QVBoxLayout()
        controls_layout.setContentsMargins(28, 26, 28, 28)
        controls_layout.setSpacing(20)

        row = QHBoxLayout()
        row.setSpacing(22)

        left = QVBoxLayout()
        left.setSpacing(10)
        mission_label = QLabel("Mission Type")
        mission_label.setObjectName("FieldLabel")
        self.mission_box = QComboBox()
        self.mission_box.addItems(MISSION_TYPES.keys())
        self.mission_box.setPlaceholderText("Select mission type")
        self.mission_box.setCurrentIndex(-1)
        self.mission_box.currentTextChanged.connect(self.update_mission_preview)
        self.mission_box.currentIndexChanged.connect(self.update_start_button_state)
        left.addWidget(mission_label)
        left.addWidget(self.mission_box)

        right = QVBoxLayout()
        right.setSpacing(10)
        home_label = QLabel("Target Home")
        home_label.setObjectName("FieldLabel")
        self.home_box = QComboBox()
        self.home_box.addItems(HOME_LABEL_TO_ID.keys())
        self.home_box.setPlaceholderText("Select target home")
        self.home_box.setCurrentIndex(-1)
        self.home_box.currentTextChanged.connect(self.update_mission_preview)
        self.home_box.currentIndexChanged.connect(self.update_start_button_state)
        right.addWidget(home_label)
        right.addWidget(self.home_box)

        row.addLayout(left)
        row.addLayout(right)

        self.start_button = QPushButton("Start Mission")
        self.start_button.setObjectName("StartButton")
        self.start_button.setCursor(Qt.CursorShape.PointingHandCursor)
        self.start_button.setFixedWidth(300)
        self.start_button.clicked.connect(self.start_mission)

        button_row = QHBoxLayout()
        button_row.addStretch()
        button_row.addWidget(self.start_button)
        button_row.addStretch()

        controls_layout.addLayout(row)
        controls_layout.addLayout(button_row)
        controls_card.setLayout(controls_layout)

        preview_card = QFrame()
        preview_card.setObjectName("Card")
        preview_card.setMinimumWidth(760)
        preview_card.setMaximumWidth(1080)
        preview_card.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        preview_layout = QVBoxLayout()
        preview_layout.setContentsMargins(24, 20, 24, 22)
        preview_layout.setSpacing(8)

        preview_title = QLabel("Mission Preview")
        preview_title.setObjectName("SectionTitle")
        preview_helper = QLabel("Planned route")
        preview_helper.setObjectName("HelperText")
        self.preview_route = QLabel()
        self.preview_route.setObjectName("PreviewRoute")
        self.preview_route.setWordWrap(True)

        preview_layout.addWidget(preview_title)
        preview_layout.addWidget(preview_helper)
        preview_layout.addWidget(self.preview_route)
        preview_card.setLayout(preview_layout)

        status_card = QFrame()
        status_card.setObjectName("StatusCard")
        status_card.setMinimumWidth(760)
        status_card.setMaximumWidth(1080)
        status_card.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        status_layout = QVBoxLayout()
        status_layout.setContentsMargins(24, 20, 24, 24)
        status_layout.setSpacing(12)

        status_header = QHBoxLayout()
        status_title = QLabel("Mission Status")
        status_title.setObjectName("SectionTitle")
        self.status = QLabel("Ready")
        self.status.setObjectName("StatusBadge")
        self.status.setProperty("status", "ready")
        self.status.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)

        status_header.addWidget(status_title)
        status_header.addStretch()
        status_header.addWidget(self.status)

        self.output = QTextEdit()
        self.output.setReadOnly(True)
        self.output.setMinimumHeight(150)
        self.output.setPlaceholderText("Mission logs will appear here after starting a mission.")

        status_layout.addLayout(status_header)
        status_layout.addWidget(self.output)
        status_card.setLayout(status_layout)

        footer_bar = QFrame()
        footer_bar.setObjectName("FooterBar")
        footer_layout = QHBoxLayout()
        footer_layout.setContentsMargins(32, 10, 32, 10)

        footer_left = QLabel("Final Course Project  |  Spring 2025-2026")
        footer_left.setObjectName("FooterText")
        footer_left.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        footer_left.setWordWrap(True)

        footer_right = QLabel("Team: Gabriel Menassa  |  Larissa Azar")
        footer_right.setObjectName("FooterText")
        footer_right.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        footer_right.setWordWrap(True)

        footer_layout.addWidget(footer_left, stretch=1)
        footer_layout.addWidget(footer_right, stretch=1)
        footer_bar.setLayout(footer_layout)

        content.addLayout(header)
        content.addSpacing(4)
        content.addWidget(controls_card, alignment=Qt.AlignmentFlag.AlignHCenter)
        content.addWidget(preview_card, alignment=Qt.AlignmentFlag.AlignHCenter)
        content.addWidget(status_card, stretch=1, alignment=Qt.AlignmentFlag.AlignHCenter)

        root.addWidget(header_bar)
        root.addLayout(content, stretch=1)
        root.addWidget(footer_bar)

        self.setLayout(root)
        self.set_status("Ready", "ready")
        self.update_mission_preview()
        self.update_start_button_state()

    def update_mission_preview(self):
        if self.mission_box.currentIndex() < 0 or self.home_box.currentIndex() < 0:
            self.preview_route.setText("Select mission type and target home to preview route")
            return

        mission_label = self.mission_box.currentText()
        mission_type = MISSION_TYPES[mission_label]
        pickup = PICKUP_LABELS[mission_type]
        home = self.home_box.currentText()
        self.preview_route.setText(f"Docking station → {pickup} → {home} → Docking station")

    def format_ui_text(self, text):
        text = re.sub(r"\bDOCK\b", "Docking station", text)
        text = re.sub(r"\bDock\b", "Docking station", text)
        return text

    def append_output_line(self, text):
        self.output.append(self.format_ui_text(text))

    def update_start_button_state(self):
        selections_ready = self.mission_box.currentIndex() >= 0 and self.home_box.currentIndex() >= 0
        self.start_button.setDisabled((not selections_ready) or (self.worker is not None))

    def set_status(self, text, state="ready"):
        status_icons = {
            "ready": "●",
            "running": "↻",
            "success": "✓",
            "error": "⚠",
        }
        icon = status_icons.get(state, "●")
        self.status.setText(f"{icon}  {text}")
        self.status.setProperty("status", state)
        self.status.style().unpolish(self.status)
        self.status.style().polish(self.status)

    def start_mission(self):
        if self.worker is not None:
            return

        if self.mission_box.currentIndex() < 0 or self.home_box.currentIndex() < 0:
            self.set_status("Selection required", "error")
            self.output.append("Please select both mission type and target home before starting.")
            return

        mission_label = self.mission_box.currentText()
        mission_type = MISSION_TYPES[mission_label]
        home_label = self.home_box.currentText()
        home = HOME_LABEL_TO_ID[home_label]

        self.output.clear()
        self.set_status("Mission sent", "running")
        self.output.append(f"Mission sent: {mission_label} to {home_label}")
        self.output.append(f"Route: {self.preview_route.text()}")
        self.output.append("Navigating...")
        self.set_status("Navigating", "running")
        self.start_button.setText("Mission Running...")
        self.start_button.setDisabled(True)

        self.worker = MissionWorker(mission_type, home)
        self.worker.line.connect(self.append_output_line)
        self.worker.finished_ok.connect(self.mission_finished)
        self.worker.start()

    def mission_finished(self, ok):
        if ok:
            self.set_status("Completed", "success")
        else:
            self.set_status("Error", "error")

        self.start_button.setText("Start Mission")
        self.worker = None
        self.update_start_button_state()


def main():
    app = QApplication(sys.argv)
    window = MissionGUI()
    window.showMaximized()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
