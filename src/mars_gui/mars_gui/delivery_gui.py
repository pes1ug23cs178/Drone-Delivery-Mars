#!/usr/bin/env python3

import sys
from typing import Callable, Dict, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import QBrush, QColor, QFont, QPainter, QPen
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QWidget
from rclpy.node import Node
from std_msgs.msg import String


class DeliveryMapWidget(QWidget):
    def __init__(self, on_house_selected: Callable[[str], None]) -> None:
        super().__init__()
        self.on_house_selected = on_house_selected

        self.setFixedSize(600, 600)

        self.world_limit = 25.0
        self.padding = 45.0

        self.warehouse_name = 'Warehouse'
        self.warehouse_pos = (0.0, 0.0)

        self.houses: Dict[str, Tuple[float, float]] = {
            'House1': (10.0, 10.0),
            'House2': (10.0, -10.0),
            'House3': (-10.0, 10.0),
            'House4': (-10.0, -10.0),
            'House5': (20.0, 0.0),
            'House6': (-20.0, 0.0),
        }

        self.house_colors: Dict[str, QColor] = {
            'House1': QColor(214, 79, 79),
            'House2': QColor(76, 170, 85),
            'House3': QColor(72, 112, 205),
            'House4': QColor(226, 184, 86),
            'House5': QColor(59, 187, 194),
            'House6': QColor(196, 108, 177),
        }

        self.selected_house: Optional[str] = None
        self.drone_world_position = (0.0, 0.0)

    def world_to_canvas(self, x: float, y: float) -> QPointF:
        draw_size = self.width() - 2.0 * self.padding
        x_norm = (x + self.world_limit) / (2.0 * self.world_limit)
        y_norm = (y + self.world_limit) / (2.0 * self.world_limit)

        px = self.padding + (x_norm * draw_size)
        py = self.height() - (self.padding + (y_norm * draw_size))
        return QPointF(px, py)

    def rect_for_world_object(self, x: float, y: float, width_px: float, height_px: float) -> QRectF:
        center = self.world_to_canvas(x, y)
        return QRectF(center.x() - width_px / 2.0, center.y() - height_px / 2.0, width_px, height_px)

    def set_drone_position(self, x: float, y: float) -> None:
        self.drone_world_position = (x, y)
        self.update()

    def set_selected_house(self, house_name: Optional[str]) -> None:
        self.selected_house = house_name
        self.update()

    def mousePressEvent(self, event) -> None:  # pylint: disable=invalid-name
        if event.button() != Qt.LeftButton:
            return

        click_point = event.pos()
        for house_name, (hx, hy) in self.houses.items():
            house_rect = self.rect_for_world_object(hx, hy, 34.0, 34.0)
            if house_rect.contains(click_point):
                self.set_selected_house(house_name)
                self.on_house_selected(house_name)
                return

    def paintEvent(self, event) -> None:  # pylint: disable=invalid-name, unused-argument
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        painter.fillRect(self.rect(), QColor(238, 242, 246))

        map_rect = QRectF(self.padding, self.padding, self.width() - 2.0 * self.padding, self.height() - 2.0 * self.padding)
        painter.setBrush(QBrush(QColor(220, 227, 232)))
        painter.setPen(QPen(QColor(180, 190, 200), 1))
        painter.drawRect(map_rect)

        painter.setPen(QPen(QColor(188, 198, 206), 1, Qt.DashLine))
        for i in range(1, 10):
            ratio = i / 10.0
            x = self.padding + ratio * map_rect.width()
            y = self.padding + ratio * map_rect.height()
            painter.drawLine(int(x), int(self.padding), int(x), int(self.height() - self.padding))
            painter.drawLine(int(self.padding), int(y), int(self.width() - self.padding), int(y))

        warehouse_rect = self.rect_for_world_object(self.warehouse_pos[0], self.warehouse_pos[1], 56.0, 56.0)
        painter.setBrush(QBrush(QColor(128, 128, 128)))
        painter.setPen(QPen(QColor(70, 70, 70), 2))
        painter.drawRect(warehouse_rect)

        painter.setPen(QPen(QColor(15, 15, 15), 1))
        painter.setFont(QFont('DejaVu Sans', 9, QFont.Bold))
        painter.drawText(warehouse_rect.adjusted(-8, -20, 8, -6), Qt.AlignCenter, self.warehouse_name)

        for house_name, (hx, hy) in self.houses.items():
            house_rect = self.rect_for_world_object(hx, hy, 34.0, 34.0)

            painter.setBrush(QBrush(self.house_colors[house_name]))
            painter.setPen(QPen(QColor(60, 60, 60), 2))
            painter.drawRect(house_rect)

            if house_name == self.selected_house:
                painter.setBrush(Qt.NoBrush)
                painter.setPen(QPen(QColor(250, 210, 60), 4))
                painter.drawRect(house_rect.adjusted(-4, -4, 4, 4))

            painter.setPen(QPen(QColor(20, 20, 20), 1))
            painter.setFont(QFont('DejaVu Sans', 9, QFont.Bold))
            painter.drawText(house_rect.adjusted(-10, -20, 10, -6), Qt.AlignCenter, house_name)

        drone_canvas = self.world_to_canvas(self.drone_world_position[0], self.drone_world_position[1])
        painter.setBrush(QBrush(QColor(220, 35, 35)))
        painter.setPen(QPen(QColor(120, 0, 0), 1))
        painter.drawEllipse(drone_canvas, 6.0, 6.0)

        painter.setPen(QPen(QColor(30, 30, 30), 1))
        painter.setFont(QFont('DejaVu Sans', 9))
        painter.drawText(
            self.rect().adjusted(10, 8, -10, -8),
            Qt.AlignTop | Qt.AlignLeft,
            'Top-Down Map (Gazebo World Coordinates)',
        )


class DeliveryGuiWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()

        self.setWindowTitle('MARS Drone Delivery')
        self.setMinimumSize(640, 700)

        self.ros_node: Optional['DeliveryGuiRosNode'] = None
        self.last_clicked_house: Optional[str] = None
        self.current_state = 'IDLE'

        self.map_widget = DeliveryMapWidget(self.handle_house_clicked)

        self.info_label = QLabel('Select a house to send the drone from the warehouse.')
        self.info_label.setAlignment(Qt.AlignCenter)
        self.info_label.setFont(QFont('DejaVu Sans', 10))

        central_widget = QWidget()
        layout = QVBoxLayout(central_widget)
        layout.addWidget(self.map_widget, alignment=Qt.AlignCenter)
        layout.addWidget(self.info_label)
        self.setCentralWidget(central_widget)

        self.statusBar().showMessage('Mission: Idle | State: IDLE')

    def attach_ros_node(self, ros_node: 'DeliveryGuiRosNode') -> None:
        self.ros_node = ros_node

    def handle_house_clicked(self, house_name: str) -> None:
        self.last_clicked_house = house_name
        self.info_label.setText(f'Mission: Flying to {house_name}')

        if self.ros_node is not None:
            self.ros_node.publish_target(house_name)

        self.refresh_status()

    def update_drone_position(self, x: float, y: float) -> None:
        self.map_widget.set_drone_position(x, y)

    def update_state(self, state: str) -> None:
        self.current_state = state

        if state == 'IDLE':
            self.info_label.setText('Mission: Idle')
        elif state == 'DELIVER' and self.last_clicked_house is not None:
            self.info_label.setText(f'Mission: Delivering at {self.last_clicked_house}')

        self.refresh_status()

    def refresh_status(self) -> None:
        mission_text = 'Mission: Idle'
        if self.last_clicked_house is not None and self.current_state != 'IDLE':
            mission_text = f'Mission: Flying to {self.last_clicked_house}'
        self.statusBar().showMessage(f'{mission_text} | State: {self.current_state}')


class DeliveryGuiRosNode(Node):
    def __init__(self, window: DeliveryGuiWindow) -> None:
        super().__init__('delivery_gui_node')
        self.window = window

        self.target_pub = self.create_publisher(String, '/mars/delivery_target', 10)
        self.odom_sub = self.create_subscription(Odometry, '/drone/odom', self.odom_callback, 20)
        self.state_sub = self.create_subscription(String, '/mars/mission_state', self.state_callback, 10)

        self.get_logger().info('MARS delivery GUI is running.')

    def publish_target(self, house_name: str) -> None:
        msg = String()
        msg.data = house_name
        self.target_pub.publish(msg)
        self.get_logger().info(f'Published delivery target: {house_name}')

    def odom_callback(self, msg: Odometry) -> None:
        self.window.update_drone_position(msg.pose.pose.position.x, msg.pose.pose.position.y)

    def state_callback(self, msg: String) -> None:
        self.window.update_state(msg.data)


def main(args=None) -> None:
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    window = DeliveryGuiWindow()
    ros_node = DeliveryGuiRosNode(window)
    window.attach_ros_node(ros_node)
    window.show()

    spin_timer = QTimer()
    cleanup_done = False

    def cleanup_ros() -> None:
        nonlocal cleanup_done
        if cleanup_done:
            return

        cleanup_done = True
        spin_timer.stop()

        try:
            ros_node.destroy_node()
        except Exception:  # pylint: disable=broad-except
            pass

        if rclpy.ok():
            rclpy.shutdown()

    def spin_ros_once() -> None:
        if cleanup_done or not rclpy.ok():
            spin_timer.stop()
            return

        try:
            rclpy.spin_once(ros_node, timeout_sec=0.0)
        except Exception:  # pylint: disable=broad-except
            # During shutdown, Qt timers may fire after rclpy context is invalidated.
            # Stop spinning to avoid repeated RCLError traces.
            spin_timer.stop()

    spin_timer.timeout.connect(spin_ros_once)
    spin_timer.start(20)

    app.aboutToQuit.connect(cleanup_ros)

    exit_code = app.exec_()

    cleanup_ros()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
