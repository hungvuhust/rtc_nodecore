#!/usr/bin/env python3

"""
RTC AGV System Demo Script

Script demo để test và điều khiển hệ thống RTC AGV.
Hiển thị state và cho phép điều khiển cơ bản.

Author: RTC Technology JSC
License: Apache 2.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import time
import json
from datetime import datetime

# Messages
from vda5050_msgs.msg import State, Order, Node as VDANode, Edge, Action
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist


class RTCAGVDemo(Node):
    def __init__(self):
        super().__init__('rtc_agv_demo')

        self.get_logger().info("🚀 RTC AGV Demo Script khởi động...")

        # QoS profiles
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Subscribers để monitor hệ thống
        self.agv_state_sub = self.create_subscription(
            State, '/agv/state', self.agv_state_callback, qos_reliable)

        self.heartbeat_status_sub = self.create_subscription(
            String, '/agv/heartbeat_status', self.heartbeat_callback, 10)

        self.battery_info_sub = self.create_subscription(
            String, '/battery/info', self.battery_info_callback, 10)

        self.navigation_status_sub = self.create_subscription(
            String, '/navigation/status', self.navigation_callback, 10)

        self.safety_status_sub = self.create_subscription(
            String, '/safety/status', self.safety_callback, 10)

        # Publishers để điều khiển
        self.order_pub = self.create_publisher(
            Order, '/agv/order', qos_reliable)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.emergency_stop_pub = self.create_publisher(
            Bool, '/emergency_stop', qos_reliable)
        self.safety_reset_pub = self.create_publisher(
            Bool, '/safety/reset', qos_reliable)
        self.battery_charge_pub = self.create_publisher(
            Bool, '/battery/start_charging', 10)

        # State tracking
        self.last_agv_state = None
        self.last_heartbeat = ""
        self.last_battery_info = ""
        self.last_navigation_status = ""
        self.last_safety_status = ""

        # Demo timer
        self.demo_timer = self.create_timer(5.0, self.demo_cycle)
        self.cycle_count = 0

        self.get_logger().info("✅ RTC AGV Demo sẵn sàng!")
        self.print_usage()

    def agv_state_callback(self, msg):
        self.last_agv_state = msg
        self.get_logger().debug("📡 AGV State nhận được")

    def heartbeat_callback(self, msg):
        self.last_heartbeat = msg.data
        self.get_logger().debug("💓 Heartbeat: " + msg.data)

    def battery_info_callback(self, msg):
        self.last_battery_info = msg.data
        self.get_logger().debug("🔋 Battery: " + msg.data)

    def navigation_callback(self, msg):
        self.last_navigation_status = msg.data
        self.get_logger().debug("🗺️ Navigation: " + msg.data)

    def safety_callback(self, msg):
        self.last_safety_status = msg.data
        if "E-STOP ACTIVE" in msg.data:
            self.get_logger().warn("🚨 " + msg.data)
        else:
            self.get_logger().debug("🛡️ Safety: " + msg.data)

    def print_usage(self):
        print("\n" + "="*80)
        print("🎮 RTC AGV DEMO CONTROLS")
        print("="*80)
        print("Các lệnh điều khiển (gõ trong terminal khác):")
        print()
        print("🚚 MOVEMENT:")
        print(
            "  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"linear: {x: 0.5}\"")
        print(
            "  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"angular: {z: 0.3}\"")
        print("  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{}\"  # Stop")
        print()
        print("🔋 BATTERY:")
        print("  ros2 topic pub /battery/start_charging std_msgs/msg/Bool \"data: true\"")
        print("  ros2 topic pub /battery/start_charging std_msgs/msg/Bool \"data: false\"")
        print()
        print("🚨 SAFETY:")
        print("  ros2 topic pub /emergency_stop std_msgs/msg/Bool \"data: true\"")
        print("  ros2 topic pub /safety/reset std_msgs/msg/Bool \"data: true\"")
        print()
        print("📡 MONITORING:")
        print("  ros2 topic echo /agv/state")
        print("  ros2 topic echo /agv/heartbeat_status")
        print("="*80)
        print()

    def demo_cycle(self):
        """Automatic demo cycle để showcase các tính năng"""
        self.cycle_count += 1

        print(
            f"\n🔄 DEMO CYCLE #{self.cycle_count} - {datetime.now().strftime('%H:%M:%S')}")
        print("-" * 60)

        # Display current status
        self.display_status()

        # Auto demo actions theo cycle
        if self.cycle_count == 2:
            self.get_logger().info("🚀 Demo: Bắt đầu automatic navigation...")
            self.send_demo_order()

        elif self.cycle_count == 6:
            self.get_logger().info("🔋 Demo: Bắt đầu charging...")
            self.start_charging(True)

        elif self.cycle_count == 12:
            self.get_logger().info("🔋 Demo: Dừng charging...")
            self.start_charging(False)

        elif self.cycle_count == 15:
            self.get_logger().info("🚨 Demo: Test emergency stop...")
            self.trigger_emergency_stop()

        elif self.cycle_count == 18:
            self.get_logger().info("✅ Demo: Reset emergency stop...")
            self.reset_safety()

        elif self.cycle_count == 25:
            self.get_logger().info("🔄 Demo: Reset cycle count...")
            self.cycle_count = 0

    def display_status(self):
        """Hiển thị tổng quan trạng thái hệ thống"""

        print("📊 SYSTEM STATUS:")
        print(f"  💓 Heartbeat: {self.last_heartbeat}")
        print(f"  🔋 Battery: {self.last_battery_info}")
        print(f"  🗺️ Navigation: {self.last_navigation_status}")
        print(f"  🛡️ Safety: {self.last_safety_status}")

        if self.last_agv_state:
            state = self.last_agv_state
            print()
            print("🤖 AGV STATE:")
            print(f"  • Operating Mode: {state.operating_mode}")
            print(f"  • Driving: {state.driving}")
            print(f"  • Paused: {state.paused}")
            print(f"  • Battery: {state.battery_state.battery_charge:.1f}% " +
                  f"({'CHARGING' if state.battery_state.charging else 'DISCHARGING'})")
            print(
                f"  • Position: ({state.agv_position.x:.2f}, {state.agv_position.y:.2f}, {state.agv_position.theta:.2f})")
            print(
                f"  • Velocity: ({state.velocity.vx:.2f}, {state.velocity.omega:.2f})")
            print(f"  • Actions: {len(state.action_states)} active")
            print(f"  • Errors: {len(state.errors)} errors")

            if state.errors:
                print("  🚨 ERRORS:")
                for error in state.errors:
                    print(
                        f"    - {error.error_type} ({error.error_level}): {error.error_description}")

        print("-" * 60)

    def send_demo_order(self):
        """Gửi order demo để test navigation"""
        order = Order()
        order.header_id = int(time.time())
        # order.timestamp = self.get_clock().now().to_msg()
        order.version = "2.0.0"
        order.manufacturer = "RTC Technology"
        order.serial_number = "AGV_001"
        order.order_id = f"order_{int(time.time())}"
        order.order_update_id = 1

        # Tạo nodes cho path
        node1 = VDANode()
        node1.node_id = "node_start"
        node1.sequence_id = 0
        node1.node_position.x = 10.0
        node1.node_position.y = 20.0
        node1.node_position.theta = 0.0
        node1.node_position.map_id = "warehouse_map_v1"

        node2 = VDANode()
        node2.node_id = "node_target"
        node2.sequence_id = 1
        node2.node_position.x = 15.0
        node2.node_position.y = 25.0
        node2.node_position.theta = 1.57
        node2.node_position.map_id = "warehouse_map_v1"

        # Tạo edge
        edge = Edge()
        edge.edge_id = "edge_001"
        edge.sequence_id = 0
        edge.start_node_id = "node_start"
        edge.end_node_id = "node_target"
        edge.max_speed = 1.0
        edge.max_height = 2.0
        edge.length = 7.07  # sqrt((15-10)^2 + (25-20)^2)

        order.nodes = [node1, node2]
        order.edges = [edge]

        self.order_pub.publish(order)
        self.get_logger().info(f"📤 Đã gửi demo order: {order.order_id}")

    def start_charging(self, enable: bool):
        """Bắt đầu/dừng charging"""
        msg = Bool()
        msg.data = enable
        self.battery_charge_pub.publish(msg)
        action = "Bắt đầu" if enable else "Dừng"
        self.get_logger().info(f"🔋 {action} charging...")

    def trigger_emergency_stop(self):
        """Kích hoạt emergency stop"""
        msg = Bool()
        msg.data = True
        self.emergency_stop_pub.publish(msg)
        self.get_logger().warn("🚨 EMERGENCY STOP được kích hoạt!")

    def reset_safety(self):
        """Reset safety system"""
        msg = Bool()
        msg.data = True
        self.safety_reset_pub.publish(msg)
        self.get_logger().info("✅ Safety system được reset")

    def send_velocity_command(self, linear_x: float, angular_z: float):
        """Gửi lệnh điều khiển velocity"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(
            f"🎮 Velocity command: linear={linear_x}, angular={angular_z}")


def main(args=None):
    rclpy.init(args=args)

    print("🚀 Khởi động RTC AGV Demo...")
    print("⏳ Đợi các nodes khởi động...")
    time.sleep(3)

    demo_node = RTCAGVDemo()

    try:
        print("✅ Demo đang chạy... (Ctrl+C để thoát)")
        rclpy.spin(demo_node)
    except KeyboardInterrupt:
        print("\n🛑 Demo dừng bởi người dùng")
    finally:
        demo_node.destroy_node()
        rclpy.shutdown()
        print("👋 Tạm biệt!")


if __name__ == '__main__':
    main()
