#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import redis, json, time, threading, signal, sys, os

REDIS_KEYS = {
    'command': 'robot_command',
    'feedback': 'robot_feedback',
    'status': 'robot_status',
    'last_command': 'last_command_status'
}

DEBUG_MODE = os.getenv("DEBUG_MODE", "false").lower() == "true"

class RobotBridge(Node):
    def __init__(self):
        super().__init__('robot_bridge')
        try:
            self.redis_client = redis.Redis(host='localhost', port=6379, db=0, decode_responses=True)
            self.redis_client.ping()
            self.get_logger().info('✅ Connected to Redis')
        except redis.ConnectionError:
            self.get_logger().error('❌ Could not connect to Redis')
            raise

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_publisher = self.create_publisher(String, '/robot_status', 10)
        self.status_subscription = self.create_subscription(String, '/robot_feedback', self.robot_feedback_callback, 10)

        self.command_timer = self.create_timer(0.1, self.process_redis_commands)
        self.status_timer = self.create_timer(1.0, self.publish_robot_status)

        self.last_command_time = 0
        self.robot_state = "idle"
        self.get_logger().info('🤖 ROS2 Robot Bridge Started')

    def robot_feedback_callback(self, msg):
        feedback_data = {
            'timestamp': time.time(),
            'message': msg.data,
            'status': 'received',
            'source': 'ros2'
        }
        self.redis_client.set(REDIS_KEYS['feedback'], json.dumps(feedback_data))
        self.get_logger().info(f'📥 Feedback: {msg.data}')

    def process_redis_commands(self):
        try:
            raw = self.redis_client.get(REDIS_KEYS['command'])
            if raw:
                command = json.loads(raw)
                ts = command.get('timestamp', 0)
                if ts <= self.last_command_time:
                    self.get_logger().info(f'⏸️ Ignored stale command')
                    return
                self.execute_robot_command(command)
                self.last_command_time = ts
                self.redis_client.delete(REDIS_KEYS['command'])
        except Exception as e:
            self.get_logger().error(f'Error reading Redis: {e}')

    def execute_robot_command(self, command):
        action = command.get('action', '')
        params = command.get('parameters', {})
        self.robot_state = f"executing_{action}"
        self.get_logger().info(f'🎮 Executing: {action} | params: {params}')

        if action == 'move_forward':
            self.send_velocity_command(0.5, 0.0)
        elif action == 'move_backward':
            self.send_velocity_command(-0.5, 0.0)
        elif action == 'turn_left':
            self.send_velocity_command(0.0, 0.5)
        elif action == 'turn_right':
            self.send_velocity_command(0.0, -0.5)
        elif action == 'stop':
            self.send_velocity_command(0.0, 0.0)
            self.robot_state = "stopped"
        elif action == 'home':
            self.execute_home_sequence()
        else:
            self.get_logger().warn(f'⚠️ Unknown command: {action}')
            self.robot_state = "error"

        self.update_command_status(action, 'executed')

    def send_velocity_command(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f'📤 Velocity: {linear_x:.2f}, {angular_z:.2f}')

    def execute_home_sequence(self):
        self.robot_state = "homing"
        self.send_velocity_command(0.0, 0.0)
        threading.Timer(3.0, self.complete_homing).start()

    def complete_homing(self):
        self.robot_state = "homed"
        self.get_logger().info('✅ Homing complete')

    def publish_robot_status(self):
        try:
            status = {
                'timestamp': time.time(),
                'state': self.robot_state,
                'node_status': 'running',
                'connections': {
                    'redis_connected': bool(self.redis_client.ping()),
                    'ros2_node_active': True
                },
                'last_command_time': self.last_command_time
            }
            msg = String()
            msg.data = json.dumps(status)
            self.status_publisher.publish(msg)
            self.redis_client.set(REDIS_KEYS['status'], msg.data)
            if DEBUG_MODE:
                self.get_logger().info(f'Debug status: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Status publish error: {e}')

    def update_command_status(self, command, status):
        data = {
            'command': command,
            'status': status,
            'timestamp': time.time(),
            'robot_state': self.robot_state
        }
        self.redis_client.set(REDIS_KEYS['last_command'], json.dumps(data))

def signal_handler(sig, frame):
    print('\n🛑 Shutting down ROS2 Bridge...')
    rclpy.shutdown()
    sys.exit(0)

def main(args=None):
    signal.signal(signal.SIGINT, signal_handler)
    rclpy.init(args=args)
    bridge = RobotBridge()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(bridge)
    print("✅ ROS2 Bridge running...")
    executor.spin()

if __name__ == '__main__':
    main()
