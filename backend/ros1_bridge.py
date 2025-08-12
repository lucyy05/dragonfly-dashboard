#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import redis, json, time, threading, signal, sys

REDIS_KEYS = {
    'command': 'robot_command',
    'feedback': 'robot_feedback',
    'status': 'robot_status',
    'last_command': 'last_command_status'
}

class ROS1Bridge:
    def __init__(self):
        rospy.init_node('ros1_bridge', anonymous=True)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.status_pub = rospy.Publisher('/robot_status', String, queue_size=10)
        rospy.Subscriber('/robot_feedback', String, self.feedback_cb)

        self.redis = redis.Redis(host='localhost', port=6379, decode_responses=True)
        self.last_command_time = 0
        self.robot_state = "idle"

        rospy.Timer(rospy.Duration(0.1), self.check_commands)
        rospy.Timer(rospy.Duration(1.0), self.publish_status)
        rospy.loginfo("🤖 ROS1 Bridge Started")

    def feedback_cb(self, msg):
        data = {
            'timestamp': time.time(),
            'message': msg.data,
            'status': 'received',
            'source': 'ros1'
        }
        self.redis.set(REDIS_KEYS['feedback'], json.dumps(data))
        rospy.loginfo(f'📥 Feedback: {msg.data}')

    def check_commands(self, event):
        try:
            raw = self.redis.get(REDIS_KEYS['command'])
            if raw:
                command = json.loads(raw)
                ts = command.get('timestamp', 0)
                if ts <= self.last_command_time:
                    return
                self.execute_command(command)
                self.last_command_time = ts
                self.redis.delete(REDIS_KEYS['command'])
        except Exception as e:
            rospy.logerr(f'Redis error: {e}')

    def execute_command(self, command):
        action = command.get('action', '')
        params = command.get('parameters', {})
        self.robot_state = f"executing_{action}"
        rospy.loginfo(f'🎮 Executing: {action} | params: {params}')

        twist = Twist()
        if action == 'move_forward':
            twist.linear.x = 0.5
        elif action == 'move_backward':
            twist.linear.x = -0.5
        elif action == 'turn_left':
            twist.angular.z = 0.5
        elif action == 'turn_right':
            twist.angular.z = -0.5
        elif action == 'stop':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.robot_state = "stopped"
        elif action == 'home':
            self.robot_state = "homing"
            threading.Timer
