# test_agentic_ros.py

import rospy
from std_msgs.msg import String
import json
import time

def publish_robot_status():
    """Simulate robot status updates"""
    pub = rospy.Publisher('robot_status', String, queue_size=10)
    rospy.init_node('robot_status_simulator')
    rate = rospy.Rate(1)  # 1 Hz
    
    battery_level = 100
    x, y = 0, 0
    
    while not rospy.is_shutdown():
        status = {
            'battery_level': battery_level,
            'position': {'x': x, 'y': y},
            'gripper_state': 'open',
            'system_status': 'operational',
        }
        
        pub.publish(String(json.dumps(status)))
        battery_level = max(0, battery_level - 0.1)
        x = (x + 0.1) % 10  # Simulate movement
        rate.sleep()

def command_callback(msg):
    """Handle robot commands"""
    try:
        command = json.loads(msg.data)
        rospy.loginfo(f"Received command: {command}")
        # In real implementation, would execute command here
    except json.JSONDecodeError:
        rospy.logwarn(f"Invalid command message: {msg.data}")

if __name__ == '__main__':
    try:
        # Subscribe to robot commands
        rospy.Subscriber('robot_commands', String, command_callback)
        # Start publishing status
        publish_robot_status()
    except rospy.ROSInterruptException:
        pass