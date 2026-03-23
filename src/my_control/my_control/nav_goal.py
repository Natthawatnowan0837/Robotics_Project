#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float32MultiArray
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import subprocess
import os

class MapNavigator(Node):
    def __init__(self):
        super().__init__('map_navigator')
        
        # --- [ Configurations ] ---
        self.params_file = '/home/noone/Robotics_Project/src/my_control/config/nav2_params.yaml'
        self.target_x = None
        self.target_y = None
        self.is_nav_active = False 
        self.nav2_launched = False  
        self.goal_sent = False     
        self.nav2_process = None
        self.nav = None             

        # --- [ Publishers ] ---
        # เพิ่ม Publisher สำหรับส่งสถานะกลับไปให้ Manager
        self.pub_status = self.create_publisher(String, 'status', 10)

        # --- [ Subscriptions ] ---
        self.sub_action = self.create_subscription(String, 'action', self.action_callback, 10)
        self.sub_target = self.create_subscription(Float32MultiArray, 'final_target', self.target_callback, 10)

        # Timer: เช็คเงื่อนไขทุก 1 วินาที
        self.timer = self.create_timer(1.0, self.navigation_logic_loop)
        
        self.get_logger().info('✅ Node Started. Waiting for Signal & Target...')

    def launch_nav2_stack(self):
        if self.nav2_launched:
            return

        self.get_logger().info('🚀 Conditions met! Starting Nav2 Stack...')
        
        cmd = [
            'ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
            'use_sim_time:=false',
            f'params_file:={self.params_file}',
            'use_amcl:=false'
        ]
        
        try:
            self.nav2_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
            self.nav2_launched = True
            
            # สร้าง Navigator และรอให้ระบบ Active
            self.nav = BasicNavigator()
            self.get_logger().info('⏳ Waiting for Nav2 Services to become Active...')
            self.nav.waitUntilNav2Active(localizer='bt_navigator')
            
            self.get_logger().info('✅ Nav2 is Ready!')
            
            # --- [ ส่งสถานะยืนยันกลับไปที่ Manager ] ---
            status_msg = String()
            status_msg.data = 'nav2,done'
            self.pub_status.publish(status_msg)
            self.get_logger().info('📤 Sent status: "nav2,done" to Manager')
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to launch Nav2: {e}')

    def action_callback(self, msg):
        if msg.data.lower() == 'nav2':
            if not self.is_nav_active:
                self.get_logger().info('🔔 [SIGNAL] "nav2" received.')
                self.is_nav_active = True

    def target_callback(self, msg):
        if len(msg.data) >= 2:
            self.target_x = msg.data[0]
            self.target_y = msg.data[1]
            self.get_logger().info(f"📍 Target set to: x={self.target_x}, y={self.target_y}")

    def navigation_logic_loop(self):
        # 1. เช็คว่าต้องเริ่ม Launch หรือยัง (ต้องมีทั้ง Action และ Target)
        if self.is_nav_active and self.target_x is not None and not self.nav2_launched:
            self.launch_nav2_stack()
            return

        # 2. ถ้า Launch แล้ว และพร้อมส่ง Goal
        if self.nav2_launched and not self.goal_sent and self.nav:
            self.send_nav_goal(self.target_x, self.target_y)
            self.goal_sent = True

        # 3. ติดตามสถานะ
        if self.goal_sent and self.nav:
            if not self.nav.isTaskComplete():
                feedback = self.nav.getFeedback()
                if feedback:
                    self.get_logger().info(f'🛰️ Remaining: {feedback.distance_remaining:.2f} m', throttle_duration_sec=5.0)
            else:
                result = self.nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info('🏁 Goal Reached Successfully!')
                else:
                    self.get_logger().error('❌ Navigation Failed.')
                self.goal_sent = False 

    def send_nav_goal(self, x, y):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.orientation.w = 1.0 

        self.get_logger().info(f'🚀 Sending Robot to: x={x}, y={y}')
        self.nav.goToPose(goal_pose)

    def destroy_node(self):
        if self.nav2_process:
            self.get_logger().info('🛑 Terminating Nav2 Stack...')
            self.nav2_process.terminate()
            self.nav2_process.wait()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MapNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()