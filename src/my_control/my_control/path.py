#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np
import math
from typing import List, Tuple, Optional

# === Import Message types ===
from std_msgs.msg import Float32MultiArray 
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# =============================================================
# 1. IMPORT Service ให้ถูกต้อง
# ต้องใช้ Sendposition จาก package ที่คุณเก็บไฟล์ .srv ไว้ (my_command)
# =============================================================
from my_command.srv import Sendposition 

# Type definitions
WorldPt = Tuple[float, float]
GridPt  = Tuple[int, int]

class SimpleAStarPlanner(Node):
    """
    Pure A* Planner บน Robot-Centric Map
    - รับเป้าหมายจาก Service 'move_robot_service' (Type: Sendposition)
    """
    
    def __init__(self):
        super().__init__('path_planing_astar')

        # === QoS ===
        qos_scan = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # === Subscribers ===
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, qos_scan)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 50)
        
        # =========================================================
        # 2. สร้าง Service Server ด้วย Type 'Sendposition'
        # =========================================================
        self.srv = self.create_service(Sendposition, 'move_robot_service', self.handle_move_service)

        # === Goal Subscriber (Debug) ===
        self.goal_sub = self.create_subscription(
            Float32MultiArray, 
            '/input_coordinates', 
            self.goal_callback_topic, 
            10
        )

        # === Publishers ===
        self.map_pub  = self.create_publisher(OccupancyGrid, '/local_map', 50)
        self.path_pub = self.create_publisher(Path, '/path', 50)

        # === Map Parameters ===
        self.map_size   = 10.0     
        self.resolution = 0.05     
        self.width      = int(self.map_size / self.resolution)
        self.height     = int(self.map_size / self.resolution)
        
        self.grid       = np.zeros((self.height, self.width), dtype=np.float32)
        self.memory     = np.zeros_like(self.grid) 

        # === Goal Setting ===
        self.goal_world: Optional[WorldPt] = None 

        # === Robot State ===
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_ready = False

        # === Planning Control ===
        self.last_plan_pose: WorldPt = (0.0, 0.0)
        self.last_plan_time = self.get_clock().now()
        self.replan_dist_thresh = 0.5 
        self.replan_time_thresh = 2.0 

        # === Timers ===
        self._last_map_pub = self.get_clock().now()
        self.map_pub_interval = 0.5  

        self.get_logger().info("A* Planner Ready (Service Mode). Waiting for command...")

    # =============================================================
    # 3. SERVICE CALLBACK (จัดการ Request และ Response)
    # =============================================================
    def handle_move_service(self, request, response):
        """
        รับค่าจาก Client (Voice Processor)
        Request:
          - request.room_name (string)
          - request.x (float32)
          - request.y (float32)
          - request.z (float32) -> Floor
        Response:
          - response.success (bool)
          - response.message (string)
        """
        # Log ข้อมูลที่ได้รับ
        self.get_logger().info(f"🎤 Service Request Received:")
        self.get_logger().info(f"   Room: {request.room_name}")
        self.get_logger().info(f"   Target: X={request.x:.2f}, Y={request.y:.2f}, Floor={int(request.z)}")

        # 1. ตั้งค่าเป้าหมาย (Goal) สำหรับ A* (ใช้แค่ x, y)
        self.goal_world = (request.x, request.y)

        # 2. สั่งคำนวณเส้นทางทันที
        self.plan_path()

        # 3. ส่งค่าตอบกลับ (Response)
        response.success = True
        response.message = f"Received! Navigating to {request.room_name} on Floor {int(request.z)}"
        
        return response

    # ===================== TOPIC CALLBACK (DEBUG) =====================
    def goal_callback_topic(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            gx, gy, gz = msg.data[0], msg.data[1], msg.data[2]
            self.goal_world = (gx, gy)
            self.get_logger().info(f"📍 Manual Topic Goal: X={gx:.2f}, Y={gy:.2f}, Floor={int(gz)}")
            self.plan_path()

    # ===================== ODOMETRY =====================
    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.robot_yaw = yaw
        self.odom_ready = True

    # ===================== LASER & MAPPING =====================
    def laser_callback(self, msg: LaserScan):
        if not self.odom_ready:
            return

        self.memory = np.clip(self.memory * 0.95, 0, 100)

        angle = msg.angle_min
        for r in msg.ranges:
            if np.isnan(r) or np.isinf(r) or r < msg.range_min or r > msg.range_max:
                angle += msg.angle_increment
                continue

            lx = r * math.cos(angle)
            ly = r * math.sin(angle)
            wx = self.robot_x + (lx * math.cos(self.robot_yaw) - ly * math.sin(self.robot_yaw))
            wy = self.robot_y + (lx * math.sin(self.robot_yaw) + ly * math.cos(self.robot_yaw))

            gx, gy = self.world_to_grid(wx, wy)
            if 0 <= gx < self.width and 0 <= gy < self.height:
                self.memory[gy, gx] = min(100, self.memory[gy, gx] + 60)

            angle += msg.angle_increment

        self.grid = self.inflate_obstacles(self.memory, radius=4)

        now = self.get_clock().now()
        if (now - self._last_map_pub).nanoseconds * 1e-9 > self.map_pub_interval:
            self.publish_occupancy_grid()
            self._last_map_pub = now

        if self.goal_world is not None:
            dist_moved = math.hypot(self.robot_x - self.last_plan_pose[0], self.robot_y - self.last_plan_pose[1])
            time_elapsed = (now - self.last_plan_time).nanoseconds * 1e-9
            if dist_moved > self.replan_dist_thresh or time_elapsed > self.replan_time_thresh:
                self.plan_path()

    # ===================== PLANNING (A*) =====================
    def plan_path(self):
        if self.goal_world is None:
            return

        start_node = self.world_to_grid(self.robot_x, self.robot_y)
        local_goal = self.compute_local_goal(self.goal_world)
        goal_node = self.world_to_grid(*local_goal)

        path_grid = self.astar(start_node, goal_node)

        if path_grid:
            self.publish_path(path_grid)
            self.last_plan_pose = (self.robot_x, self.robot_y)
            self.last_plan_time = self.get_clock().now()

    def astar(self, start: GridPt, goal: GridPt) -> Optional[List[GridPt]]:
        if not self.is_valid(start) or not self.is_valid(goal):
            return None
        
        open_set = {start}
        came_from = {}
        g_score = {start: 0.0}
        f_score = {start: self.heuristic(start, goal)}
        
        while open_set:
            current = min(open_set, key=lambda n: f_score.get(n, float('inf')))
            if current == goal:
                return self.reconstruct_path(came_from, current)

            open_set.remove(current)

            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                if not self.is_valid(neighbor) or not self.is_free(neighbor):
                    continue
                
                tentative_g = g_score[current] + math.hypot(dx, dy)
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    open_set.add(neighbor)
        return None

    # ===================== UTILS =====================
    def world_to_grid(self, wx: float, wy: float) -> GridPt:
        origin_x = self.robot_x - (self.map_size / 2.0)
        origin_y = self.robot_y - (self.map_size / 2.0)
        gx = int((wx - origin_x) / self.resolution)
        gy = int((wy - origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> WorldPt:
        origin_x = self.robot_x - (self.map_size / 2.0)
        origin_y = self.robot_y - (self.map_size / 2.0)
        wx = origin_x + (gx * self.resolution)
        wy = origin_y + (gy * self.resolution)
        return wx, wy

    def compute_local_goal(self, global_goal: WorldPt) -> WorldPt:
        gx, gy = global_goal
        margin = 1.0 
        min_x, max_x = self.robot_x - self.map_size/2.0 + margin, self.robot_x + self.map_size/2.0 - margin
        min_y, max_y = self.robot_y - self.map_size/2.0 + margin, self.robot_y + self.map_size/2.0 - margin
        return (np.clip(gx, min_x, max_x), np.clip(gy, min_y, max_y))

    def heuristic(self, a, b): return math.hypot(b[0]-a[0], b[1]-a[1])
    def is_valid(self, p): return 0 <= p[0] < self.width and 0 <= p[1] < self.height
    def is_free(self, p): return self.grid[p[1], p[0]] < 50

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def inflate_obstacles(self, grid_data, radius=3):
        inflated = np.copy(grid_data)
        rows, cols = np.where(grid_data > 50) 
        for r, c in zip(rows, cols):
            r_min, r_max = max(0, r-radius), min(self.height, r+radius+1)
            c_min, c_max = max(0, c-radius), min(self.width, c+radius+1)
            inflated[r_min:r_max, c_min:c_max] = np.maximum(inflated[r_min:r_max, c_min:c_max], 80)
        return inflated

    # ===================== PUBLISHERS =====================
    def publish_path(self, path_grid: List[GridPt]):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        for gx, gy in path_grid:
            wx, wy = self.grid_to_world(gx, gy)
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def publish_occupancy_grid(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.info.resolution = self.resolution
        msg.info.width, msg.info.height = self.width, self.height
        msg.info.origin.position.x = self.robot_x - (self.map_size / 2.0)
        msg.info.origin.position.y = self.robot_y - (self.map_size / 2.0)
        msg.info.origin.orientation.w = 1.0
        data = np.zeros_like(self.grid, dtype=np.int8)
        data[self.grid >= 50] = 100 
        msg.data = data.flatten().tolist()
        self.map_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleAStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()