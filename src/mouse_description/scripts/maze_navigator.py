#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
import math
from collections import deque

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z  

def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

# ==========================================
# FLOOD FILL ALGORITHM CLASS
# ==========================================
class FloodFill:
    def __init__(self):
        self.size = 16  
        self.dist = [[9999 for _ in range(self.size)] for _ in range(self.size)]
        self.walls = [[[False, False, False, False] for _ in range(self.size)] for _ in range(self.size)]
        
        self.targets = [(7, 7), (7, 8), (8, 7), (8, 8)]
        self.dx = [0, 1, 0, -1]
        self.dy = [1, 0, -1, 0]
        self.recalculate_distances()

    def add_wall(self, x, y, direction):
        if 0 <= x < self.size and 0 <= y < self.size:
            self.walls[x][y][direction] = True
            nx, ny = x + self.dx[direction], y + self.dy[direction]
            if 0 <= nx < self.size and 0 <= ny < self.size:
                self.walls[nx][ny][(direction + 2) % 4] = True

    def remove_wall(self, x, y, direction):
        if 0 <= x < self.size and 0 <= y < self.size:
            self.walls[x][y][direction] = False
            nx, ny = x + self.dx[direction], y + self.dy[direction]
            if 0 <= nx < self.size and 0 <= ny < self.size:
                self.walls[nx][ny][(direction + 2) % 4] = False

    def recalculate_distances(self):
        for i in range(self.size):
            for j in range(self.size):
                self.dist[i][j] = 9999
                
        queue = deque()
        for tx, ty in self.targets:
            self.dist[tx][ty] = 0
            queue.append((tx, ty))
            
        while queue:
            x, y = queue.popleft()
            current_d = self.dist[x][y]
            for i in range(4):
                if not self.walls[x][y][i]:
                    nx, ny = x + self.dx[i], y + self.dy[i]
                    if 0 <= nx < self.size and 0 <= ny < self.size:
                        if self.dist[nx][ny] == 9999:
                            self.dist[nx][ny] = current_d + 1
                            queue.append((nx, ny))

    def get_best_move(self, x, y, current_heading, logger=None):
        if not (0 <= x < self.size and 0 <= y < self.size):
            return current_heading
            
        min_d = 9999
        
        for i in range(4):
            if not self.walls[x][y][i]:
                nx, ny = x + self.dx[i], y + self.dy[i]
                if 0 <= nx < self.size and 0 <= ny < self.size:
                    if self.dist[nx][ny] < min_d:
                        min_d = self.dist[nx][ny]
                        
        if min_d == 9999:
            if logger:
                logger.warn("[FLOODFILL] NO PATH TO TARGET! Forcing U-Turn to escape trap.")
            return (current_heading + 2) % 4
            
        preferences = [
            current_heading, 
            (current_heading + 1) % 4, 
            (current_heading - 1) % 4, 
            (current_heading + 2) % 4
        ]
        
        for h in preferences:
            if not self.walls[x][y][h]:
                nx, ny = x + self.dx[h], y + self.dy[h]
                if 0 <= nx < self.size and 0 <= ny < self.size:
                    if self.dist[nx][ny] == min_d:
                        return h
                        
        return current_heading

# ==========================================
# ROS 2 NODE
# ==========================================
class MazeNavigator(Node):
    def __init__(self):
        super().__init__('maze_navigator')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/tof/middle', self.tof_m_callback, 10)
        self.create_subscription(LaserScan, '/tof/right', self.tof_r_callback, 10)
        self.create_subscription(LaserScan, '/tof/left', self.tof_l_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Speed & Thresholds
        self.forward_speed = 0.2           
        self.min_forward_speed = 0.1        
        self.braking_distance = 0.20 
        self.odom_braking_distance = 0.08  
        
        self.turn_speed = 1.0          
        self.min_turn_speed = 0.3     
        self.front_stop_distance = 0.065   
        self.hole_alignment_dist = 0.065    
        
        # Heading PID
        self.kp_heading = 2.0           
        self.kd_heading = 0.5           
        self.prev_yaw_error = 0.0

        # Linear PID
        self.kp_linear = 2.0
        self.kd_linear = 0.8
        self.prev_linear_error = 0.0

        self.dist_m = 2.0
        self.raw_r = 2.0
        self.raw_l = 2.0
        self.lat_r = 2.0
        self.lat_l = 2.0
        self.fwd_r = 2.0
        self.fwd_l = 2.0
        self.current_yaw = 0.0
        
        # Real-time wall latches (CRITICAL for 45-degree sensors)
        self.right_wall_blocked = True
        self.left_wall_blocked = True
        
        self.start_x = 0.0
        self.start_y = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.cell_size = 0.18
        
        self.grid_x = 0
        self.grid_y = 0
        self.heading = 0 
        
        self.flood_fill = FloodFill()

        self.state = 'WAIT_FOR_ODOM'
        self.next_state_after_settle = 'EVALUATE_CELL'
        self.target_distance = 0.09 
        self.first_cell_evaluated = False
        self.is_short_movement = False
        
        self.target_yaw = 0.0
        self.settle_counter = 0

        self.timer = self.create_timer(0.01, self.control_loop)
        self.get_logger().info("[START] Dynamic Hardware-Centering Navigator Started.")

    def tof_m_callback(self, msg):
        self.dist_m = msg.ranges[0] if not math.isinf(msg.ranges[0]) else 2.0
        
    def tof_r_callback(self, msg):
        self.raw_r = msg.ranges[0] if not math.isinf(msg.ranges[0]) else 2.0
        self.lat_r = self.raw_r * math.sin(math.radians(45))
        self.fwd_r = self.raw_r * math.cos(math.radians(45))
        
    def tof_l_callback(self, msg):
        self.raw_l = msg.ranges[0] if not math.isinf(msg.ranges[0]) else 2.0
        self.lat_l = self.raw_l * math.sin(math.radians(45))
        self.fwd_l = self.raw_l * math.cos(math.radians(45))
        
    def imu_callback(self, msg):
        q = msg.orientation
        self.current_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        
    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

    def control_loop(self):
        vel_msg = Twist()
        
        if self.state == 'FINISHED':
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.cmd_pub.publish(vel_msg)
            return

        if self.state == 'MOVE_FORWARD':
            if self.raw_r > 0.24:
                if self.right_wall_blocked:
                    self.right_wall_blocked = False
                    self.get_logger().info(f"[SCAN] Right Open Latched! Raw:{self.raw_r:.2f} > 0.24")
            
            if self.raw_l > 0.24:
                if self.left_wall_blocked:
                    self.left_wall_blocked = False
                    self.get_logger().info(f"[SCAN] Left Open Latched! Raw:{self.raw_l:.2f} > 0.24")

        if self.state == 'WAIT_FOR_ODOM':
            if self.odom_x != 0.0 or self.odom_y != 0.0:
                self.start_x = self.odom_x
                self.start_y = self.odom_y
                self.state = 'MOVE_FORWARD'
                self.get_logger().info("[DEBUG] Odometry active. Moving 9cm to align center.")

        elif self.state == 'EVALUATE_CELL':
            if (self.grid_x, self.grid_y) in self.flood_fill.targets:
                self.get_logger().info(f"[VICTORY] Target Reached at Cell({self.grid_x}, {self.grid_y})! Maze solved.")
                self.state = 'FINISHED'
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
                self.cmd_pub.publish(vel_msg)
                return

            self.target_distance = self.cell_size 
            self.grid_x = max(0, min(self.flood_fill.size - 1, self.grid_x))
            self.grid_y = max(0, min(self.flood_fill.size - 1, self.grid_y))

            front_blocked = self.dist_m < 0.15
            right_blocked = self.right_wall_blocked
            left_blocked = self.left_wall_blocked

            self.get_logger().info(
                f"[EVALUATE] Cell({self.grid_x},{self.grid_y}) | Head: {self.heading} | "
                f"Front Distance: {self.dist_m:.3f}m | Blocked -> F: {front_blocked}, R: {right_blocked}, L: {left_blocked}"
            )
            
            if front_blocked: self.flood_fill.add_wall(self.grid_x, self.grid_y, self.heading)
            else: self.flood_fill.remove_wall(self.grid_x, self.grid_y, self.heading)
            
            if not self.is_short_movement:
                if right_blocked: self.flood_fill.add_wall(self.grid_x, self.grid_y, (self.heading + 1) % 4)
                else: self.flood_fill.remove_wall(self.grid_x, self.grid_y, (self.heading + 1) % 4)
                
                if left_blocked:  self.flood_fill.add_wall(self.grid_x, self.grid_y, (self.heading - 1) % 4)
                else: self.flood_fill.remove_wall(self.grid_x, self.grid_y, (self.heading - 1) % 4)
            
            self.right_wall_blocked = True
            self.left_wall_blocked = True
            
            self.flood_fill.recalculate_distances()
            best_heading = self.flood_fill.get_best_move(self.grid_x, self.grid_y, self.heading, self.get_logger())
            
            self.start_x = self.odom_x
            self.start_y = self.odom_y
            
            if best_heading == self.heading:
                self.state = 'MOVE_FORWARD'
                self.get_logger().info(f"[PATH] Clear ahead. Continuing seamlessly.")
                
                vel_msg.linear.x = self.forward_speed
                yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
                angular_correction = (self.kp_heading * yaw_error) + (self.kd_heading * (yaw_error - self.prev_yaw_error))
                self.prev_yaw_error = yaw_error
                vel_msg.angular.z = max(-0.5, min(0.5, angular_correction))
            else:
                diff = (best_heading - self.heading) % 4
                if diff == 1:
                    turn_dir = -1 
                    self.get_logger().info(f"[TURN] Best path is RIGHT. Stopping to rotate.")
                elif diff == 3:
                    turn_dir = 1  
                    self.get_logger().info(f"[TURN] Best path is LEFT. Stopping to rotate.")
                else:
                    turn_dir = 2  
                    self.get_logger().info(f"[TURN] Dead end. Stopping for U-TURN.")
                    
                self.heading = best_heading
                self.target_yaw = normalize_angle(self.target_yaw + (turn_dir * (math.pi / 2.0)))
                
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
                self.next_state_after_settle = 'TURNING'
                self.state = 'SETTLE'

        elif self.state == 'MOVE_FORWARD':
            dist_moved = math.sqrt((self.odom_x - self.start_x)**2 + (self.odom_y - self.start_y)**2)
            
            active_odom_limit = self.target_distance
            if self.target_distance >= 0.18: 
                if self.dist_m < 0.25: 
                    active_odom_limit = 0.25 
                elif (not self.right_wall_blocked) and (self.fwd_r < 0.15): 
                    active_odom_limit = 0.22
                elif (not self.left_wall_blocked) and (self.fwd_l < 0.15): 
                    active_odom_limit = 0.22
                    
            moved_enough = dist_moved > 0.12 
            
            reached_front = self.dist_m <= self.front_stop_distance
            align_right   = (not self.right_wall_blocked) and moved_enough and (0.02 < self.fwd_r <= self.hole_alignment_dist)
            align_left    = (not self.left_wall_blocked)  and moved_enough and (0.02 < self.fwd_l <= self.hole_alignment_dist)
            reached_odom  = dist_moved >= active_odom_limit
            
            if reached_front or align_right or align_left or reached_odom:
                if reached_front: self.get_logger().info(f"[CENTER] Centered by FRONT Wall (dist_m: {self.dist_m:.3f}m)")
                elif align_right: self.get_logger().info(f"[CENTER] Centered by RIGHT Hole")
                elif align_left: self.get_logger().info(f"[CENTER] Centered by LEFT Hole")
                else: self.get_logger().info(f"[CENTER] Reached Odometry Target")
                
                if self.first_cell_evaluated:
                    if dist_moved > (self.cell_size * 0.5):
                        if self.heading == 0: self.grid_y += 1   
                        elif self.heading == 1: self.grid_x += 1 
                        elif self.heading == 2: self.grid_y -= 1 
                        elif self.heading == 3: self.grid_x -= 1 
                        self.get_logger().info(f"[GRID] Position Updated -> New Cell({self.grid_x}, {self.grid_y})")
                        self.is_short_movement = False
                    else:
                        self.get_logger().warn(f"[GRID] Short movement. Staying in Cell({self.grid_x}, {self.grid_y})")
                        self.is_short_movement = True
                    
                    self.grid_x = max(0, min(self.flood_fill.size - 1, self.grid_x))
                    self.grid_y = max(0, min(self.flood_fill.size - 1, self.grid_y))
                else:
                    self.first_cell_evaluated = True
                    self.is_short_movement = False

                self.state = 'EVALUATE_CELL' 
                
                if reached_front:
                    vel_msg.linear.x = 0.0
                else:
                    vel_msg.linear.x = self.forward_speed
                    yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
                    vel_msg.angular.z = max(-0.5, min(0.5, (self.kp_heading * yaw_error) + (self.kd_heading * (yaw_error - self.prev_yaw_error))))
            else:
                yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
                angular_correction = (self.kp_heading * yaw_error) + (self.kd_heading * (yaw_error - self.prev_yaw_error))
                self.prev_yaw_error = yaw_error
                vel_msg.angular.z = max(-0.5, min(0.5, angular_correction))

                remaining_odom_dist = active_odom_limit - dist_moved
                
                if self.dist_m < self.braking_distance:
                    linear_error = self.dist_m - self.front_stop_distance
                    speed_front = (self.kp_linear * linear_error) + (self.kd_linear * (linear_error - self.prev_linear_error))
                    self.prev_linear_error = linear_error
                else:
                    speed_front = self.forward_speed
                    self.prev_linear_error = self.dist_m - self.front_stop_distance

                if 0 < remaining_odom_dist < self.odom_braking_distance:
                    speed_odom = (remaining_odom_dist / self.odom_braking_distance) * self.forward_speed
                else:
                    speed_odom = self.forward_speed

                speed_side = self.forward_speed
                if (not self.right_wall_blocked) or (not self.left_wall_blocked):
                    speed_side = 0.25  
                    
                calculated_speed = min(speed_front, speed_odom, speed_side)
                vel_msg.linear.x = max(self.min_forward_speed, calculated_speed)

        elif self.state == 'TURNING':
            yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
            if abs(yaw_error) > 0.05:
                angular_z = 1.8 * yaw_error
                if angular_z > self.turn_speed: angular_z = self.turn_speed
                elif angular_z < -self.turn_speed: angular_z = -self.turn_speed
                if abs(angular_z) < self.min_turn_speed: angular_z = self.min_turn_speed if angular_z > 0 else -self.min_turn_speed
                vel_msg.angular.z = angular_z
            else:
                vel_msg.angular.z = 0.0
                self.next_state_after_settle = 'MOVE_FORWARD'
                self.state = 'SETTLE'

        elif self.state == 'SETTLE':
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.settle_counter += 1
            if self.settle_counter > 15: 
                self.state = self.next_state_after_settle
                self.settle_counter = 0
                self.prev_yaw_error = 0.0
                self.prev_linear_error = 0.0 
                
                # [FIX]: Reset odometry anchors to absorb any translational drift that occurred during rotation
                self.start_x = self.odom_x
                self.start_y = self.odom_y

        self.cmd_pub.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MazeNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()