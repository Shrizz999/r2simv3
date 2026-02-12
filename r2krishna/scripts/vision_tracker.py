#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
from geometry_msgs.msg import Twist
from ros_gz_interfaces.msg import Contacts
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import math

# --- STATES ---
STATE_SEARCH = 0      
STATE_APPROACH = 1    
STATE_ALIGNING = 2    
STATE_CLIMBING = 3    

class LevelManager(Node):
    def __init__(self):
        super().__init__('level_manager')
        self.get_logger().info('--- R2KRISHNA: STRICT SEQUENCE (APPROACH -> ALIGN -> CLIMB) ---')

        # --- SETTINGS ---
        self.invert_steering = True   
        self.invert_search = False    
        self.linear_speed = 0.5       
        self.search_speed = 0.5
        self.steering_gain = 0.005    

        # --- CLIMB SETTINGS ---
        self.climb_torque = 1.0       
        self.alignment_tolerance = 40 
        self.virtual_bumper_area = 45000 # Increased to ensure we are REALLY close before climbing

        # --- LEVEL TRACKING ---
        self.current_level = 0        
        self.target_level_seen = None 
        self.valid_target_locked = False 
        self.align_start_time = 0.0
        self.bumper_hit = False
        self.current_area = 0

        # --- COLOR RANGES (Strict Saturation > 100 to ignore Grey Floor) ---
        self.color_200_lower = np.array([35, 150, 50]); self.color_200_upper = np.array([85, 255, 255])
        self.color_400_lower = np.array([90, 100, 50]); self.color_400_upper = np.array([130, 255, 255])
        self.color_600_lower = np.array([0, 0, 0]); self.color_600_upper = np.array([180, 255, 40])
        self.red1_lower = np.array([0, 120, 50]); self.red1_upper = np.array([10, 255, 255])
        self.red2_lower = np.array([170, 120, 50]); self.red2_upper = np.array([180, 255, 255])

        # --- SENSORS ---
        self.pitch_threshold = 0.12 
        self.is_tilting = False
        self.sonar_dist = 99.9
        
        # --- STATE ---
        self.state = STATE_SEARCH
        self.block_x = None
        self.last_known_direction = -1 
        self.cv_image = None
        
        # --- ROS ---
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(LaserScan, '/ultrasonic', self.sonar_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Contacts, '/bumper_states', self.bumper_callback, 10)
        
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_front = self.create_publisher(Twist, '/cmd_vel_front', 10)

        self.br = CvBridge()
        self.create_timer(0.05, self.control_loop) 

    def drive(self, fwd, rot, engage_climbers=False):
        cmd = Twist()
        cmd.linear.x = float(fwd)
        cmd.angular.z = float(rot)
        self.pub_vel.publish(cmd)
        
        if engage_climbers:
            self.pub_front.publish(cmd)
        else:
            self.pub_front.publish(Twist())

    def bumper_callback(self, msg):
        self.bumper_hit = True if len(msg.contacts) > 0 else False

    def sonar_callback(self, msg):
        valid = [r for r in msg.ranges if not np.isinf(r) and not np.isnan(r)]
        if len(valid) > 0: self.sonar_dist = min(valid)

    def imu_callback(self, msg):
        q = msg.orientation
        sinp = 2 * (q.w * q.y - q.z * q.x)
        self.current_pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi/2, sinp)

    def get_contour_data(self, mask):
        # Noise Filter
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area > 3000: # Filter small noise
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    return int(M["m10"] / M["m00"]), area
        return None, 0

    def image_callback(self, msg):
        try:
            frame = self.br.imgmsg_to_cv2(msg, "bgr8")
            self.cv_image = frame
        except: return
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        height, width, _ = frame.shape

        x_200, a_200 = self.get_contour_data(cv2.inRange(hsv, self.color_200_lower, self.color_200_upper))
        x_400, a_400 = self.get_contour_data(cv2.inRange(hsv, self.color_400_lower, self.color_400_upper))
        x_600, a_600 = self.get_contour_data(cv2.inRange(hsv, self.color_600_lower, self.color_600_upper))
        x_red, a_red = self.get_contour_data(cv2.inRange(hsv, self.red1_lower, self.red1_upper) + cv2.inRange(hsv, self.red2_lower, self.red2_upper))

        self.valid_target_locked = False; self.block_x = None; self.target_level_seen = None; self.current_area = 0
        
        # UNIVERSAL SEARCH (Any Block)
        if self.current_level == 0:
            if x_red: self.block_x = x_red; self.target_level_seen = 999; self.current_area = a_red
            elif x_200: self.block_x = x_200; self.target_level_seen = 200; self.current_area = a_200
            elif x_400: self.block_x = x_400; self.target_level_seen = 400; self.current_area = a_400
            elif x_600: self.block_x = x_600; self.target_level_seen = 600; self.current_area = a_600
        else:
            if self.current_level == 200 and x_400: 
                self.block_x = x_400; self.target_level_seen = 400; self.current_area = a_400
            elif self.current_level == 400 and x_600:
                self.block_x = x_600; self.target_level_seen = 600; self.current_area = a_600
            elif self.current_level >= 600 and x_red:
                self.block_x = x_red; self.target_level_seen = 999; self.current_area = a_red

        if self.block_x is not None:
            self.valid_target_locked = True
            self.last_known_direction = 1 if (320 - self.block_x) > 0 else -1
            cv2.circle(frame, (self.block_x, height//2), 10, (0, 255, 0), 3)

        # Debug Overlay
        state_name = ["SEARCH", "APPROACH", "ALIGNING", "CLIMBING"][self.state]
        status = f"STATE: {state_name} | AREA: {int(self.current_area)}"
        cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imshow("Main Vision", frame); cv2.waitKey(1)

    def control_loop(self):
        # STATE: CLIMBING (6WD ON, Torque 1.0)
        if self.state == STATE_CLIMBING:
            self.drive(self.climb_torque, 0.0, True) 
            if abs(self.current_pitch) > self.pitch_threshold: self.is_tilting = True
            if self.is_tilting and abs(self.current_pitch) < 0.08:
                self.current_level = self.target_level_seen
                self.get_logger().info(f"✅ LEVEL COMPLETE: {self.current_level}")
                self.state = STATE_SEARCH; self.is_tilting = False
                self.drive(0.0, 0.0, False); time.sleep(1.0)
            return

        # STATE: ALIGNING (4WD ONLY)
        if self.state == STATE_ALIGNING:
            if not self.valid_target_locked: self.state = STATE_SEARCH; return
            err = 320 - self.block_x
            
            # TRIGGER CLIMB: Only if error is SMALL (< 40)
            # OR if we physically hit the bumper (Emergency Climb)
            if self.bumper_hit or (abs(err) < self.alignment_tolerance and self.current_area > self.virtual_bumper_area): 
                self.get_logger().info("✅ ALIGNED & CLOSE! STARTING 6WD CLIMB.")
                self.state = STATE_CLIMBING; return
            
            # Rotate in place (NO 6WD)
            turn_cmd = self.steering_gain * err * 2.0
            if 0 < turn_cmd < 0.2: turn_cmd = 0.2 
            
            # Small forward creep (0.1) to get closer, but NO 6WD yet
            self.drive(0.1, -turn_cmd if self.invert_steering else turn_cmd, False)
            
            # Timeout Failsafe: If aligning takes > 4s, force climb
            if time.time() - self.align_start_time > 4.0:
                 self.get_logger().warn("⚠️ ALIGN TIMEOUT. FORCING CLIMB.")
                 self.state = STATE_CLIMBING; return
            return

        # STATE: APPROACH (4WD ONLY)
        if self.valid_target_locked:
            # STRICT RULE: NEVER CLIMB FROM APPROACH. ALWAYS GO TO ALIGN.
            
            err = 320 - self.block_x
            turn_cmd = self.steering_gain * err
            self.drive(self.linear_speed, -turn_cmd if self.invert_steering else turn_cmd, False)
            
            # Stop to Align (Sonar < 0.7 OR Visual Area > 20000)
            if self.sonar_dist < 0.7 or self.current_area > 20000:
                self.get_logger().info(f"🛑 TARGET CLOSE (Area {int(self.current_area)}). STARTING ALIGNMENT.")
                self.drive(0.0, 0.0, False)
                self.state = STATE_ALIGNING; self.align_start_time = time.time()
        else:
            self.drive(0.0, self.search_speed * self.last_known_direction, False)

def main(args=None):
    rclpy.init(args=args); node = LevelManager()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown(); cv2.destroyAllWindows()

if __name__ == '__main__': main()
