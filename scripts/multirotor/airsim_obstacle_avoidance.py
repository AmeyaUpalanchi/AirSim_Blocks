#!/usr/bin/env python3
"""
AirSim Multi-Sensor Obstacle Avoidance System - 30M SQUARE MISSION

Advanced obstacle avoidance with 30m square mission pattern.
The drone will fly in a square pattern while avoiding obstacles using
Camera + LiDAR + IMU sensor fusion.

Mission Pattern:
- Start at (0, 0, -5)
- Waypoint 1: (30, 0, -5)
- Waypoint 2: (30, 30, -5)  
- Waypoint 3: (0, 30, -5)
- Waypoint 4: (0, 0, -5) - Back to start
- Repeat mission

Dependencies:
    - airsim
    - opencv-python
    - numpy
    - ultralytics (optional, for YOLO11 detection)
"""

import airsim
import cv2
import numpy as np
import time
import threading
import queue
import math
import asyncio
from collections import deque
from typing import Dict, List, Tuple, Optional, Any

class SquareMissionObstacleAvoidance:
    """Multi-sensor obstacle avoidance system with 30m square mission pattern"""
    
    def __init__(self, use_yolo: bool = True, debug_mode: bool = False, square_size: float = 30.0):
        # Initialize AirSim connection
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        
        # Reset to known state
        self.client.reset()
        time.sleep(2)
        self.client.enableApiControl(True)
        
        # Ensure we have a proper event loop for the main thread
        try:
            asyncio.get_event_loop()
        except RuntimeError:
            asyncio.set_event_loop(asyncio.new_event_loop())
        
        self.use_yolo = use_yolo
        self.debug_mode = debug_mode
        self.yolo_model = None
        self.square_size = square_size
        
        if use_yolo:
            self._initialize_yolo()
        
        # Obstacle detection storage
        self.obstacles = {
            'camera': [],
            'lidar': [],
            'fused': []
        }
        
        # Navigation state
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_orientation = np.array([0.0, 0.0, 0.0])
        
        # SQUARE MISSION PARAMETERS
        self.mission_altitude = -5.0  # 5m altitude
        self.waypoints = self._generate_square_waypoints()
        self.current_waypoint_index = 0
        self.target_position = self.waypoints[0]
        self.waypoint_reached_distance = 3.0  # Distance to consider waypoint reached
        self.mission_loops_completed = 0
        self.max_mission_loops = 3  # Complete 3 full squares
        
        # Enhanced control parameters for square mission
        self.safe_distance = 8.0          # Reduced safe distance for 5m altitude
        self.critical_distance = 4.0      # Critical zone for emergency maneuvers
        self.max_speed = 2.5              # Slightly faster for longer distances
        self.avoidance_force = 6.0        # Strong avoidance force
        self.waypoint_approach_speed = 1.5 # Slower speed when approaching waypoints
        
        # Mission tracking
        self.mission_start_time = 0
        self.waypoint_start_time = 0
        self.total_distance_traveled = 0.0
        self.previous_position = np.array([0.0, 0.0, 0.0])
        
        # Debug visualization
        self.show_debug_images = debug_mode
        self.last_debug_image = None
        
        # System state
        self.avoidance_active = False
        self.sensors_active = False
        self.mission_completed = False
        
        print("🟦 30m Square Mission Obstacle Avoidance System initialized")
        print(f"📐 Square size: {self.square_size}m x {self.square_size}m")
        print(f"🎯 Mission altitude: {abs(self.mission_altitude)}m")
        print(f"🔄 Will complete {self.max_mission_loops} full loops")
        print(f"📍 Waypoints: {len(self.waypoints)}")
        for i, wp in enumerate(self.waypoints):
            print(f"   WP{i+1}: ({wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f})")
        if self.yolo_model:
            print("🤖 YOLO11n object detection enabled")
    
    def _generate_square_waypoints(self) -> List[np.ndarray]:
        """Generate waypoints for 30m square mission"""
        waypoints = [
            np.array([0.0, 0.0, self.mission_altitude]),                    # Start/End
            np.array([self.square_size, 0.0, self.mission_altitude]),      # Corner 1 - East
            np.array([self.square_size, self.square_size, self.mission_altitude]), # Corner 2 - Northeast  
            np.array([0.0, self.square_size, self.mission_altitude]),      # Corner 3 - North
            np.array([0.0, 0.0, self.mission_altitude])                    # Back to start
        ]
        return waypoints
    
    def _initialize_yolo(self):
        """Initialize YOLO11 object detection"""
        try:
            from ultralytics import YOLO
            self.yolo_model = YOLO('../models/yolo11n.pt')
            if self.debug_mode:
                print("YOLO11n loaded successfully")
        except ImportError:
            if self.debug_mode:
                print("Ultralytics not available - install with: pip install ultralytics")
        except Exception as e:
            if self.debug_mode:
                print(f"YOLO initialization failed: {e}")
    
    def start_sensors(self):
        """Start simplified sensor collection"""
        self.sensors_active = True
        print("📡 Sensors initialized (direct mode)")
    
    def _get_sensor_data(self):
        """Get all sensor data synchronously"""
        sensor_data = {
            'camera_obstacles': [],
            'lidar_obstacles': [],
            'position_updated': False
        }
        
        try:
            # Get camera data
            try:
                responses = self.client.simGetImages([
                    airsim.ImageRequest("low_res", airsim.ImageType.Scene, False, False),
                    airsim.ImageRequest("low_res", airsim.ImageType.DepthVis, True, False)
                ])
                
                if len(responses) >= 2:
                    rgb_img = self._process_rgb_image(responses[0])
                    depth_img = self._process_depth_image(responses[1])
                    sensor_data['camera_obstacles'] = self._detect_visual_obstacles(rgb_img, depth_img)
                    
            except Exception as e:
                if self.debug_mode:
                    print(f"Camera error: {e}")
            
            # Get LiDAR data
            try:
                lidar_data = self.client.getLidarData("Lidar1")
                if len(lidar_data.point_cloud) > 0:
                    sensor_data['lidar_obstacles'] = self._process_lidar_data(lidar_data)
                    
            except Exception as e:
                if self.debug_mode:
                    print(f"LiDAR error: {e}")
            
            # Get position data and update mission tracking
            try:
                state = self.client.getMultirotorState()
                self.previous_position = self.current_position.copy()
                self.current_position = np.array([
                    state.kinematics_estimated.position.x_val,
                    state.kinematics_estimated.position.y_val,
                    state.kinematics_estimated.position.z_val
                ])
                
                # Update total distance traveled
                if np.any(self.previous_position):
                    distance_step = np.linalg.norm(self.current_position - self.previous_position)
                    self.total_distance_traveled += distance_step
                
                sensor_data['position_updated'] = True
                
            except Exception as e:
                if self.debug_mode:
                    print(f"State error: {e}")
        
        except Exception as e:
            if self.debug_mode:
                print(f"Sensor data error: {e}")
        
        return sensor_data
    
    def _process_rgb_image(self, response) -> np.ndarray:
        """Convert AirSim image response to numpy array"""
        try:
            img_data = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
            img_rgb = img_data.reshape(response.height, response.width, 3)
            return img_rgb
        except Exception:
            return np.zeros((144, 256, 3), dtype=np.uint8)
    
    def _process_depth_image(self, response) -> np.ndarray:
        """Convert depth response to distance array in meters"""
        try:
            img_data = np.array(response.image_data_float, dtype=np.float32)
            img_depth = img_data.reshape(response.height, response.width)
            
            # Clamp invalid values
            img_depth[img_depth > 100] = 100
            img_depth[img_depth < 0.1] = 100
            
            return img_depth
        except Exception:
            return np.full((144, 256), 100.0, dtype=np.float32)
    
    def _detect_visual_obstacles(self, rgb_img: np.ndarray, depth_img: np.ndarray) -> List[Dict]:
        """Detect obstacles using camera vision with debug visualization"""
        try:
            if self.yolo_model is not None:
                obstacles = self._yolo_obstacle_detection(rgb_img, depth_img)
            else:
                obstacles = self._basic_obstacle_detection(rgb_img, depth_img)
            
            # Create enhanced debug visualization for square mission
            if self.show_debug_images:
                debug_img = self._create_mission_debug_visualization(rgb_img, depth_img, obstacles)
                self.last_debug_image = debug_img
                
                # Show debug window
                cv2.imshow('Square Mission Obstacle Avoidance', debug_img)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("Debug window quit requested")
                    self.avoidance_active = False
                elif key == ord('n'):
                    print("Manual next waypoint requested")
                    self._advance_to_next_waypoint()
                elif key == ord('r'):
                    print("Mission reset requested")
                    self._reset_mission()
            
            return obstacles
        except Exception as e:
            if self.debug_mode:
                print(f"Visual detection error: {e}")
            return []
    
    def _basic_obstacle_detection(self, rgb_img: np.ndarray, depth_img: np.ndarray) -> List[Dict]:
        """Enhanced basic obstacle detection for square mission"""
        obstacles = []
        
        try:
            # Use larger detection distance for square mission
            close_mask = depth_img < self.safe_distance
            
            if np.any(close_mask):
                # Morphological operations to clean up the mask
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                close_mask = cv2.morphologyEx(close_mask.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
                
                # Find contours of close objects
                close_uint8 = (close_mask * 255).astype(np.uint8)
                contours, _ = cv2.findContours(close_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > 40:  # Adjusted threshold for square mission
                        # Get bounding rectangle
                        x, y, w, h = cv2.boundingRect(contour)
                        
                        # Get contour center
                        M = cv2.moments(contour)
                        if M["m00"] != 0:
                            center_x = int(M["m10"] / M["m00"])
                            center_y = int(M["m01"] / M["m00"])
                            
                            if (0 <= center_x < depth_img.shape[1] and 
                                0 <= center_y < depth_img.shape[0]):
                                
                                # Sample depth in region around center
                                region_size = 4
                                y_start = max(0, center_y - region_size)
                                y_end = min(depth_img.shape[0], center_y + region_size + 1)
                                x_start = max(0, center_x - region_size)
                                x_end = min(depth_img.shape[1], center_x + region_size + 1)
                                
                                depth_region = depth_img[y_start:y_end, x_start:x_end]
                                valid_depths = depth_region[depth_region < 100]
                                
                                if len(valid_depths) > 0:
                                    distance = np.mean(valid_depths)
                                    
                                    if distance > 0.5 and distance < self.safe_distance:
                                        world_pos = self._pixel_to_world(center_x, center_y, distance)
                                        
                                        obstacle = {
                                            'type': 'camera_basic',
                                            'object_class': 'obstacle',
                                            'confidence': 0.8,
                                            'distance': distance,
                                            'position': world_pos,
                                            'bbox': [x, y, x + w, y + h],
                                            'area': area,
                                            'center': [center_x, center_y]
                                        }
                                        obstacles.append(obstacle)
                                        
                                        if self.debug_mode:
                                            print(f"Basic detected: obstacle at {distance:.1f}m (area: {area:.0f})")
        
        except Exception as e:
            if self.debug_mode:
                print(f"Basic detection error: {e}")
        
        return obstacles
    
    def _yolo_obstacle_detection(self, rgb_img: np.ndarray, depth_img: np.ndarray) -> List[Dict]:
        """YOLO object detection optimized for square mission"""
        obstacles = []
        
        try:
            results = self.yolo_model(rgb_img, verbose=False)
            
            if results[0].boxes is not None:
                for box in results[0].boxes:
                    confidence = float(box.conf[0])
                    
                    # Adjusted confidence threshold for square mission
                    if confidence > 0.25:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        class_id = int(box.cls[0])
                        object_name = self.yolo_model.names[class_id]
                        
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)
                        
                        if (0 <= center_x < depth_img.shape[1] and 
                            0 <= center_y < depth_img.shape[0]):
                            
                            # Sample depth in larger region for better accuracy
                            region_size = 6
                            y_start = max(0, center_y - region_size)
                            y_end = min(depth_img.shape[0], center_y + region_size + 1)
                            x_start = max(0, center_x - region_size)
                            x_end = min(depth_img.shape[1], center_x + region_size + 1)
                            
                            depth_region = depth_img[y_start:y_end, x_start:x_end]
                            valid_depths = depth_region[depth_region < 100]
                            
                            if len(valid_depths) > 0:
                                distance = np.mean(valid_depths)
                                
                                if distance > 0.5 and distance < 60:  # Extended range for square mission
                                    world_pos = self._pixel_to_world(center_x, center_y, distance)
                                    
                                    obstacle = {
                                        'type': 'camera_yolo',
                                        'object_class': object_name,
                                        'confidence': confidence,
                                        'distance': distance,
                                        'position': world_pos,
                                        'bbox': [int(x1), int(y1), int(x2), int(y2)],
                                        'size': (x2 - x1) * (y2 - y1),
                                        'center': [center_x, center_y]
                                    }
                                    obstacles.append(obstacle)
                                    
                                    if self.debug_mode:
                                        print(f"YOLO detected: {object_name} at {distance:.1f}m (conf: {confidence:.2f})")
        
        except Exception as e:
            if self.debug_mode:
                print(f"YOLO detection error: {e}")
        
        return obstacles
    
    def _pixel_to_world(self, pixel_x: int, pixel_y: int, distance: float) -> np.ndarray:
        """Convert pixel coordinates to world coordinates"""
        # Camera intrinsics for AirSim low_res camera (256x144)
        fx, fy = 256.0, 144.0
        cx, cy = 128.0, 72.0
        
        # Convert to camera coordinates
        x_cam = (pixel_x - cx) * distance / fx
        y_cam = (pixel_y - cy) * distance / fy
        z_cam = distance
        
        # Transform to world coordinates (camera facing forward)
        world_pos = np.array([z_cam, -x_cam, y_cam])
        return world_pos
    
    def _process_lidar_data(self, lidar_data) -> List[Dict]:
        """Process LiDAR point cloud for obstacle detection"""
        obstacles = []
        
        try:
            points = np.array(lidar_data.point_cloud)
            
            if len(points) >= 3:
                points = points.reshape(-1, 3)
                distances = np.linalg.norm(points, axis=1)
                # Use larger detection radius for square mission
                valid_points = points[distances < self.safe_distance * 1.5]
                
                if len(valid_points) > 0:
                    obstacles = self._cluster_lidar_points(valid_points)
        
        except Exception as e:
            if self.debug_mode:
                print(f"LiDAR processing error: {e}")
        
        return obstacles
    
    def _cluster_lidar_points(self, points: np.ndarray) -> List[Dict]:
        """Cluster LiDAR points into discrete obstacles"""
        obstacles = []
        
        try:
            cluster_distance = 1.2  # Slightly larger clustering for square mission
            used_points = np.zeros(len(points), dtype=bool)
            
            for i, point in enumerate(points):
                if used_points[i]:
                    continue
                
                distances = np.linalg.norm(points - point, axis=1)
                cluster_mask = distances < cluster_distance
                cluster_points = points[cluster_mask]
                used_points[cluster_mask] = True
                
                if len(cluster_points) >= 2:
                    center = np.mean(cluster_points, axis=0)
                    distance = np.linalg.norm(center)
                    size = np.max(np.linalg.norm(cluster_points - center, axis=1))
                    
                    obstacle = {
                        'type': 'lidar',
                        'position': center,
                        'distance': distance,
                        'size': size,
                        'point_count': len(cluster_points),
                        'confidence': min(1.0, len(cluster_points) / 8.0)
                    }
                    obstacles.append(obstacle)
        
        except Exception as e:
            if self.debug_mode:
                print(f"LiDAR clustering error: {e}")
        
        return obstacles
    
    def _create_mission_debug_visualization(self, rgb_img: np.ndarray, depth_img: np.ndarray, obstacles: List[Dict]) -> np.ndarray:
        """Create enhanced debug visualization for square mission"""
        debug_img = rgb_img.copy()
        
        try:
            # Draw obstacles
            for obstacle in obstacles:
                bbox = obstacle.get('bbox', [])
                if len(bbox) == 4:
                    x1, y1, x2, y2 = bbox
                    
                    # Choose color based on distance
                    distance = obstacle.get('distance', 100)
                    if distance < self.critical_distance:
                        color = (0, 0, 255)  # Red for critical
                    elif distance < self.safe_distance:
                        color = (0, 165, 255)  # Orange for caution
                    else:
                        color = (0, 255, 0)  # Green for safe
                    
                    # Draw bounding box
                    cv2.rectangle(debug_img, (x1, y1), (x2, y2), color, 2)
                    
                    # Draw object info
                    obj_class = obstacle.get('object_class', 'unknown')
                    confidence = obstacle.get('confidence', 0)
                    label = f"{obj_class} {distance:.1f}m"
                    
                    # Background for text
                    (text_width, text_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
                    cv2.rectangle(debug_img, (x1, y1 - text_height - 5), (x1 + text_width, y1), color, -1)
                    
                    # Text
                    cv2.putText(debug_img, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                    
                    # Draw center point
                    center = obstacle.get('center', [int((x1+x2)/2), int((y1+y2)/2)])
                    cv2.circle(debug_img, tuple(center), 3, color, -1)
            
            # Enhanced status text for square mission
            current_time = time.time()
            mission_time = current_time - self.mission_start_time if self.mission_start_time > 0 else 0
            waypoint_time = current_time - self.waypoint_start_time if self.waypoint_start_time > 0 else 0
            
            # Line 1: Mission info
            mission_text = f"🟦 SQUARE MISSION | Loop: {self.mission_loops_completed}/{self.max_mission_loops} | WP: {self.current_waypoint_index+1}/{len(self.waypoints)}"
            cv2.putText(debug_img, mission_text, (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            # Line 2: Position and target
            distance_to_target = np.linalg.norm(self.current_position - self.target_position)
            pos_text = f"📍 Pos: ({self.current_position[0]:.1f},{self.current_position[1]:.1f}) | Target: {distance_to_target:.1f}m"
            cv2.putText(debug_img, pos_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            # Line 3: Obstacles and timing
            obstacles_text = f"🚧 Obstacles: {len(obstacles)} | Mission: {mission_time:.0f}s | WP: {waypoint_time:.0f}s"
            cv2.putText(debug_img, obstacles_text, (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            # Line 4: Controls
            controls_text = f"⌨️  Controls: 'q'=quit, 'n'=next waypoint, 'r'=reset mission"
            cv2.putText(debug_img, controls_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (200, 200, 200), 1)
            
            # Create side-by-side view with depth
            depth_colored = cv2.applyColorMap(np.uint8(depth_img / 40 * 255), cv2.COLORMAP_JET)
            
            # Draw waypoint indicator on depth image
            wp_color = (255, 255, 255)
            cv2.putText(depth_colored, f"WP{self.current_waypoint_index+1}", (10, 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, wp_color, 2)
            
            combined = np.hstack([debug_img, depth_colored])
            
            return combined
            
        except Exception as e:
            if self.debug_mode:
                print(f"Debug visualization error: {e}")
            return rgb_img
    
    def _fuse_obstacles(self, camera_obstacles: List[Dict], lidar_obstacles: List[Dict]) -> List[Dict]:
        """Combine obstacles from multiple sensors"""
        fused_obstacles = []
        
        try:
            # Start with LiDAR obstacles (most reliable for distance)
            for lidar_obs in lidar_obstacles:
                fused_obs = lidar_obs.copy()
                fused_obs['sensors'] = ['lidar']
                
                # Look for matching camera obstacles
                for cam_obs in camera_obstacles:
                    if self._obstacles_match(lidar_obs, cam_obs):
                        fused_obs['sensors'].append('camera')
                        fused_obs['object_class'] = cam_obs.get('object_class', 'unknown')
                        fused_obs['visual_confidence'] = cam_obs.get('confidence', 0.0)
                        break
                
                fused_obstacles.append(fused_obs)
            
            # Add camera-only obstacles
            for cam_obs in camera_obstacles:
                if not any(self._obstacles_match(lidar_obs, cam_obs) for lidar_obs in lidar_obstacles):
                    fused_obs = cam_obs.copy()
                    fused_obs['sensors'] = ['camera']
                    fused_obstacles.append(fused_obs)
        
        except Exception as e:
            if self.debug_mode:
                print(f"Obstacle fusion error: {e}")
        
        return fused_obstacles
    
    def _obstacles_match(self, obs1: Dict, obs2: Dict, threshold: float = 2.5) -> bool:
        """Check if obstacles represent the same object"""
        try:
            pos1 = obs1.get('position', np.array([0, 0, 0]))
            pos2 = obs2.get('position', np.array([0, 0, 0]))
            distance = np.linalg.norm(pos1 - pos2)
            return distance < threshold
        except Exception:
            return False
    
    def _advance_to_next_waypoint(self):
        """Advance to the next waypoint in the square mission"""
        self.current_waypoint_index += 1
        
        if self.current_waypoint_index >= len(self.waypoints):
            # Completed one loop of the square
            self.mission_loops_completed += 1
            self.current_waypoint_index = 0
            
            print(f"🔄 COMPLETED SQUARE LOOP {self.mission_loops_completed}/{self.max_mission_loops}")
            
            if self.mission_loops_completed >= self.max_mission_loops:
                print("🎯 MISSION COMPLETED! All square loops finished.")
                self.mission_completed = True
                return
        
        # Set new target
        self.target_position = self.waypoints[self.current_waypoint_index]
        self.waypoint_start_time = time.time()
        
        print(f"📍 NEW WAYPOINT {self.current_waypoint_index + 1}: ({self.target_position[0]:.1f}, {self.target_position[1]:.1f}, {self.target_position[2]:.1f})")
    
    def _reset_mission(self):
        """Reset mission to start"""
        self.current_waypoint_index = 0
        self.mission_loops_completed = 0
        self.target_position = self.waypoints[0]
        self.mission_start_time = time.time()
        self.waypoint_start_time = time.time()
        self.total_distance_traveled = 0.0
        self.mission_completed = False
        print("🔄 MISSION RESET - Starting from waypoint 1")
    
    def start_square_mission(self):
        """Start the 30m square obstacle avoidance mission"""
        
        print("🟦" + "="*60)
        print("🚁 STARTING 30M SQUARE OBSTACLE AVOIDANCE MISSION")
        print("🟦" + "="*60)
        print(f"📐 Mission: {self.square_size}m x {self.square_size}m square")
        print(f"🎯 Altitude: {abs(self.mission_altitude)}m")
        print(f"🔄 Loops to complete: {self.max_mission_loops}")
        print(f"📏 Safe distance: {self.safe_distance}m")
        print(f"⚡ Max speed: {self.max_speed}m/s")
        print()
        
        # Enhanced takeoff sequence
        print("🚀 Enhanced takeoff sequence...")
        try:
            # Arm the drone
            self.client.armDisarm(True)
            time.sleep(1)
            
            # Takeoff
            self.client.takeoffAsync().join()
            time.sleep(3)
            
            # Move to mission altitude
            print(f"📏 Moving to mission altitude: {abs(self.mission_altitude)}m...")
            self.client.moveToZAsync(self.mission_altitude, 3).join()
            time.sleep(3)
            
            # Get initial position
            state = self.client.getMultirotorState()
            self.current_position = np.array([
                state.kinematics_estimated.position.x_val,
                state.kinematics_estimated.position.y_val,
                state.kinematics_estimated.position.z_val
            ])
            print(f"✅ Mission altitude reached. Position: ({self.current_position[0]:.1f}, {self.current_position[1]:.1f}, {self.current_position[2]:.1f})")
            
        except Exception as e:
            print(f"❌ Takeoff error: {e}")
            print("🔄 Attempting to continue anyway...")
        
        # Initialize mission tracking
        self.mission_start_time = time.time()
        self.waypoint_start_time = time.time()
        self.target_position = self.waypoints[0]
        
        # Start sensors
        self.start_sensors()
        time.sleep(1)
        
        self.avoidance_active = True
        print("🟦 SQUARE MISSION ACTIVE!")
        print(f"📍 First waypoint: ({self.target_position[0]:.1f}, {self.target_position[1]:.1f}, {self.target_position[2]:.1f})")
        print("🟦" + "="*60)
        
        try:
            self._square_mission_navigation_loop()
        except KeyboardInterrupt:
            print("\n⚠️  Mission interrupted by user")
        finally:
            self._stop_mission()
    
    def _square_mission_navigation_loop(self):
        """Main navigation loop for square mission"""
        
        while self.avoidance_active and not self.mission_completed:
            try:
                # Get all sensor data synchronously
                sensor_data = self._get_sensor_data()
                
                # Update obstacles
                self.obstacles['camera'] = sensor_data['camera_obstacles']
                self.obstacles['lidar'] = sensor_data['lidar_obstacles']
                self.obstacles['fused'] = self._fuse_obstacles(
                    sensor_data['camera_obstacles'], 
                    sensor_data['lidar_obstacles']
                )
                
                # Check if current waypoint reached
                distance_to_target = np.linalg.norm(self.current_position - self.target_position)
                
                if distance_to_target < self.waypoint_reached_distance:
                    current_time = time.time()
                    waypoint_time = current_time - self.waypoint_start_time
                    print(f"✅ WAYPOINT {self.current_waypoint_index + 1} REACHED in {waypoint_time:.1f}s!")
                    
                    self._advance_to_next_waypoint()
                    
                    if self.mission_completed:
                        break
                    
                    # Brief pause at waypoint
                    time.sleep(0.5)
                    continue
                
                # Calculate enhanced velocity for square mission
                final_velocity = self._calculate_square_mission_velocity()
                
                # Send movement command
                try:
                    self.client.moveByVelocityAsync(
                        final_velocity[0],
                        final_velocity[1], 
                        final_velocity[2],
                        0.4  # Responsive control for square mission
                    )
                except Exception as e:
                    if self.debug_mode:
                        print(f"Movement command error: {e}")
                
                # Enhanced debug output for square mission
                if self.debug_mode:
                    current_time = time.time()
                    mission_time = current_time - self.mission_start_time
                    waypoint_time = current_time - self.waypoint_start_time
                    
                    print(f"🟦 SQUARE MISSION STATUS:")
                    print(f"   📍 Pos: ({self.current_position[0]:.1f}, {self.current_position[1]:.1f}, {self.current_position[2]:.1f})")
                    print(f"   🎯 WP{self.current_waypoint_index+1}: {distance_to_target:.1f}m | Loop: {self.mission_loops_completed+1}/{self.max_mission_loops}")
                    print(f"   ⏱️  Mission: {mission_time:.0f}s | WP: {waypoint_time:.0f}s | Distance: {self.total_distance_traveled:.1f}m")
                    print(f"   🚧 Obstacles: {len(self.obstacles['fused'])} | Speed: {np.linalg.norm(final_velocity):.1f}m/s")
                    if len(self.obstacles['fused']) > 0:
                        closest_obstacle = min(self.obstacles['fused'], key=lambda x: x.get('distance', float('inf')))
                        print(f"   ⚠️  Closest: {closest_obstacle.get('object_class', 'unknown')} at {closest_obstacle.get('distance', 0):.1f}m")
                    print()
                
                time.sleep(0.2)  # 5 Hz control loop
                
            except Exception as e:
                if self.debug_mode:
                    print(f"Navigation error: {e}")
                time.sleep(0.5)
        
        if self.mission_completed:
            print("🎉 SQUARE MISSION SUCCESSFULLY COMPLETED!")
            print(f"📊 Mission Statistics:")
            print(f"   🔄 Loops completed: {self.mission_loops_completed}")
            print(f"   📏 Total distance: {self.total_distance_traveled:.1f}m")
            print(f"   ⏱️  Total time: {time.time() - self.mission_start_time:.1f}s")
    
    def _calculate_square_mission_velocity(self) -> np.ndarray:
        """Calculate velocity for square mission with enhanced obstacle avoidance"""
        
        # Calculate desired velocity toward current waypoint
        direction_to_target = self.target_position - self.current_position
        distance_to_target = np.linalg.norm(direction_to_target)
        
        if distance_to_target > 0.1:
            direction_to_target = direction_to_target / distance_to_target
            
            # Adaptive speed based on distance to waypoint and obstacles
            if distance_to_target < 5.0:
                # Slow down when approaching waypoint
                target_speed = min(self.waypoint_approach_speed, distance_to_target * 0.4)
            else:
                target_speed = self.max_speed
            
            desired_velocity = direction_to_target * target_speed
        else:
            desired_velocity = np.array([0.0, 0.0, 0.0])
        
        # Calculate obstacle avoidance with enhanced square mission logic
        avoidance_velocity = np.array([0.0, 0.0, 0.0])
        active_obstacles = 0
        critical_obstacles = 0
        
        for obstacle in self.obstacles['fused']:
            obstacle_pos = obstacle.get('position', np.array([0, 0, 0]))
            obstacle_distance = obstacle.get('distance', float('inf'))
            
            if obstacle_distance < self.safe_distance:
                active_obstacles += 1
                
                if obstacle_distance < self.critical_distance:
                    critical_obstacles += 1
                
                # Calculate repulsive vector
                relative_pos = self.current_position - obstacle_pos
                
                if np.linalg.norm(relative_pos) > 0.1:
                    avoidance_direction = relative_pos / np.linalg.norm(relative_pos)
                    
                    # Enhanced force calculation for square mission
                    if obstacle_distance < self.critical_distance:
                        force_multiplier = 2.5
                    else:
                        force_multiplier = 1.0
                    
                    distance_ratio = (self.safe_distance - obstacle_distance) / self.safe_distance
                    force_strength = self.avoidance_force * force_multiplier * (distance_ratio ** 1.5)
                    
                    # Add vertical avoidance component
                    if obstacle_pos[2] > self.current_position[2] - 3:
                        avoidance_direction[2] -= 0.8  # Strong upward bias
                    
                    # Apply avoidance force
                    obstacle_avoidance = avoidance_direction * force_strength
                    avoidance_velocity += obstacle_avoidance
                    
                    if self.debug_mode and obstacle_distance < self.critical_distance:
                        obj_class = obstacle.get('object_class', 'unknown')
                        print(f"⚠️  CRITICAL AVOIDANCE: {obj_class} at {obstacle_distance:.1f}m")
        
        # Combine desired velocity and avoidance
        if critical_obstacles > 0:
            # In critical situation, prioritize avoidance
            final_velocity = avoidance_velocity * 0.8 + desired_velocity * 0.2
        elif active_obstacles > 0:
            # Normal avoidance situation
            final_velocity = desired_velocity * 0.7 + avoidance_velocity * 0.3
        else:
            # No obstacles, normal navigation
            final_velocity = desired_velocity
        
        # Limit speed based on situation
        max_allowed_speed = self.max_speed * 0.6 if critical_obstacles > 0 else self.max_speed
        speed = np.linalg.norm(final_velocity)
        if speed > max_allowed_speed:
            final_velocity = final_velocity / speed * max_allowed_speed
        
        return final_velocity
    
    def _stop_mission(self):
        """Stop square mission safely"""
        print("🛑 Stopping square mission")
        
        self.avoidance_active = False
        self.sensors_active = False
        
        # Close debug windows
        if self.show_debug_images:
            cv2.destroyAllWindows()
        
        self.client.hoverAsync().join()
        time.sleep(1)
        
        # Final mission statistics
        total_time = time.time() - self.mission_start_time if self.mission_start_time > 0 else 0
        print("📊 FINAL MISSION STATISTICS:")
        print(f"   🔄 Loops completed: {self.mission_loops_completed}/{self.max_mission_loops}")
        print(f"   📏 Total distance: {self.total_distance_traveled:.1f}m")
        print(f"   ⏱️  Total time: {total_time:.1f}s")
        if total_time > 0:
            print(f"   🚀 Average speed: {self.total_distance_traveled/total_time:.1f}m/s")
        
        print("🛬 Landing...")
        self.client.landAsync().join()
        print("✅ Square mission completed successfully!")
    
    def get_mission_status(self) -> Dict[str, Any]:
        """Get current mission status"""
        current_time = time.time()
        return {
            'mission_active': self.avoidance_active,
            'mission_completed': self.mission_completed,
            'current_position': self.current_position.tolist(),
            'target_position': self.target_position.tolist(),
            'current_waypoint': self.current_waypoint_index + 1,
            'total_waypoints': len(self.waypoints),
            'loops_completed': self.mission_loops_completed,
            'max_loops': self.max_mission_loops,
            'distance_to_target': float(np.linalg.norm(self.current_position - self.target_position)),
            'total_distance_traveled': self.total_distance_traveled,
            'mission_time': current_time - self.mission_start_time if self.mission_start_time > 0 else 0,
            'waypoint_time': current_time - self.waypoint_start_time if self.waypoint_start_time > 0 else 0,
            'obstacles': {
                'camera_count': len(self.obstacles['camera']),
                'lidar_count': len(self.obstacles['lidar']),
                'fused_count': len(self.obstacles['fused']),
                'critical_count': len([obs for obs in self.obstacles['fused'] 
                                     if obs.get('distance', float('inf')) < self.critical_distance])
            },
            'square_size': self.square_size,
            'mission_altitude': self.mission_altitude
        }


def run_30m_square_mission():
    """Run the 30m square obstacle avoidance mission"""
    
    print("🟦" + "="*70)
    print("🚁 AirSim 30M SQUARE MISSION - Obstacle Avoidance Demo")
    print("🟦" + "="*70)
    print("🎯 MISSION FEATURES:")
    print("   • 30m x 30m square flight pattern")
    print("   • Multi-sensor obstacle avoidance (Camera + LiDAR)")
    print("   • 4 waypoints + return to start")
    print("   • Complete 3 full square loops")
    print("   • Real-time obstacle detection and avoidance")
    print("   • Enhanced debug visualization")
    print()
    print("🎮 CONTROLS (in debug window):")
    print("   • 'q' = Quit mission")
    print("   • 'n' = Skip to next waypoint")
    print("   • 'r' = Reset mission")
    print()
    
    # Allow customization
    try:
        user_input = input("Enter square size in meters (default 30): ").strip()
        square_size = float(user_input) if user_input else 30.0
    except:
        square_size = 30.0
    
    try:
        user_input = input("Enter number of loops (default 3): ").strip()
        max_loops = int(user_input) if user_input else 3
    except:
        max_loops = 3
    
    print(f"🟦 Starting mission: {square_size}m square, {max_loops} loops")
    print()
    
    # Create and start mission
    mission_system = SquareMissionObstacleAvoidance(
        use_yolo=True,
        debug_mode=True,
        square_size=square_size
    )
    mission_system.max_mission_loops = max_loops
    
    try:
        mission_system.start_square_mission()
        
    except KeyboardInterrupt:
        print("\n⚠️  Mission interrupted by user")
    except Exception as e:
        print(f"❌ Mission error: {e}")
    finally:
        print("🟦 30m Square Mission Demo completed")


if __name__ == "__main__":
    print("🟦 AirSim 30M SQUARE MISSION - Multi-Sensor Obstacle Avoidance")
    print("Enhanced with square flight pattern, waypoint navigation, and mission tracking")
    print("Dependencies: pip install ultralytics airsim opencv-python numpy")
    print()
    
    run_30m_square_mission()