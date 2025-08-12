import airsim
import time
import numpy as np

class AdvancedMissionPlanner:
    def __init__(self, client):
        self.client = client
        self.waypoints = []
        
    def add_waypoint(self, x, y, z, speed=3.0, hover_time=0.0):
        self.waypoints.append({
            'position': (x, y, z),
            'speed': speed,
            'hover_time': hover_time
        })
        
    def create_survey_pattern(self, length, width, altitude, spacing=5.0):
        """Create lawn-mower survey pattern"""
        num_lines = int(width / spacing) + 1
        
        for i in range(num_lines):
            y = i * spacing
            
            if i % 2 == 0:  # Left to right
                self.add_waypoint(0, y, -altitude)
                self.add_waypoint(length, y, -altitude)
            else:  # Right to left
                self.add_waypoint(length, y, -altitude)
                self.add_waypoint(0, y, -altitude)
                
    def execute_mission(self):
        """Execute all waypoints"""
        for i, wp in enumerate(self.waypoints):
            x, y, z = wp['position']
            speed = wp['speed']
            hover_time = wp['hover_time']
            
            print(f"Waypoint {i+1}/{len(self.waypoints)}: ({x:.1f}, {y:.1f}, {z:.1f})")
            
            self.client.moveToPositionAsync(x, y, z, speed).join()
            
            if hover_time > 0:
                time.sleep(hover_time)
                
        print("Mission complete")

# Example usage
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

planner = AdvancedMissionPlanner(client)
planner.create_survey_pattern(50, 30, 10, spacing=8.0)

client.takeoffAsync().join()
planner.execute_mission()
client.landAsync().join()
