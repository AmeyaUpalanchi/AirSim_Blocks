import airsim
import time
import numpy as np
class DroneController:
    def __init__(self):
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        
    def takeoff_and_hover(self, altitude=4.0):
        """Take off to specified altitude"""
        print("Taking off...")
        self.client.takeoffAsync().join()
        self.client.moveToZAsync(-altitude, 2).join()
        print(f"Reached {altitude}m altitude")
        
    def fly_rectangle_pattern(self, length=10.0, width=5.0, altitude=4.0):
        """Fly rectangular pattern - 10x5 meter rectangle"""
        waypoints = [
            (length, 0, -altitude),
            (length, width, -altitude), 
            (0, width, -altitude),
            (0, 0, -altitude)
        ]
        
        for i, (x, y, z) in enumerate(waypoints):
            print(f"Flying to waypoint {i+1}: ({x}, {y}, {z})")
            self.client.moveToPositionAsync(x, y, z, 3).join()
            time.sleep(2)
            
    def land_and_disarm(self):
        """Land and shut down"""
        print("Landing...")
        self.client.landAsync().join()
        self.client.armDisarm(False)
        self.client.enableApiControl(False)

# Execute mission
if __name__ == "__main__":
    drone = DroneController()
    
    try:
        drone.takeoff_and_hover(4.0)
        time.sleep(5)  # Hover for 5 seconds as mentioned
        drone.fly_rectangle_pattern(10.0, 5.0, 4.0)  # 10x5 meter rectangle
        time.sleep(2)  # Return to start and hover
        drone.land_and_disarm()
        
    except KeyboardInterrupt:
        print("Mission interrupted")
        drone.client.landAsync().join()
        drone.client.armDisarm(False)
