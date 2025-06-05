# keyboard_control.py
# A Python script to control a drone in AirSim using keyboard inputs.
# License: MIT License
#
# Copyright (c) 2025
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Description:
# This script allows users to control a drone in the AirSim simulator using
# keyboard inputs. It supports basic commands like connecting to the simulator,
# launching the drone, controlling movement, landing, and quitting the program.
#
# Requirements:
# - Python 3.7–3.10 (recommended, as AirSim may have issues with newer versions)
# - AirSim Python package (`pip install airsim`)
# - Keyboard Python package (`pip install keyboard`)
# - AirSim simulator running with a multirotor environment
#
# Usage:
# 1. Ensure AirSim is running with the correct settings (see settings.json example).
# 2. Run the script: `python keyboard_control.py`
# 3. Enter commands: connect, takeoff, fly, land, quit, or auto
# 4. During flight (after 'fly'), use the keyboard controls displayed at startup.
#
# Example settings.json (place in C:\Users\<YourUser>\Documents\AirSim\settings.json):
# {
#   "SettingsVersion": 2.0,
#   "SimMode": "Multirotor",
#   "Vehicles": {
#     "SimpleFlight": {
#       "VehicleType": "SimpleFlight",
#       "RC": {
#         "RemoteControlID": -1
#       }
#     }
#   }
# }

import airsim
import keyboard
import time
import sys

class DroneController:
    def __init__(self, client):
        """Initialize the drone controller with an AirSim client."""
        self.client = client
        self.is_connected = False
        self.is_flying = False
        self.is_controlling = False
        
        # Movement velocities
        self.velocity_x = 0.0  # Forward/backward
        self.velocity_y = 0.0  # Left/right
        self.velocity_z = 0.0  # Up/down
        self.yaw_rate = 0.0    # Rotation speed
        
        # Movement settings
        self.movement_speed = 10.0  # Speed in m/s
        self.rotation_speed = 30.0  # Yaw rate in degrees/s
        
        # Enable velocity logging if --verbose is passed
        self.log_velocity = "--verbose" in sys.argv

    def initialize_drone_connection(self):
        """Establish connection with the drone in AirSim."""
        if self.is_connected:
            print("Already connected to the drone.")
            return True
        try:
            print("Connecting to the drone in AirSim...")
            self.client.enableApiControl(True, "SimpleFlight")
            print("API control enabled:", self.client.isApiControlEnabled("SimpleFlight"))
            self.client.armDisarm(True, "SimpleFlight")
            self.is_connected = True
            print("Connection established successfully.")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            print("Ensure AirSim is running and settings are correct.")
            return False
    
    def launch_drone(self):
        """Launch the drone into the air."""
        if not self.is_connected:
            print("Please connect to the drone first.")
            return False
            
        try:
            print("Launching the drone...")
            self.client.takeoffAsync(vehicle_name="SimpleFlight").join()
            time.sleep(2)  # Allow time for the drone to stabilize
            self.is_flying = True
            print("Drone has launched successfully.")
            # Log initial position for debugging
            state = self.client.getMultirotorState(vehicle_name="SimpleFlight")
            pos = state.kinematics_estimated.position
            print(f"Initial position: x={pos.x_val:.2f}, y={pos.y_val:.2f}, z={pos.z_val:.2f}")
            return True
        except Exception as e:
            print(f"Launch failed: {e}")
            return False
    
    def land_drone(self):
        """Land the drone safely."""
        if not self.is_flying:
            print("Drone is not flying, cannot land.")
            return False
        try:
            print("Landing the drone...")
            self.is_controlling = False
            self.client.landAsync(vehicle_name="SimpleFlight").join()
            self.is_flying = False
            print("Drone has landed successfully.")
            return True
        except Exception as e:
            print(f"Landing failed: {e}")
            return False
    
    def handle_drone_movement(self):
        """Process keyboard inputs to control drone movement."""
        print("Starting drone movement control.")
        try:
            while self.is_controlling and self.is_flying:
                # Reset velocities
                self.velocity_x = self.velocity_y = self.velocity_z = self.yaw_rate = 0.0
                
                # Update velocities based on keyboard input
                if keyboard.is_pressed('w'):
                    self.velocity_x = self.movement_speed
                    print("Moving forward")
                if keyboard.is_pressed('s'):
                    self.velocity_x = -self.movement_speed
                    print("Moving backward")
                if keyboard.is_pressed('a'):
                    self.velocity_y = -self.movement_speed
                    print("Moving left")
                if keyboard.is_pressed('d'):
                    self.velocity_y = self.movement_speed
                    print("Moving right")
                if keyboard.is_pressed('r'):
                    self.velocity_z = -self.movement_speed
                    print("Moving up")
                if keyboard.is_pressed('f'):
                    self.velocity_z = self.movement_speed
                    print(f"Moving down (velocity_z={self.velocity_z})")
                if keyboard.is_pressed('q'):
                    self.yaw_rate = -self.rotation_speed
                    print("Rotating left")
                if keyboard.is_pressed('e'):
                    self.yaw_rate = self.rotation_speed
                    print("Rotating right")
                if keyboard.is_pressed('space'):
                    self.velocity_x = self.velocity_y = self.velocity_z = self.yaw_rate = 0
                    print("Stopping movement")
                
                # Apply movement command to the drone
                try:
                    self.client.moveByVelocityAsync(
                        self.velocity_x, self.velocity_y, self.velocity_z, 0.1,
                        yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=self.yaw_rate),
                        vehicle_name="SimpleFlight"
                    ).join()
                    # Log position and velocity for debugging
                    state = self.client.getMultirotorState(vehicle_name="SimpleFlight")
                    pos = state.kinematics_estimated.position
                    vel = state.kinematics_estimated.linear_velocity
                    print(f"Position: x={pos.x_val:.2f}, y={pos.y_val:.2f}, z={pos.z_val:.2f}")
                    if self.log_velocity or self.velocity_z != 0:  # Always log if moving up/down
                        print(f"Velocity: x={vel.x_val:.2f}, y={vel.y_val:.2f}, z={vel.z_val:.2f}")
                except Exception as e:
                    print(f"Movement error: {e}")
                
                # Check for ESC key to land
                if keyboard.is_pressed('esc'):
                    print("ESC pressed - landing the drone.")
                    self.land_drone()
                    break
                
                time.sleep(0.1)  # Control at 10Hz
        finally:
            print("Drone movement control stopped.")
    
    def start_keyboard_control(self):
        """Enable keyboard control for flying the drone."""
        if not self.is_flying:
            print("Please launch the drone first.")
            return
        
        print("\n" + "="*50)
        print("KEYBOARD CONTROL ACTIVE")
        print("="*50)
        print("Controls (reminder):")
        print("  W/S - Forward/Backward")
        print("  A/D - Left/Right") 
        print("  R/F - Up/Down")
        print("  Q/E - Yaw Left/Right")
        print("  SPACE - Stop")
        print("  ESC - Land and Exit")
        print("="*50)
        print("Start flying now!")
        
        self.is_controlling = True
        
        try:
            self.handle_drone_movement()
        except KeyboardInterrupt:
            print("\nCTRL+C detected - landing the drone.")
            self.land_drone()
        finally:
            self.is_controlling = False
            print("Keyboard control ended.")

def display_key_mappings():
    """Display the keyboard control mappings for the user."""
    print("\nKeyboard Controls:")
    print("="*50)
    print("  W/S - Forward/Backward")
    print("  A/D - Left/Right") 
    print("  R/F - Up/Down")
    print("  Q/E - Yaw Left/Right")
    print("  SPACE - Stop")
    print("  ESC - Land and Exit")
    print("="*50)

def run_drone_control():
    """Run the main drone control program."""
    print("DRONE CONTROL PROGRAM")
    print("=" * 40)
    display_key_mappings()  # Show key mappings at startup
    
    # Initialize AirSim client
    try:
        print("Initializing AirSim client...")
        client = airsim.MultirotorClient(ip="127.0.0.1", port=41451)
        client.confirmConnection()
        print("AirSim client initialized successfully.")
    except Exception as e:
        print(f"Failed to initialize AirSim client: {e}")
        print("Ensure AirSim is running.")
        return
    
    drone = DroneController(client)
    
    while True:
        cmd = input("\nEnter command (connect/takeoff/fly/land/quit): ").lower().strip()
        
        if cmd == 'connect' or cmd == 'c':
            drone.initialize_drone_connection()
        elif cmd == 'takeoff' or cmd == 't':
            drone.launch_drone()
        elif cmd == 'fly' or cmd == 'f':
            drone.start_keyboard_control()
        elif cmd == 'land' or cmd == 'l':
            drone.land_drone()
        elif cmd == 'quit' or cmd == 'q':
            if drone.is_flying:
                drone.land_drone()
            break
        elif cmd == 'auto':
            if drone.initialize_drone_connection():
                if drone.launch_drone():
                    print("AUTO MODE - Starting flight in 3 seconds...")
                    time.sleep(3)
                    drone.start_keyboard_control()
        else:
            print("Available commands: connect, takeoff, fly, land, quit")
            print("Or use 'auto' for full sequence")

if __name__ == "__main__":
    try:
        import keyboard
        print("Keyboard library found.")
    except ImportError:
        print("Missing keyboard library!")
        print("Install it with: pip install keyboard")
        exit(1)
    
    try:
        import airsim
        print("AirSim library found.")
    except ImportError:
        print("Missing AirSim library!")
        print("Install it with: pip install airsim")
        exit(1)
    
    print("All requirements satisfied.")
    run_drone_control()