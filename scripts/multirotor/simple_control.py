#!/usr/bin/env python3
"""
Fixed Simple AirSim Drone Controller with Mouse Control
Based on working simple_control.py with mouse error fixes
"""

import airsim
import time
import threading
import sys
from pynput import keyboard, mouse
from pynput.keyboard import Key
import math

class SimpleDroneController:
    def __init__(self):
        # AirSim connection
        self.client = airsim.MultirotorClient()
        
        # Control parameters
        self.step_size = 2.0
        self.yaw_step = 10.0
        self.mouse_sensitivity = 0.15  # Increased for smoother 360 control
        self.max_speed = 3.0
        
        # State variables
        self.keyboard_control_active = False
        self.mouse_control_active = False
        self.last_mouse_pos = None
        self.current_keys = set()
        
        # Rate limiting for API calls
        self._last_mouse_command_time = 0
        self._last_keyboard_command_time = 0
        self._last_state_query_time = 0
        self._cached_state = None
        
        # Control flags
        self.running = True
        self.armed = False
        
        # Listeners
        self.keyboard_listener = None
        self.mouse_listener = None
        
        print("Simple AirSim Drone Controller Initialized")
        self.print_help()
    
    def print_help(self):
        """Print control instructions"""
        print("\n" + "="*60)
        print("SIMPLE AIRSIM DRONE CONTROLLER")
        print("="*60)
        print("TERMINAL COMMANDS:")
        print("  connect      - Connect to AirSim")
        print("  arm          - Arm the drone")
        print("  disarm       - Disarm the drone")
        print("  takeoff      - Takeoff (auto arms)")
        print("  land         - Land the drone")
        print("  hover        - Hover in place")
        print("  kc           - Toggle keyboard control")
        print("  mc           - Toggle mouse control")
        print("  status       - Show drone status")
        print("  help         - Show this help")
        print("  quit/exit    - Exit program")
        print()
        print("KEYBOARD CONTROLS (when 'kc' mode active):")
        print("  W/S          - Forward/Backward")
        print("  A/D          - Left/Right")
        print("  Z/X          - Up/Down")
        print("  Q/E          - Yaw Left/Right")
        print("  ESC          - Stop keyboard control")
        print()
        print("MOUSE CONTROLS (when 'mc' mode active):")
        print("  Mouse Move   - 360° Pitch/Yaw control (like flight simulator)")
        print("  Mouse Wheel  - Altitude control")
        print("  Left Click   - Quick forward")
        print("  Right Click  - Quick backward")
        print("  Middle Click - Hover")
        print("  ESC          - Stop mouse control")
        print("="*60)
    
    def connect_drone(self):
        """Initialize drone connection"""
        try:
            self.client.confirmConnection()
            self.client.enableApiControl(True)
            print("✓ Connected to AirSim and API control enabled")
            return True
        except Exception as e:
            print(f"✗ Failed to connect: {e}")
            return False
    
    def arm_drone(self):
        """Arm the drone"""
        try:
            self.client.armDisarm(True)
            self.armed = True
            print("✓ Drone armed")
        except Exception as e:
            print(f"✗ Failed to arm: {e}")
    
    def disarm_drone(self):
        """Disarm the drone"""
        try:
            self.client.armDisarm(False)
            self.armed = False
            print("✓ Drone disarmed")
        except Exception as e:
            print(f"✗ Failed to disarm: {e}")
    
    def takeoff_drone(self):
        """Takeoff sequence"""
        try:
            if not self.armed:
                self.arm_drone()
            self.client.takeoffAsync().join()
            print("✓ Takeoff complete")
        except Exception as e:
            print(f"✗ Takeoff failed: {e}")
    
    def land_drone(self):
        """Land the drone"""
        try:
            self.client.landAsync().join()
            print("✓ Landing complete")
        except Exception as e:
            print(f"✗ Landing failed: {e}")
    
    def hover_drone(self):
        """Hover the drone"""
        try:
            self.client.hoverAsync()
            print("✓ Hovering")
        except Exception as e:
            print(f"✗ Hover failed: {e}")
    
    def get_drone_state(self):
        """Get drone state with rate limiting"""
        current_time = time.time()
        
        # Rate limit state queries to every 200ms
        if current_time - self._last_state_query_time < 0.2:
            return self._cached_state
        
        try:
            state = self.client.getMultirotorState()
            pose = self.client.simGetVehiclePose()
            
            result = {
                'position': pose.position,
                'orientation': pose.orientation,
                'velocity': state.kinematics_estimated.linear_velocity,
                'armed': self.armed
            }
            
            self._cached_state = result
            self._last_state_query_time = current_time
            return result
            
        except Exception as e:
            print(f"✗ Failed to get state: {e}")
            return None
    
    def get_drone_status(self):
        """Get and display drone status"""
        try:
            state = self.get_drone_state()
            if not state:
                print("✗ Could not get drone status")
                return
            
            pos = state['position']
            vel = state['velocity']
            
            # Convert quaternion to Euler angles
            orientation = state['orientation']
            euler = airsim.to_eularian_angles(orientation)
            
            print("\n" + "-"*40)
            print("DRONE STATUS")
            print("-"*40)
            print(f"Armed: {state['armed']}")
            print(f"Position: X={pos.x_val:.2f}, Y={pos.y_val:.2f}, Z={pos.z_val:.2f}")
            print(f"Velocity: X={vel.x_val:.2f}, Y={vel.y_val:.2f}, Z={vel.z_val:.2f}")
            print(f"Orientation: Pitch={math.degrees(euler[0]):.1f}°, Roll={math.degrees(euler[1]):.1f}°, Yaw={math.degrees(euler[2]):.1f}°")
            print(f"Keyboard Control: {'ON' if self.keyboard_control_active else 'OFF'}")
            print(f"Mouse Control: {'ON' if self.mouse_control_active else 'OFF'}")
            print("-"*40)
        except Exception as e:
            print(f"✗ Failed to get status: {e}")
    
    # Keyboard Control Methods
    def on_key_press(self, key):
        """Handle key press events"""
        if not self.keyboard_control_active:
            return
        
        try:
            if hasattr(key, 'char') and key.char:
                self.current_keys.add(key.char.lower())
            else:
                self.current_keys.add(key)
        except AttributeError:
            self.current_keys.add(key)
    
    def on_key_release(self, key):
        """Handle key release events"""
        if not self.keyboard_control_active:
            if key == Key.esc:
                self.stop_keyboard_control()
            return
        
        try:
            if hasattr(key, 'char') and key.char:
                self.current_keys.discard(key.char.lower())
            else:
                self.current_keys.discard(key)
            
            if key == Key.esc:
                self.stop_keyboard_control()
        except AttributeError:
            self.current_keys.discard(key)
    
    def process_keyboard_movement(self):
        """Process keyboard input and move drone with rate limiting"""
        if not self.keyboard_control_active or not self.current_keys:
            return
        
        current_time = time.time()
        # Rate limit keyboard commands to 10Hz
        if current_time - self._last_keyboard_command_time < 0.1:
            return
        
        try:
            # Get current state
            state = self.get_drone_state()
            if not state:
                return
            
            current_pos = state['position']
            
            # Calculate movement
            x_offset = 0
            y_offset = 0
            z_offset = 0
            yaw_offset = 0
            
            if 'w' in self.current_keys:
                x_offset += self.step_size
            if 's' in self.current_keys:
                x_offset -= self.step_size
            if 'd' in self.current_keys:
                y_offset += self.step_size
            if 'a' in self.current_keys:
                y_offset -= self.step_size
            if 'z' in self.current_keys:
                z_offset -= self.step_size  # Up (negative Z)
            if 'x' in self.current_keys:
                z_offset += self.step_size  # Down
            if 'q' in self.current_keys:
                yaw_offset -= self.yaw_step
            if 'e' in self.current_keys:
                yaw_offset += self.yaw_step
            
            # Apply movement if significant
            if abs(x_offset) > 0.1 or abs(y_offset) > 0.1 or abs(z_offset) > 0.1:
                new_pos = airsim.Vector3r(
                    current_pos.x_val + x_offset,
                    current_pos.y_val + y_offset,
                    current_pos.z_val + z_offset
                )
                self.client.moveToPositionAsync(
                    new_pos.x_val, new_pos.y_val, new_pos.z_val, 
                    velocity=self.max_speed
                )
                self._last_keyboard_command_time = current_time
            
            # Apply yaw if significant
            if abs(yaw_offset) > 0.5:
                current_yaw = airsim.to_eularian_angles(state['orientation'])[2]
                new_yaw = current_yaw + math.radians(yaw_offset)
                self.client.rotateToYawAsync(math.degrees(new_yaw))
                self._last_keyboard_command_time = current_time
                
        except Exception as e:
            print(f"Keyboard movement error: {e}")
    
    # Mouse Control Methods
    def on_mouse_move(self, x, y):
        """Handle mouse movement for full 360-degree pitch/yaw control"""
        if not self.mouse_control_active:
            return
        
        if self.last_mouse_pos is None:
            self.last_mouse_pos = (x, y)
            return
        
        current_time = time.time()
        # Rate limit mouse commands to 8Hz for smoother 360 control
        if current_time - self._last_mouse_command_time < 0.125:
            return
        
        try:
            # Calculate relative movement
            dx = x - self.last_mouse_pos[0]
            dy = y - self.last_mouse_pos[1]
            
            # Only process significant movement (reduced threshold for smoother control)
            if abs(dx) > 3 or abs(dy) > 3:
                # Get current state
                state = self.get_drone_state()
                if not state:
                    return
                
                # Get current orientation
                current_euler = airsim.to_eularian_angles(state['orientation'])
                current_pitch = current_euler[0]  # Pitch (up/down)
                current_roll = current_euler[1]   # Roll (tilt)
                current_yaw = current_euler[2]    # Yaw (left/right)
                
                # Calculate movement changes
                yaw_change = dx * self.mouse_sensitivity * 0.2    # Horizontal mouse = yaw
                pitch_change = -dy * self.mouse_sensitivity * 0.2  # Vertical mouse = pitch (inverted)
                
                # Calculate new angles
                new_yaw = current_yaw + math.radians(yaw_change)
                new_pitch = current_pitch + math.radians(pitch_change)
                
                # Limit pitch to safe range (prevent flipping)
                max_pitch = math.radians(45)  # 45 degrees max pitch
                new_pitch = max(-max_pitch, min(max_pitch, new_pitch))
                
                # Apply 360-degree rotation using moveByAngleRatesThrottleAsync for smooth control
                # This gives us continuous control over pitch, roll, yaw, and throttle
                pitch_rate = pitch_change * 2.0   # Pitch rate (degrees/second)
                roll_rate = 0.0                   # Keep roll stable
                yaw_rate = yaw_change * 2.0       # Yaw rate (degrees/second)
                throttle = 0.5                    # Maintain altitude
                duration = 0.1                    # Short duration for responsiveness
                
                # Send the movement command
                self.client.moveByAngleRatesThrottleAsync(
                    pitch_rate, roll_rate, yaw_rate, throttle, duration
                )
                
                self._last_mouse_command_time = current_time
                self.last_mouse_pos = (x, y)
            
        except Exception as e:
            print(f"Mouse movement error: {e}")
    
    def on_mouse_click(self, x, y, button, pressed):
        """Handle mouse click events with rate limiting"""
        if not self.mouse_control_active or not pressed:
            return
        
        current_time = time.time()
        # Rate limit click commands
        if current_time - self._last_mouse_command_time < 0.5:
            return
        
        try:
            state = self.get_drone_state()
            if not state:
                return
            
            current_pos = state['position']
            
            if button == mouse.Button.left:
                # Quick forward movement
                new_pos = airsim.Vector3r(
                    current_pos.x_val + self.step_size,
                    current_pos.y_val,
                    current_pos.z_val
                )
                self.client.moveToPositionAsync(
                    new_pos.x_val, new_pos.y_val, new_pos.z_val, 
                    velocity=self.max_speed
                )
                print("→ Quick forward")
                
            elif button == mouse.Button.right:
                # Quick backward movement
                new_pos = airsim.Vector3r(
                    current_pos.x_val - self.step_size,
                    current_pos.y_val,
                    current_pos.z_val
                )
                self.client.moveToPositionAsync(
                    new_pos.x_val, new_pos.y_val, new_pos.z_val, 
                    velocity=self.max_speed
                )
                print("← Quick backward")
                
            elif button == mouse.Button.middle:
                # Hover
                self.client.hoverAsync()
                print("⏸ Hover")
            
            self._last_mouse_command_time = current_time
                
        except Exception as e:
            print(f"Mouse click error: {e}")
    
    def on_mouse_scroll(self, x, y, dx, dy):
        """Handle mouse scroll for altitude control with rate limiting"""
        if not self.mouse_control_active:
            return
        
        current_time = time.time()
        # Rate limit scroll commands
        if current_time - self._last_mouse_command_time < 0.3:
            return
        
        try:
            state = self.get_drone_state()
            if not state:
                return
            
            current_pos = state['position']
            
            # Scroll up = go up (negative Z), scroll down = go down (positive Z)
            z_offset = -dy * self.step_size * 0.5  # Half step for finer control
            
            new_pos = airsim.Vector3r(
                current_pos.x_val,
                current_pos.y_val,
                current_pos.z_val + z_offset
            )
            
            self.client.moveToPositionAsync(
                new_pos.x_val, new_pos.y_val, new_pos.z_val, 
                velocity=self.max_speed
            )
            
            direction = "↑" if dy > 0 else "↓"
            print(f"{direction} Altitude change")
            self._last_mouse_command_time = current_time
            
        except Exception as e:
            print(f"Mouse scroll error: {e}")
    
    # Control Mode Management
    def start_keyboard_control(self):
        """Start keyboard control mode"""
        if self.keyboard_control_active:
            print("Keyboard control already active")
            return
        
        self.keyboard_control_active = True
        
        # Start keyboard listener
        self.keyboard_listener = keyboard.Listener(
            on_press=self.on_key_press,
            on_release=self.on_key_release
        )
        self.keyboard_listener.start()
        
        print("🎮 Keyboard control ACTIVATED")
        print("Use WASD for movement, ZX for altitude, QE for yaw, ESC to stop")
        
        # Start movement processing thread
        self.keyboard_thread = threading.Thread(target=self._keyboard_control_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
    
    def stop_keyboard_control(self):
        """Stop keyboard control mode"""
        if not self.keyboard_control_active:
            return
        
        self.keyboard_control_active = False
        self.current_keys.clear()
        
        if self.keyboard_listener:
            self.keyboard_listener.stop()
        
        print("🎮 Keyboard control DEACTIVATED")
    
    def start_mouse_control(self):
        """Start mouse control mode"""
        if self.mouse_control_active:
            print("Mouse control already active")
            return
        
        self.mouse_control_active = True
        self.last_mouse_pos = None
        
        # Start mouse listener
        self.mouse_listener = mouse.Listener(
            on_move=self.on_mouse_move,
            on_click=self.on_mouse_click,
            on_scroll=self.on_mouse_scroll
        )
        self.mouse_listener.start()
        
        print("🖱️  Mouse control ACTIVATED")
        print("360° pitch/yaw control: Move mouse to look around like a flight simulator")
        print("Scroll for altitude, Left=forward, Right=backward, Middle=hover")
    
    def stop_mouse_control(self):
        """Stop mouse control mode"""
        if not self.mouse_control_active:
            return
        
        self.mouse_control_active = False
        
        if self.mouse_listener:
            self.mouse_listener.stop()
        
        print("🖱️  Mouse control DEACTIVATED")
    
    def _keyboard_control_loop(self):
        """Background loop for processing keyboard movement"""
        while self.keyboard_control_active and self.running:
            self.process_keyboard_movement()
            time.sleep(0.05)  # 20Hz update rate for keyboard
    
    # Main Control Loop
    def run(self):
        """Main program loop"""
        print("\\nType 'help' for commands.")
        
        try:
            while self.running:
                try:
                    command = input("drone> ").strip().lower()
                    
                    if command in ['quit', 'exit']:
                        break
                    elif command == 'help':
                        self.print_help()
                    elif command == 'connect':
                        self.connect_drone()
                    elif command == 'arm':
                        self.arm_drone()
                    elif command == 'disarm':
                        self.disarm_drone()
                    elif command == 'takeoff':
                        self.takeoff_drone()
                    elif command == 'land':
                        self.land_drone()
                    elif command == 'hover':
                        self.hover_drone()
                    elif command == 'kc':
                        if self.keyboard_control_active:
                            self.stop_keyboard_control()
                        else:
                            self.start_keyboard_control()
                    elif command == 'mc':
                        if self.mouse_control_active:
                            self.stop_mouse_control()
                        else:
                            self.start_mouse_control()
                    elif command == 'status':
                        self.get_drone_status()
                    elif command == '':
                        continue
                    else:
                        print(f"Unknown command: {command}. Type 'help' for available commands.")
                        
                except KeyboardInterrupt:
                    print("\\n^C detected")
                    break
                except Exception as e:
                    print(f"Command error: {e}")
                    
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        print("\\nShutting down...")
        
        self.running = False
        self.stop_keyboard_control()
        self.stop_mouse_control()
        
        try:
            self.client.hoverAsync().join()
            self.disarm_drone()
            self.client.enableApiControl(False)
        except:
            pass
        
        print("Goodbye!")

def main():
    """Main entry point"""
    print("Simple AirSim Drone Controller Starting...")
    
    controller = SimpleDroneController()
    try:
        controller.run()
    except KeyboardInterrupt:
        print("\\nInterrupted by user")
    except Exception as e:
        print(f"Fatal error: {e}")
    finally:
        controller.cleanup()

if __name__ == "__main__":
    main()