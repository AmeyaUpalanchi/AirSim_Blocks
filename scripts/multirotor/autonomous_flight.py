import airsim
import time

# Connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Take off
client.takeoffAsync().join()

# Define positions to move to
positions = [
    airsim.Vector3r(10, 0, -5),
    airsim.Vector3r(10, 10, -5),
    airsim.Vector3r(0, 10, -5),
    airsim.Vector3r(0, 0, -5)
]

# Move to each position
for pos in positions:
    client.moveToPositionAsync(pos.x_val, pos.y_val, pos.z_val, 5).join()
    time.sleep(2)

# Land
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)
