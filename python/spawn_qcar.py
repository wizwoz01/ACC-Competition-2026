"""
QCar 2 Spawn Script for ACC 2026 Self-Driving Car Competition
Beach Autonomous Systems - CSULB

COMPETITION COMPLIANT:
This script ONLY spawns the QCar and sets up the environment.
It does NOT control the QCar or gather sensor data - those are done via QUARC/Simulink.

The competition rule states:
"Controlling the QCar or gathering data via the qvl library functions will invalidate any submission."

Spawning is environment setup, NOT control or data gathering.

Usage:
    python spawn_qcar.py
    
After running this script, run QUARC/Simulink model to control the vehicle.
"""

import sys
import time

try:
    from qvl.qlabs import QuanserInteractiveLabs
    from qvl.qcar2 import QLabsQCar2
    from qvl.system import QLabsSystem
except ImportError:
    print("ERROR: qvl library not installed.")
    print("Install via: pip install qvl")
    sys.exit(1)


def spawn_qcar_for_quarc():
    """
    Spawn a QCar 2 in QLabs for QUARC/Simulink control.
    
    This function:
    1. Connects to QLabs
    2. Spawns a QCar 2 at a specified location
    3. Prints the port numbers for QUARC connection
    
    After this runs, your Simulink model can connect via:
    - HIL Initialize: 0@tcpip://localhost:18960
    - Cameras, Lidar, GPS at their respective ports
    """
    
    print("=" * 60)
    print("  QCar 2 Spawn Script - ACC 2026 Competition")
    print("  Beach Autonomous Systems - CSULB")
    print("=" * 60)
    print()
    
    # Connect to QLabs
    print("[1/4] Connecting to QLabs...")
    qlabs = QuanserInteractiveLabs()
    
    try:
        qlabs.open("localhost")
        print("      Connected to QLabs successfully!")
    except Exception as e:
        print(f"      ERROR: Could not connect to QLabs: {e}")
        print("      Make sure QLabs is running and a workspace is loaded.")
        return False
    
    # Clear existing actors 
    print("[2/4] Preparing environment...")
    try:
        system = QLabsSystem(qlabs)
        # Uncomment to destroy all existing actors:
        # system.destroy_all_spawned_actors()
        print("      Environment ready.")
    except Exception as e:
        print(f"      Warning: Could not access system: {e}")
    
    # Spawn QCar 2
    print("[3/4] Spawning QCar 2...")
    
    # Cityscape spawn coordinates 
    # "Car Spawn Spot": x=0, y=-1.300, z=0.005, yaw=90 degrees (pi/2)
    spawn_x = 0.0
    spawn_y = -1.3
    spawn_z = 0.005
    spawn_yaw = 1.5708  # 90 degrees in radians (pi/2)
    
    try:
        qcar = QLabsQCar2(qlabs)
        
        # Spawn the QCar 2
        # Parameters: actorNumber, location [x,y,z], rotation [roll,pitch,yaw], scale [x,y,z], configuration, waitForConfirmation
        qcar.spawn_id(
            actorNumber=0,  # Actor ID 0 corresponds to the default ports
            location=[spawn_x, spawn_y, spawn_z],
            rotation=[0, 0, spawn_yaw],
            scale=[1, 1, 1],  # Must be an array, not a single float!
            configuration=0,
            waitForConfirmation=True
        )
        
        print(f"      QCar 2 spawned at position ({spawn_x}, {spawn_y}, {spawn_z})")
        print(f"      Heading: {spawn_yaw:.2f} rad ({spawn_yaw * 180 / 3.14159:.0f} degrees)")
        
    except Exception as e:
        print(f"      ERROR: Could not spawn QCar 2: {e}")
        qlabs.close()
        return False
    
    # Print QUARC connection info
    print("[4/4] QCar 2 ready for QUARC/Simulink control!")
    print()
    print("=" * 60)
    print("  QUARC/Simulink Connection Information")
    print("=" * 60)
    print()
    print("  HIL Initialize Block:")
    print("    Board type:       qcar2")
    print("    Board identifier: 0@tcpip://localhost:18960")
    print()
    print("  Camera Ports (Video Capture blocks):")
    print("    Front Camera:  0@tcpip://localhost:18942")
    print("    Right Camera:  0@tcpip://localhost:18940")
    print("    Back Camera:   0@tcpip://localhost:18941")
    print("    Left Camera:   0@tcpip://localhost:18943")
    print()
    print("  RGBD Camera (Video3D Capture block):")
    print("    Device:        0@tcpip://localhost:18965")
    print()
    print("  Lidar (Lidar block):")
    print("    URI:           tcpip://localhost:18966")
    print()
    print("  GPS (GPS block):")
    print("    URI:           tcpip://localhost:18967")
    print()
    print("=" * 60)
    print("  QCar 2 is now ready!")
    print("  Run Simulink model to start autonomous control.")
    print("=" * 60)
    print()
    
    # Keep connection open 
    print("Press Ctrl+C to close connection and destroy QCar...")
    print("(Keep this script running while using Simulink)")
    print()
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nClosing connection...")
        qlabs.close()
        print("Done.")
    
    return True


if __name__ == "__main__":
    spawn_qcar_for_quarc()

