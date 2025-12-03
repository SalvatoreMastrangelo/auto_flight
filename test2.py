import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError
import pyzed.sl as sl
import cv2
import numpy as np
import time

pos_ned = None
att_euler = None

async def init_camera():
    # Create a ZED camera object
    zed = sl.Camera()

    # Set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720   # or sl.RESOLUTION.HD1080
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL   
    init_params.coordinate_units = sl.UNIT.MILLIMETER

    # Open the camera
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print("Error:", status)
        exit(1)

    runtime = sl.RuntimeParameters()

    return zed, runtime

async def connect_drone():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # Wait for connection
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    # Wait for position lock
    async for _ in drone.telemetry.position():
        break

    return drone

async def listen_position(d):
    global pos_ned
    async for pos in d.telemetry.position_velocity_ned():
        pos_ned = pos

async def listen_attitude(d):
    global att_euler
    async for att in d.telemetry.attitude_euler():
        att_euler = att

async def print_position(d):
    global pos_ned, att_euler
    while True:
        if pos_ned and att_euler:
            print(f"Position - N: {pos_ned.position.north_m:.2f}m, E: {pos_ned.position.east_m:.2f}m, D: {pos_ned.position.down_m:.2f}m")
            print(f"Attitude - Roll: {att_euler.roll_deg:.2f}°, Pitch: {att_euler.pitch_deg:.2f}°, Yaw: {att_euler.yaw_deg:.2f}°")
        await asyncio.sleep(0.5)

async def arm(drone):
    await drone.action.arm()

async def takeoff(drone, altitude):
    try:
        # Must send a first setpoint BEFORE starting offboard
        await drone.offboard.set_position_ned(PositionNedYaw(0, 0, -altitude, 0))
        await asyncio.sleep(0.1)
        await drone.offboard.start()
    except OffboardError as e:
        print("Could not start offboard:", e)
        await drone.action.disarm()
        return
    await asyncio.sleep(5)

async def land(drone):
    await drone.offboard.stop()
    await drone.action.land()

async def move_to(drone, north, east, down, yaw):
    await drone.offboard.set_position_ned(PositionNedYaw(north, east, down, yaw))
    await asyncio.sleep(3)

async def main():
    zed, runtime = await init_camera()
    drone = await connect_drone()

    # Start listening to position and attitude
    asyncio.create_task(listen_position(drone))
    asyncio.create_task(listen_attitude(drone))

    print("Arming...")
    await arm(drone)

    asyncio.create_task(print_position(drone))


    print("Takeoff to 5 m")
    await takeoff(drone, 5)

    print("Move forward 2 m")
    await move_to(drone, 2, 0, -5, 0)

    print("Move right 2 m")
    await move_to(drone, 2, 2, -5, 0)

    print("Move backward 2 m")
    await move_to(drone, 0, 2, -5, 0)

    print("Move left 2 m")
    await move_to(drone, 0, 0, -5, 0)

    print("Stop offboard and land")
    # Cancel all running tasks
    for task in asyncio.all_tasks():
        if task is not asyncio.current_task():
            task.cancel()
    await asyncio.sleep(0.1)  # Give tasks time to cancel

    await land(drone)

asyncio.run(main())
