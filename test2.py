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

async def takeoff(drone, altitude):
    pass

async def land(drone):
    pass

async def move_to(drone, north, east, down, yaw):
    pass

async def main():
    zed, runtime = await init_camera()
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # Wait for connection
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    # Wait for position lock
    async for _ in drone.telemetry.position():
        break

    # Start listening to position and attitude
    asyncio.create_task(listen_position(drone))
    asyncio.create_task(listen_attitude(drone))

    print("Arming...")
    await drone.action.arm()

    print("Starting offboard...")
    try:
        # Must send a first setpoint BEFORE starting offboard
        await drone.offboard.set_position_ned(PositionNedYaw(0, 0, -5, 0))
        await asyncio.sleep(0.1)
        await drone.offboard.start()
        asyncio.create_task(print_position(drone))
    except OffboardError as e:
        print("Could not start offboard:", e)
        await drone.action.disarm()
        return
    await asyncio.sleep(5)

    print("Move forward 5 m")
    await drone.offboard.set_position_ned(PositionNedYaw(2, 0, -5, 0))
    await asyncio.sleep(3)

    print("Move right 2 m")
    await drone.offboard.set_position_ned(PositionNedYaw(2, 2, -5, 0))
    await asyncio.sleep(3)

    print("Move backward 3 m")
    await drone.offboard.set_position_ned(PositionNedYaw(0, 2, -5, 0))
    await asyncio.sleep(3)

    print("Hold position...")
    await drone.offboard.set_position_ned(PositionNedYaw(0, 0, -5, 0))
    await asyncio.sleep(3)

    print("Stop offboard and land")
    # Cancel all running tasks
    for task in asyncio.all_tasks():
        if task is not asyncio.current_task():
            task.cancel()
    await asyncio.sleep(0.1)  # Give tasks time to cancel

    await drone.offboard.stop()
    await drone.action.land()

asyncio.run(main())
