import asyncio
from mavsdk import System,action
from mavsdk.mission import (MissionItem, MissionPlan)
import numpy as np
R = 6371
def haversine_dist(point1, point2):
    point1_cart = np.array(pol2cart(*point1))
    point2_cart = np.array(pol2cart(*point2))
    euc_dist = np.linalg.norm(point1_cart - point2_cart)
    sin_theta_2 = euc_dist / (R * 2)
    theta_2 = np.arcsin(sin_theta_2)
    theta = 2 * theta_2
    dist = R * theta
    return dist, theta
def pol2cart(lat, long):
    lat, long = np.radians(lat), np.radians(long)
    return R*np.cos(lat) *np.cos(long),\
           R*np.cos(lat) *np.sin(long),\
           R*np.sin(lat)

async def run():
    drone = System()

    await drone.connect(system_address="udp://:14540")
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered!")
            break
    print_mission_progress_task = asyncio.ensure_future(
        print_mission_progress(drone))

    running_tasks = [print_mission_progress_task]
    termination_task = asyncio.ensure_future(
        observe_is_in_air(drone, running_tasks))
    mission_items = []
    mission_items.append(MissionItem(39.8719220,32.7321097,
                                     2,
                                     1.1,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     3.5,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(39.8719225	,32.7321684,
                                     2,
                                     1.1,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     3.5
                                     ,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(39.8718778,32.7321707,
                                     2,
                                     1.1,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     3,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(39.8718785,32.7321121,
                                     2,
                                     1.1,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     3,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(39.8719220, 32.7321097,
                                     2,
                                     1.1,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     3,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(39.8719220, 32.7321097,
                                     7,
                                     1.1,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     3.5
                                     ,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(39.8719225, 32.7321684,
                                     7,
                                     1.1,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     3,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(39.8718778, 32.7321707,
                                     7,
                                     1.1,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     3,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(39.8718785, 32.7321121,
                                     7,
                                     1.1,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     3,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(39.8719220, 32.7321097,
                                     7,
                                     1.1,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     3.5,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))

    mission_items.append(MissionItem(39.8719225, 32.7321684,
                                     7,
                                     1.1,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     3,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(39.8719225, 32.7321684,
                                     2,
                                     1.1,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     3,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(39.8718778, 32.7321707,
                                     2,
                                     1.1,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     3,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(39.8718778, 32.7321707,
                                     7,
                                     1.1,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     3,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(39.8718785, 32.7321121,
                                     7,
                                     1.1,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     3,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(39.8718785, 32.7321121,
                                     2,
                                     1.1,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     10,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))

    mission_items.append(MissionItem(39.8719220,32.7321097,
                                     5,
                                     1.1,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     3,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(39.8719102,32.7321667,
                                      5,
                                     1.1,
                                      True,
                                      float('nan'),
                                      float('nan'),
                                      MissionItem.CameraAction.NONE,
                                      3,
                                      float('nan'),
                                      float('nan'),
                                      float('nan'),
                                      float('nan')))
    mission_items.append(MissionItem(39.8718790,32.7321248,
                                      5,
                                     1.1,
                                      True,
                                      float('nan'),
                                      float('nan'),
                                      MissionItem.CameraAction.NONE,
                                      3,
                                      float('nan'),
                                      float('nan'),
                                      float('nan'),
                                      float('nan')))
    mission_items.append(MissionItem(39.8719220, 32.7321097,
                                      5,
                                      1.1,
                                      True,
                                      float('nan'),
                                      float('nan'),
                                      MissionItem.CameraAction.NONE,
                                      3,
                                      float('nan'),
                                      float('nan'),
                                      float('nan'),
                                      float('nan')))

    mission_plan = MissionPlan(mission_items)
    print("-- Uploading mission")
    await drone.mission.upload_mission(mission_plan)
    await drone.mission.set_return_to_launch_after_mission(True)
    print("-- Arming")
    await drone.action.arm()
    print("-- Starting mission")
    await drone.mission.start_mission()
    await drone.mission.get_return_to_launch_after_mission()
    await termination_task
async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: "
              f"{mission_progress.current}/"
              f"{mission_progress.total}")


async def observe_is_in_air(drone, running_tasks):
    was_in_air = False
    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()
            return
if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
