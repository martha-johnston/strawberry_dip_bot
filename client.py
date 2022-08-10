import asyncio

from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions, dial_direct
from viam.components.arm import Arm, WorldState, Pose, JointPositions
from viam.components.gripper import Gripper
from viam.services.motion import MotionClient
from viam.services.types import ServiceType
from viam.proto.api.common import Pose, PoseInFrame, WorldState, GeometriesInFrame, Geometry, RectangularPrism, ResourceName
from viam.services.vision import DetectorConfig, DetectorType, VisionClient


async def client():
  creds = Credentials(
      type='robot-location-secret',
      payload='lauqrt4op4x6ubhji867wue0qsnqdq76x00ljfv7vcoyq7mi')
  opts = RobotClient.Options(
      refresh_interval=0,
      dial_options=DialOptions(credentials=creds)
  )
  async with await RobotClient.at_address(
    'FriendArm-main.rdt5n4brox.local.viam.cloud:8080',
    opts) as robot:
      print('Resources:')
      print(robot.resource_names)

  #GRIPPER TEST
  # soft_gripper = Gripper.from_robot(robot, 'gripper-server:soft_gripper')
  # await soft_gripper.open()
  # await asyncio.sleep(1.5)
  # await soft_gripper.stop()
  # return

  # PROGRAM

  arm = Arm.from_robot(robot, 'ur')
  gripper = Gripper.from_robot(robot, 'gripper-server:soft_gripper')
  vision = robot.get_service(ServiceType.VISION)
  motion = robot.get_service(ServiceType.MOTION)

  arm_name = "ur"
  ref_name = arm_name + "_offset"

  # initalize motion planning info
  geom = Geometry(center=Pose(x=-500, y=0, z=-180), box=RectangularPrism(width_mm =2000, length_mm =2000, depth_mm =10))
  geomFrame = GeometriesInFrame(reference_frame=ref_name, geometries=[geom])

  worldstate = WorldState(obstacles=[geomFrame])

  for resname in robot.resource_names:
    if resname.name == arm_name:
       armRes = resname

  names = await vision.get_detector_names()
  print(names)

  detect_strawberry = await vision.get_detections_from_camera("cam", "detector_color")
#   print(detect_strawberry)

  count=0

  YCONV = 1.67
  XCONV = 1.54

  # probably some type of loop starting here
  while(len(detect_strawberry)!=0):

    y = (detect_strawberry[0].x_max + detect_strawberry[0].x_min)/2
    x = (detect_strawberry[0].y_max + detect_strawberry[0].y_min)/2

    # move to above the strawberry
    cam_pose = Pose(x=-x*XCONV, y=-y*YCONV, z=100, o_x=0, o_y=0, o_z=-1)
    await motion.move(component_name=armRes, destination=PoseInFrame(reference_frame="cam", pose=cam_pose),world_state=worldstate)

    # move down, grab, move back up
    cam_pose.z -= 90
    await motion.move(component_name=armRes, destination=PoseInFrame(reference_frame="cam", pose=cam_pose),world_state=worldstate)
    await gripper.grab()
    await asyncio.sleep(1)
    await gripper.stop()
    cam_pose.z += 90
    await motion.move(component_name=armRes, destination=PoseInFrame(reference_frame="cam", pose=cam_pose),world_state=worldstate)

    # return

    # look for chocolate detections
    detect_chocolate = await vision.get_detections_from_camera("cam", "detector_chocolate")
    # print(detect_chocolate)

    x_choc = 350 #(detect_chocolate[0].x_max - detect_chocolate[0].x_min) + detect_chocolate[0].x_min
    y_choc = 400 #(detect_chocolate[0].y_max - detect_chocolate[0].y_min) + detect_chocolate[0].y_min

    # move to above the chocolate
    choc_pose = Pose(x=x_choc*XCONV, y=-y_choc*YCONV, z=300, o_x=0, o_y=0, o_z=-1) # change z based on chocolate warmer height
    await motion.move(component_name=armRes, destination=PoseInFrame(reference_frame="world", pose=choc_pose),world_state=worldstate)

    # dip
    choc_pose.z -= 200
    await motion.move(component_name=armRes, destination=PoseInFrame(reference_frame="world", pose=choc_pose),world_state=worldstate)
    await asyncio.sleep(1)
    choc_pose.z += 200
    await motion.move(component_name=armRes, destination=PoseInFrame(reference_frame="world", pose=choc_pose),world_state=worldstate)

    # move to plate pose
    plate_pose = Pose(x=370, y=100, z=110, o_x=0, o_y=0, o_z=-1) # move down with z
    plate_pose.x += (10*count)
    await motion.move(component_name=armRes, destination=PoseInFrame(reference_frame="world", pose=plate_pose),world_state=worldstate)
    # plate_pose.z -= 90
    # await motion.move(component_name=armRes, destination=PoseInFrame(reference_frame="world", pose=plate_pose),world_state=worldstate)

    # open gripper
    await gripper.open()
    await asyncio.sleep(1)
    await gripper.stop()

    # plate_pose.z += 90
    # await motion.move(component_name=armRes, destination=PoseInFrame(reference_frame="world", pose=plate_pose),world_state=worldstate)

    detect_strawberry = await vision.get_detections_from_camera("cam", "detector_color")
    print(detect_strawberry)
    count+=1

  await robot.close()

if __name__ == '__main__':
  asyncio.run(client())
