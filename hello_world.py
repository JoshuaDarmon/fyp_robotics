import pybullet as p
from time import sleep
import pybullet_data
import numpy as np
import imageio_ffmpeg

from environment.panda_robot.panda_robot import PandaRobot

t = 0

physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
tableId = p.loadURDF("table/table.urdf",[0,1,-0.3])
#0.65 to be on top of the table
cube_id = p.loadURDF("cube.urdf", basePosition=[0, 1.2, 0.35], globalScaling=0.2)



#franka = p.loadURDF("franka_emika_panda_pybullet/panda_robot/model_description/panda_with_gripper.urdf",[1,0,0])

#franka2 = p.loadURDF("PandaRobot.jl/deps/Panda/panda.urdf",[0,1,0])
franka3 = PandaRobot(include_gripper=True)
#target position/ focus is going to be the middle of the table
#this target position is going to be used for both arms
cam_target_pos = [0, 1, 0.65]
cam_distance = 1
cam_yaw, cam_pitch, cam_roll = 0, -15, 0
cam_width, cam_height = 480, 360

cam_up, cam_up_axis_idx, cam_near_plane, cam_far_plane, cam_fov = [0, 0, 1], 2, 0.01, 100, 60


vid = imageio_ffmpeg.write_frames('vid.mp4', (cam_width, cam_height), fps=30)
vid.send(None) # seed the video writer with a blank frame



useRealTimeSimulation = 0

if (useRealTimeSimulation):
  p.setRealTimeSimulation(1)

while t<900:
#for t in range(900):
  if (useRealTimeSimulation):
    p.setGravity(0, 0, -10)
    sleep(0.01)  # Time in seconds.
  else:
    t += 1
    p.stepSimulation()

    if t % 30 == 0: # PyBullet default simulation time step is 240fps, but we want to record video at 30fps.
        cam_view_matrix = p.computeViewMatrixFromYawPitchRoll(cam_target_pos, cam_distance, cam_yaw, cam_pitch, cam_roll, cam_up_axis_idx)
        cam_projection_matrix = p.computeProjectionMatrixFOV(cam_fov, cam_width*1./cam_height, cam_near_plane, cam_far_plane)
        image = p.getCameraImage(cam_width, cam_height, cam_view_matrix, cam_projection_matrix)[2]#[:, :, :3]
        
        p.setJointMotorControl2(franka3.robot_id,8,p.POSITION_CONTROL,0.5,force=100)
        p.setJointMotorControl2(franka3.robot_id,9,p.POSITION_CONTROL,0.5,force=100)
    #camera()
    sleep(0.01)


#vid.close()
#p.disconnect()