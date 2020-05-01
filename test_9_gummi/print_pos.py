#thx tudor for the maths ^^
import urx
import time
from scipy.spatial.transform import Rotation as R
import numpy as np

#from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper
from robotiq_hande import Robotiq_HandE


rob = urx.Robot("192.168.0.100"); 

pose = rob.getl()
xyz = rob.get_pos()
orient = rob.get_orientation()
#print("robot orient is at: ", orient)

r = R.from_rotvec(np.array([pose[3],pose[4],pose[5]]))
rpy = r.as_euler('XYZ',degrees=False)

#return {'x': pose[0] * 1000, 'y': pose[1]  * 1000, 'z':pose[2]  * 1000, 'rx': pose[3] * 57.2958, 'ry': pose[4] * 57.2958, 'rz': pose[5] * 57.2958}
print ((pose[0], pose[1], pose[2], rpy[0], rpy[1], rpy[2]))
print(pose)

rob.close()