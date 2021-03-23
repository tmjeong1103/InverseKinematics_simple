from IK_simulator_test import *

Rx = np.array([1,0,0])
Ry = np.array([0,1,0])
Rz = np.array([0,0,1])

###SetupRobot
robot = RobotObject()

##Body Parameter
#OP_Middle_Hip (Base)
robot.ulink[1].p = [0, 0, 0.7]
robot.ulink[1].R = np.eye(3)

##Left Leg Parameter
#OP_L_HIP_1
robot.ulink[11].b = [0, 0.1, 0]
robot.ulink[11].a = Rz
#OP_L_HIP_2
robot.ulink[12].b = [0, 0, 0]
robot.ulink[12].a = Rx
#OP_L_HIP_3
robot.ulink[13].b = [0, 0, 0]
robot.ulink[13].a = Ry
#OP_L_Knee
robot.ulink[14].b = [0, 0, -0.3]
robot.ulink[14].a = Ry
#OP_L_Ankle_1
robot.ulink[15].b = [0, 0, -0.3]
robot.ulink[15].a = Ry
#OP_L_Ankle_2
robot.ulink[16].b = [0, 0, 0]
robot.ulink[16].a = Rx

##Right Leg Parameter
#OP_R_HIP_1
robot.ulink[17].b = [0, -0.1, 0]
robot.ulink[17].a = Rz
#OP_R_HIP_2
robot.ulink[18].b = [0, 0, 0]
robot.ulink[18].a = Rx
#OP_R_HIP_3
robot.ulink[19].b = [0, 0, 0]
robot.ulink[19].a = Ry
#OP_R_Knee
robot.ulink[20].b = [0, 0, -0.3]
robot.ulink[20].a = Ry
#OP_R_Ankle_1
robot.ulink[21].b = [0, 0, -0.3]
robot.ulink[21].a = Ry
#OP_R_Ankle_2
robot.ulink[22].b = [0, 0, 0]
robot.ulink[22].a = Rx

robot.forward_kinematics(1)

### SET non sigularity pose (joint variable)

#Left_Leg
robot.ulink[13].q = np.deg2rad(-5)
robot.ulink[14].q = np.deg2rad(10)
robot.ulink[15].q = np.deg2rad(-5)
#Right_Leg
robot.ulink[19].q = np.deg2rad(-5)
robot.ulink[20].q = np.deg2rad(10)
robot.ulink[21].q = np.deg2rad(-5)