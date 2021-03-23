import matplotlib.pyplot as plt
from test_robot_model import *

robot.Target[16].p = np.array([0.3, 0.1, 0])
robot.Target[16].R = robot.rpy2rot(0, 30, 0)
# print(robot.ulink[16].p)
robot.InverseKinematics(16)
# print(robot.ulink[16].p)

robot.Target[22].p = np.array([-0.3, -0.1, 0])
robot.Target[22].R = robot.rpy2rot(0, 30, 0)
robot.InverseKinematics(22)
# print(robot.ulink[22].p)

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1, projection='3d')
ax.set_xlim(left=-0.3, right=0.3)
ax.set_ylim(bottom=-0.3, top=0.3)
ax.set_zlim(bottom=0, top=0.6)
robot.DrawAllJoints(ax, 1)
plt.show()
