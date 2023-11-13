import numpy as np
import modern_robotics as mr
from CustomClasses import *
import math

# kinematics of youBot
l = 0.235
w = 0.15
r = 0.0475
Tb0 = [[1, 0, 0, 0.1662],
       [0, 1, 0, 0],
       [0, 0, 1, 0.0026],
       [0, 0, 0, 1]]
M0e = [[1, 0, 0, 0.033],
       [0, 1, 0, 0],
       [0, 0, 1, 0.6546],
       [0, 0, 0, 1]]

# Screw Axes for home config
B1 = [0,  0, 1,       0, 0.033, 0]
B2 = [0, -1, 0, -0.5076,     0, 0]
B3 = [0, -1, 0, -0.3526,     0, 0]
B4 = [0, -1, 0, -0.2176,     0, 0]
B5 = [0,  0, 1,       0,     0, 0]

# H for mecanum drive
mecanum = np.matrix([[-l-w, 1, -1],
                     [l+w, 1,  1],
                     [l+w, 1, -1],
                     [-l-w, 1,  1]])


def NextState(currentConf: RobotState,
              currentVels: Velocities,
              dt: float,
              maxAngVel: float) -> RobotState:
    # Do first-order euler step for joint angles
    currentConf.joint = firstEulerStep(currentConf.joint,
                                       currentVels.joint,
                                       dt,
                                       maxAngVel)

    # Do first-order euler step for wheel angles
    currentConf.wheel = firstEulerStep(currentConf.wheel,
                                       currentVels.wheel,
                                       dt,
                                       maxAngVel)
    
    # Update chassis position
    currentConf.pos = calculateChassisPose(currentConf.pos,
                                           currentVels.wheel,
                                           dt,
                                           maxAngVel)

    return currentConf


def firstEulerStep(item: ArmAng or WheelAng,
                   itemVels: jointVel or wheelVel,
                   dt: float,
                   maxAngVel: float):
    """Calculate First euler step for each value in object.

    :param item: Object to calculate
    :param dt: time step
    :param maxAngVel: Maximum angular velocity
    :return: Object with Updated Values
    """
    # Iterate through each value in the object
    for key in vars(item):
        # If velocity is under the limit, use the velocity
        if abs(getattr(itemVels, key)) < maxAngVel:
            setattr(item,
                    key,
                    getattr(item, key) + getattr(itemVels, key)*dt)
        # Else use velocity limit
        else:
            setattr(item,
                    key,
                    getattr(item, key) + np.sign(getattr(itemVels, key))*maxAngVel)
    return item


def calculateTsb(phi: float,
                 x: float,
                 y: float):
    """Calculate Tsb given the q vector

    :param phi: Angle of rotation about the z axis of {s}
    :param x: x distance to {s}
    :param y: y distance to {s}
    :return: Tsb
    """
    return [[math.cos(phi), -math.sin(phi), 0, x],
            [math.sin(phi),  math.cos(phi), 0, y],
            [0,              0, 1, 0.0963],
            [0,              0, 0, 1]]


def calculateChassisPose(pose: Pose,
                         wheelVels: wheelVel,
                         dt: float,
                         maxAngVel: float):
    """Update position of chassis given the current position, wheel velocities, and dt

    :param pose: Current position of the robot
    :param wheelVels: Angular velocities of each wheel
    :param dt: time step
    :param maxAngVel: Maximum angular velocity
    :return: updated position
    """
    u = np.clip(np.array([[wheelVels.th1, wheelVels.th2, wheelVels.th3, wheelVels.th4]]),
                -maxAngVel,
                maxAngVel)
    #TODO: Clean up comments and check if math is right
    # chassisVels = np.divide(u.T, mecanum) * r

    chassisVels = np.matmul((u*r), np.linalg.pinv(mecanum).T)
    pose.phi += chassisVels[0, 0] * dt
    pose.x += chassisVels[0, 1] * dt
    pose.y += chassisVels[0, 2] * dt
    # print("1,", mecanum.shape)
    # print("2,", chassisVels.shape)
    u_calc = np.matmul(mecanum, chassisVels.T)
    # print("u_calculated: ", chassisVels.shape)
    return pose

if __name__ == "__main__":
    robot = RobotState()
    velocities = Velocities(wheel=wheelVel(-10, 10, 10, -10))
    t = 0
    while t < 1000:
        robot = NextState(robot, velocities, 0.01, 50)
        t += 1
    print(f"X: {robot.pos.x}, y: {robot.pos.y}, phi: {robot.pos.phi}")
