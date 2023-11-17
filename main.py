import numpy as np
import modern_robotics as mr
from CustomClasses import *
import math
from typing import List, Any
import csv

# kinematics of youBot
l = 0.235
w = 0.15
r = 0.0475
Tb0 = np.array([[1, 0, 0, 0.1662],
                [0, 1, 0, 0],
                [0, 0, 1, 0.0026],
                [0, 0, 0, 1]])
M0e = np.array([[1, 0, 0, 0.033],
                [0, 1, 0, 0],
                [0, 0, 1, 0.6546],
                [0, 0, 0, 1]])

# Screw Axes for home config
B1 = [0,  0, 1,       0, 0.033, 0]
B2 = [0, -1, 0, -0.5076,     0, 0]
B3 = [0, -1, 0, -0.3526,     0, 0]
B4 = [0, -1, 0, -0.2176,     0, 0]
B5 = [0,  0, 1,       0,     0, 0]

# H for mecanum drive
mecanum = np.array([[-l-w, 1, -1],
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
    # TODO: Clean up comments and check if math is right
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


def TrajectoryGenerator(Tsei: List[List[float]],
                        Tsci: List[List[float]],
                        Tscf: List[List[float]],
                        Tce_grasp: List[List[float]],
                        Tce_stand: List[List[float]],
                        k: int = 1):
    """
    Generate a trajectory to pick up and drop off the block.

    Arguments:
        Tsei (4x4 Matrix) -- Initial configuration of the ee
        Tsci (4x4 Matrix) -- Initial configuration of the cube
        Tscf (4x4 Matrix) -- End configuration of the cube
        Tce_grasp (4x4 Matrix) -- ee's configuration relative to the cube\
                                  when grasping
        Tce_stand (4x4 Matrix) -- ee's configuratoin above the cube before\
                                  and after grasping
        k (int): Number of trajectory configurations per 0.01 seconds
    """
    trajListRaw = []  # list of output trajectories, unformatted
    gripperChangeIndex = []  # list of indices where the gripper state changes
    gripper_time = 0.65  # Motion time for closing/opening gripper
    gripper_time = int(gripper_time * 100)
    motion_time = 5.0  # Motion time between points
    vert_time = 1.0  # Motion time for moving up and down

    # Transforms needed
    Tsci_stand = Tsci@Tce_stand
    Tsci_grasp = Tsci@Tce_grasp
    Tscf_stand = Tscf@Tce_stand
    Tscf_grasp = Tscf@Tce_grasp

    # Trajectory 1: Move from initial config to standoff
    temp = mr.ScrewTrajectory(Tsei,
                              Tsci_stand,
                              motion_time,
                              motion_time/0.01,
                              5)
    for mat in temp:
        trajListRaw.append(mat)

    # Trajectory 2: Move down to grasp
    temp = mr.ScrewTrajectory(Tsci_stand,
                              Tsci_grasp,
                              vert_time,
                              vert_time/0.01,
                              5)
    for mat in temp:
        trajListRaw.append(mat)

    # Trajectory 3: Close gripper
    currState = trajListRaw[-1]
    gripperChangeIndex.append(len(trajListRaw))
    for i in range(0, gripper_time):
        trajListRaw.append(currState)

    # Trajectory 4: Move up to standoff
    temp = mr.ScrewTrajectory(Tsci_grasp,
                              Tsci_stand,
                              vert_time,
                              vert_time/0.01,
                              5)
    for mat in temp:
        trajListRaw.append(mat)

    # Trajectory 5: Move to standoff above end configuration
    temp = mr.ScrewTrajectory(Tsci_stand,
                              Tscf_stand,
                              motion_time,
                              motion_time/0.01,
                              5)
    for mat in temp:
        trajListRaw.append(mat)

    # Trajectory 6: Move gripper down to final position
    temp = mr.ScrewTrajectory(Tscf_stand,
                              Tscf_grasp,
                              vert_time,
                              vert_time/0.01,
                              5)
    for mat in temp:
        trajListRaw.append(mat)

    # Trajectory 7: Open gripper
    currState = trajListRaw[-1]
    gripperChangeIndex.append(len(trajListRaw))
    for i in range(0, gripper_time):
        trajListRaw.append(currState)

    # Trajectory 8: Move gripper back to standoff positio
    temp = mr.ScrewTrajectory(Tscf_grasp,
                              Tscf_stand,
                              vert_time,
                              vert_time/0.01,
                              5)
    for mat in temp:
        trajListRaw.append(mat)

    # Format trajList
    trajList = []
    grip = 0
    for index, T in enumerate(trajListRaw):
        # If begun grasping/releasing motion, update grip value
        if index in gripperChangeIndex:
            grip = 1 - grip

        # Creating variables rather than directing appending to
        # make debugging easier
        r11 = T[0][0]
        r12 = T[0][1]
        r13 = T[0][2]
        x = T[0][3]
        r21 = T[1][0]
        r22 = T[1][1]
        r23 = T[1][2]
        y = T[1][3]
        r31 = T[2][0]
        r32 = T[2][1]
        r33 = T[2][2]
        z = T[2][3]

        trajList.append([r11, r12, r13,
                         r21, r22, r23,
                         r31, r32, r33,
                         x, y, z, grip])
    return trajList


def write_to_csv(list: List[Any],
                 filename: str) -> None:
    """
    Write a list to a csv.

    Arguments:
        list (List(Any)) -- the list to write
        filename (str) -- Filename of the csv file
    """
    with open(filename, 'w') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerows(list)


if __name__ == "__main__":
    # Part 1 testing
    # robot = RobotState()
    # velocities = Velocities(wheel=wheelVel(-10, 10, 10, -10))
    # t = 0
    # while t < 1000:
    #     robot = NextState(robot, velocities, 0.01, 50)
    #     t += 1
    # print(f"X: {robot.pos.x}, y: {robot.pos.y}, phi: {robot.pos.phi}")

    # Part 2 testing
    Tsei = Tb0@M0e
    Tsci = np.array([[1, 0, 0, 1],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0.025],
                     [0, 0, 0, 1]])
    Tscf = np.array([[0, 1, 0, 0],
                     [-1, 0, 0, -1],
                     [0, 0, 1, 0.025],
                     [0, 0, 0, 1]])
    Tce_grasp = np.array([[-0.707, 0,  0.707, 0.005],
                          [     0, 1,      0, 0],
                          [-0.707, 0, -0.707, -0.02],
                          [     0, 0,      0, 1]])
    Tce_stand = np.array([[-0.707, 0,  0.707, 0.005],
                          [     0, 1,      0, 0],
                          [-0.707, 0, -0.707, 0.15],
                          [     0, 0,      0, 1]])

    traj = TrajectoryGenerator(Tsei,
                               Tsci,
                               Tscf,
                               Tce_grasp,
                               Tce_stand)
    write_to_csv(traj, "test.csv")
