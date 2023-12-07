import numpy as np
import modern_robotics as mr
from CustomClasses import *
import math
from typing import List, Any
import csv
import matplotlib.pyplot as plt

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
Blist = np.array([[0,     0,       0,       0,      0],
                  [0,    -1,      -1,      -1,     0],
                  [1,     0,       0,       0,      1],
                  [0,    -0.5076,  -0.3526, -0.2176, 0],
                  [0.033, 0,       0,       0,      0],
                  [0,     0,       0,       0,      0]])

# Mecanum info
x = np.array([l, l, -l, -l])
y = np.array([w, -w, -w, w])
gamma = np.array([-np.pi/4, np.pi/4, -np.pi/4, np.pi/4])
beta = 0


def get_h(r, x, y, gamma, beta, phi):
    h = (1/((r)*(np.cos(gamma)))) * np.array([(x*(np.sin(beta+gamma))-y*(np.cos(beta+gamma))),
                                              (np.cos(beta+gamma+phi)),
                                              (np.sin(beta+gamma+phi))])
    return np.transpose(h)


def state_to_array(state: RobotState):
    q = np.array([state.pos.phi, state.pos.x, state.pos.y])
    j = np.array([state.joint.th1, state.joint.th2,
                  state.joint.th3, state.joint.th4, state.joint.th5])
    w = np.array([state.wheel.th1, state.wheel.th2,
                 state.wheel.th3, state.wheel.th4])
    return q, j, w


def vels_to_array(vels: Velocities):
    wdot = np.array([vels.wheel.th1, vels.wheel.th2,
                    vels.wheel.th3, vels.wheel.th4])
    jdot = np.array([vels.joint.th1, vels.joint.th2,
                     vels.joint.th3, vels.joint.th4, vels.joint.th5])
    return jdot, wdot


def NextState(currentConf: RobotState,
              currentVels: Velocities,
              dt: float,
              maxAngVel: float) -> RobotState:
    # Extracting variables
    pose = currentConf.pos
    wheelVels = currentVels.wheel
    q, j, w = state_to_array(currentConf)
    jdot, wdot = vels_to_array(currentVels)

    # Do first-order euler step for joint angles
    j_next = j + jdot*dt
    joint = ArmAng(j_next[0], j_next[1],
                   j_next[2], j_next[3], j_next[4])

    # Do first-order euler step for wheel angles
    w_next = w + wdot*dt
    wheel = WheelAng(w_next[0], w_next[1], w_next[2], w_next[3])

    # Update chassis position
    u = np.clip(np.array([[wheelVels.th1, wheelVels.th2, wheelVels.th3, wheelVels.th4]]),
                -maxAngVel,
                maxAngVel)
    phi = pose.phi
    h1 = get_h(r, x[0], y[0], gamma[0], beta, phi)
    h2 = get_h(r, x[1], y[1], gamma[1], beta, phi)
    h3 = get_h(r, x[2], y[2], gamma[2], beta, phi)
    h4 = get_h(r, x[3], y[3], gamma[3], beta, phi)
    H_phi = np.array([h1, h2, h3, h4])

    q_dot = np.linalg.pinv(H_phi)@u.T
    q_next = q + (q_dot*dt).T
    pos = Pose(phi=q_next[0, 0], x=q_next[0, 1], y=q_next[0, 2])

    return RobotState(pos, joint, wheel)


def calculateTsb(phi: float,
                 x: float,
                 y: float):
    return [[math.cos(phi), -math.sin(phi), 0, x],
            [math.sin(phi),  math.cos(phi), 0, y],
            [0,              0, 1, 0.0963],
            [0,              0, 0, 1]]


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

    # Trajectory 8: Move gripper back to standoff position
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


def FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt):
    T_err = mr.TransInv(X)@Xd
    X_err = mr.MatrixLog6(T_err)
    X_err = mr.se3ToVec(X_err)

    # estimated error over time (need to multiply by Ki)
    err = X_err*dt

    X_log = mr.TransInv(Xd)@Xd_next
    X_log = mr.MatrixLog6(X_log)

    Vd = (1/dt)*X_log
    Vd = mr.se3ToVec(Vd)

    # calculate for Twist in end-effector frame
    T_ad = mr.Adjoint(mr.TransInv(X)@Xd)

    # calculate for Ve
    Ve = T_ad@Vd + Kp@X_err + Ki@err

    return Ve, X_err


def generate_se3(traj):
    """Generate SE3 from trajectory."""
    T_mat = np.array([[traj[0], traj[1], traj[2], traj[9]],
                      [traj[3], traj[4], traj[5], traj[10]],
                      [traj[6], traj[7], traj[8], traj[11]],
                      [0, 0, 0, 1]])

    return T_mat


def state_to_vec(state: RobotState,
                 grip: float):
    """Create a 13-vector using RobotState."""
    return [state.pos.phi, state.pos.x, state.pos.y,
            state.joint.th1, state.joint.th2, state.joint.th3, state.joint.th4, state.joint.th5,
            state.wheel.th1, state.wheel.th2, state.wheel.th3, state.wheel.th4, grip]


if __name__ == "__main__":
    """Main loop of the program."""
    print("starting")
    # Generate Trajectory
    Tsei = np.array([(0, 0, 1,   0),
                     (0, 1, 0,   0),
                     (-1, 0, 0, 0.5),
                     (0, 0, 0,   1)])
    Tsci = np.array([[1, 0, 0,     1],
                     [0, 1, 0,     1],
                     [0, 0, 1, 0.025],
                     [0, 0, 0,     1]])
    Tscf = np.array([[1, 0, 0,   -1],
                     [0, 1, 0,   -1],
                     [0, 0, 1, 0.025],
                     [0, 0, 0,     1]])
    Tce_grasp = np.array([[-0.707, 0,  0.707, 0.005],
                          [0, 1,      0,     0],
                          [-0.707, 0, -0.707, -0.01],
                          [0, 0,      0,     1]])
    Tce_stand = np.array([[-0.707, 0,  0.707, 0.005],
                          [0, 1,      0,     0],
                          [-0.707, 0, -0.707, 0.15],
                          [0, 0,      0,    1]])
    print("Generating Trajectory")
    trajList = TrajectoryGenerator(Tsei, Tsci, Tscf, Tce_grasp, Tce_stand)

    kp = np.identity(6) * 0.05
    ki = np.identity(6) * 3.0
    state = RobotState(pos=Pose(x=0.0, y=0.0, phi=0.0),
                       joint=ArmAng(0.1, 0.1, 0.1, 0.1, 0.1),
                       wheel=WheelAng(0.0, 0.0, 0.0, 0.0))
    dt = 0.01
    X_err_total = 0
    x_err_list = []
    final_trajectory = []

    # Loop through each trajectory
    print("calculating Errors")
    for i in range(len(trajList)-1):
        thetas = state.joint
        thetalist = [thetas.th1, thetas.th2,
                     thetas.th3, thetas.th4, thetas.th5]
        
        Tsb = calculateTsb(state.pos.phi, state.pos.x, state.pos.y)
        T0e = mr.FKinBody(M0e, Blist, thetalist)
        Tse = Tsb@Tb0@M0e

        X = Tsb@Tb0@T0e
        Xd = generate_se3(trajList[i])
        Xd_next = generate_se3(trajList[i+1])

        Vd, X_err = FeedbackControl(X,
                                    Xd,
                                    Xd_next,
                                    kp,
                                    ki,
                                    0.01)
        x_err_list.append(X_err)

        # H0
        phi = 0.0
        h1 = get_h(r, x[0], y[0], gamma[0], beta, phi)
        h2 = get_h(r, x[1], y[1], gamma[1], beta, phi)
        h3 = get_h(r, x[2], y[2], gamma[2], beta, phi)
        h4 = get_h(r, x[3], y[3], gamma[3], beta, phi)
        H0 = np.array([h1, h2, h3, h4])

        # Jacobian
        zeros = np.zeros((1, 4))
        F = np.linalg.pinv(H0)
        F = np.vstack((zeros, zeros, F, zeros))
        base = mr.Adjoint(mr.TransInv(T0e)@mr.TransInv(Tb0))@F
        arm = mr.JacobianBody(Blist, thetalist)
        J = np.hstack((base, arm))

        # Get speed
        speed = np.linalg.pinv(J)@Vd
        speed = Velocities(wheel=wheelVel(speed[0], speed[1], speed[2], speed[3]),
                           joint=jointVel(speed[4], speed[5], speed[6], speed[7], speed[8]))

        # Update current state
        state = NextState(state, speed, dt, 5.0)
        final_trajectory.append(state_to_vec(state, trajList[i][12]))

    print("writing to csv")
    write_to_csv(final_trajectory, "test.csv")
    write_to_csv(x_err_list, "error.csv")
    
    plt.plot(np.arange(0, len(x_err_list) * 0.01, 0.01), x_err_list)
    plt.legend(['roll', 'pitch', 'yaw', 'x', 'y', 'z'])
    plt.title("Error Over Time (Overshoot)")
    plt.xlabel("Time (s)")
    plt.ylabel("X Error (rad)")
    plt.show()
    print("done")
