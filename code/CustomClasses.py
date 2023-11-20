class Pose:
    def __init__(self,
                 x: float = 0.0,
                 y: float = 0.0,
                 phi: float = 0.0):
        self.x = x
        self.y = y
        self.phi = phi


class ArmAng:
    def __init__(self,
                 th1: float = 0.0,
                 th2: float = 0.0,
                 th3: float = 0.0,
                 th4: float = 0.0,
                 th5: float = 0.0):
        self.th1 = th1
        self.th2 = th2
        self.th3 = th3
        self.th4 = th4
        self.th5 = th5


class WheelAng:
    def __init__(self,
                 th1: float = 0.0,
                 th2: float = 0.0,
                 th3: float = 0.0,
                 th4: float = 0.0):
        self.th1 = th1
        self.th2 = th2
        self.th3 = th3
        self.th4 = th4


class RobotState:
    def __init__(self,
                 pos: Pose = Pose(x=0.0,
                                  y=0.0,
                                  phi=0.0),
                 joint: ArmAng = ArmAng(0.0,
                                        0.0,
                                        0.0,
                                        0.0,
                                        0.0),
                 wheel: WheelAng = WheelAng(0.0,
                                            0.0,
                                            0.0,
                                            0.0)):
        self.pos = pos
        self.joint = joint
        self.wheel = wheel


class wheelVel:
    def __init__(self,
                 th1: float = 0.0,
                 th2: float = 0.0,
                 th3: float = 0.0,
                 th4: float = 0.0):
        self.th1 = th1
        self.th2 = th2
        self.th3 = th3
        self.th4 = th4


class jointVel:
    def __init__(self,
                 th1: float = 0.0,
                 th2: float = 0.0,
                 th3: float = 0.0,
                 th4: float = 0.0,
                 th5: float = 0.0):
        self.th1 = th1
        self.th2 = th2
        self.th3 = th3
        self.th4 = th4
        self.th5 = th5


class Velocities:
    def __init__(self,
                 wheel: wheelVel = wheelVel(0.0,
                                            0.0,
                                            0.0,
                                            0.0),
                 joint: jointVel = jointVel(0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0)):
        self.wheel = wheel
        self.joint = joint
