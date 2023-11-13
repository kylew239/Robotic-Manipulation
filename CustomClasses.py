class Point:
    def __init__(self,
                 x: float = 0.0,
                 y: float = 0.0,
                 z: float = 0.0):
        self.x = x
        self.y = y
        self.z = z

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

class Config:
    def __init__(self,
                 pos: Point,
                 joint: ArmAng,
                 wheel: WheelAng):
        self.pos = chassis
        self.joint = joint
        self.wheel = wheel

class wheelVel:
    def __init__(self,
                 u1: float,
                 u2: float,
                 u3: float,
                 u4: float):
        self.u1 = u1
        self.u2 = u2
        self.u3 = u3
        self.u4 = u4

class jointVel:
    def __init__(self,
                 th1: float,
                 th2: float,
                 th3: float,
                 th4: float,
                 th5: float):
        self.th1 = th1
        self.th2 = th2
        self.th3 = th3
        self.th4 = th4
        self.th5 = th5

class Controls:
    pass