import numpy as np

def map(input, minInput, maxInput, minOutput, maxOutput):
    return minOutput + (maxOutput-minOutput) / (minOutput-minInput) *(input - maxInput)

class Vector3():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def distance(self, other):
        return Vector3(other.x - self.x, other.y - self.y, other.z - self.z)

    def __add__(self, other):
        return Vector3(other.x + self.x, other.y + self.y, other.z + self.z)
    
    def __minus__(self, other):
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __print__(self):
        print(self.x + ", " + self.y + ", " + self.z)


class Servo():
    def __init__(self, angle, midAngle, side, direction):
        self.angle = angle
        self.midAngle = midAngle
        self.side = side
        self.direction = direction # +1: forward, add; -1: backwards, subtract

    def moveServo(self, moveAngle):
        if (self.angle + moveAngle*self.direction) > 150 or (self.angle + moveAngle*self.direction) < 90:
            self.direction *= -1
        else:
            self.angle += moveAngle * self.direction

    def moveFemurServo(self, moveAngle):
        if (self.angle + moveAngle*self.direction) > 120 or (self.angle + moveAngle*self.direction) < 90:
            self.direction *= -1
        else:
            self.angle += moveAngle * self.direction

    def moveTo(self, angle):
        self.angle = angle
            

class Leg():
    def __init__(self, side):
        self.side = side
        if self.side == "L":
            self.factor = -1
        elif self.side == "R":
            self.factor = 1
        else:
            self.factor = 0
            print("invalid factor")
            
        self.coxa = Servo(90, 90, side, 1)
        self.femur = Servo(90, 90, side, 1)
        self.tibia = Servo(-45, 90, side, -1)
        self.basePos = Vector3(0,0,0)
        self.coxaPos = Vector3(0,0,0)
        self.femurPos = Vector3(0,0,0)
        self.tibiaPos = Vector3(0,0,0)

    def LegFK(self, correctionAng):
        theta = self.coxa.angle - 90 + correctionAng
        phi = self.femur.angle - 90
        psi = self.tibia.angle
        self.basePos = Vector3(self.factor*67, 0, 0)
        self.coxaPos = Vector3(self.basePos.x + self.factor*(np.cos(np.deg2rad(theta)) * 62),
                               self.basePos.y + np.sin(np.deg2rad(theta)) * 62,
                               self.basePos.z + 0)
        self.femurPos = Vector3(self.coxaPos.x + self.factor*(np.cos(np.deg2rad(phi)) * 83 * np.cos(np.deg2rad(theta))),
                                self.coxaPos.y + np.cos(np.deg2rad(phi)) * 83 * np.sin(np.deg2rad(theta)),
                                self.coxaPos.z + np.sin(np.deg2rad(phi)) * 83)
        self.tibiaPos = Vector3(self.femurPos.x + self.factor*(np.cos(np.deg2rad(phi - psi)) * 112 * np.cos(np.deg2rad(theta))),
                                self.femurPos.y + np.cos(np.deg2rad(phi - psi)) * 112 * np.sin(np.deg2rad(theta)),
                                self.femurPos.z - np.sin(np.deg2rad(phi - psi)) * 112)

        # print(
        #     self.coxa.angle,
        #     correctionAng,
        #     self.tibiaPos.x,
        #     self.tibiaPos.y,
        #     self.tibiaPos.z
        # )

        fk_x = [self.basePos.x, self.coxaPos.x, self.femurPos.x, self.tibiaPos.x]
        fk_y = [self.basePos.y, self.coxaPos.y, self.femurPos.y, self.tibiaPos.y]
        fk_z = [self.basePos.z, self.coxaPos.z, self.femurPos.z, self.tibiaPos.z]

        return fk_x, fk_y, fk_z
    

class FakeRobot():
    def __init__(self):
        self.position = Vector3(0,0,0)
        self.LegA = Leg("R")
        self.LegB = Leg("R")
        self.LegC = Leg("L")
        self.LegD = Leg("L")
        self.LegE = Leg("L")
        self.LegF = Leg("R")
    
    def tripodWalkRight(self):
        self.LegC.coxa.moveServo(5)
        self.LegA.coxa.moveServo(5)
        self.LegE.coxa.moveServo(5)

        self.LegC.femur.moveFemurServo(-5)
        self.LegA.femur.moveFemurServo(-5)
        self.LegE.femur.moveFemurServo(-5)

        self.LegC.tibia.moveServo(-5)
        self.LegA.tibia.moveServo(-5)
        self.LegE.tibia.moveServo(-5)


    def tripodWalkLeft(self):
        self.LegD.coxa.moveServo(5)
        self.LegB.coxa.moveServo(5)
        self.LegF.coxa.moveServo(5)

        self.LegD.femur.moveFemurServo(-5)
        self.LegB.femur.moveFemurServo(-5)
        self.LegF.femur.moveFemurServo(-5)

        self.LegD.tibia.moveServo(-5)
        self.LegB.tibia.moveServo(-5)
        self.LegF.tibia.moveServo(-5)


    def get_data(self):

        self.position.y += 0.2

        matrix = [
            [self.LegA.coxa.angle, self.LegA.femur.angle, self.LegA.tibia.angle],
            [self.LegB.coxa.angle, self.LegB.femur.angle, self.LegB.tibia.angle],
            [self.LegC.coxa.angle, self.LegC.femur.angle, self.LegC.tibia.angle],
            [self.LegD.coxa.angle, self.LegD.femur.angle, self.LegD.tibia.angle],
            [self.LegE.coxa.angle, self.LegE.femur.angle, self.LegE.tibia.angle],
            [self.LegF.coxa.angle, self.LegF.femur.angle, self.LegF.tibia.angle]
        ]
        return matrix
