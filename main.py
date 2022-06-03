import numpy as np
from robots.biped.robot import Biped
from robots.biped.walking import PreviewControl


def stand():
    biped = Biped()
    CoM_height = 0.45

    targetRPY = [0.0, 0.0, 0.0]
    targetPosL = [0.0, 0.065, -CoM_height]
    targetPosR = [0.0, -0.065, -CoM_height]
    biped.positionInitialize(initializeTime=0.2)

    while True:
        incline = biped.getIncline()
        biped.resetIncline(incline)
        targetRPY[1] = incline

        biped.setLegPositions(targetPosL, targetPosR, targetRPY)
        biped.oneStep()


def squat():
    biped = Biped()
    CoM_height = 0.45

    targetRPY = [0.0, 0.0, 0.0]
    targetPosL = [0.0, 0.065, -CoM_height]
    targetPosR = [0.0, -0.065, -CoM_height]
    biped.positionInitialize(initializeTime=0.1)

    dp = 0.002
    while True:
        incline = biped.getIncline()
        biped.resetIncline(incline)
        targetRPY[1] = incline

        for _ in range(100):
            biped.setLegPositions(targetPosL, targetPosR, targetRPY)
            biped.oneStep()
            targetPosL[2] += dp
            targetPosR[2] += dp

        for _ in range(100):
            biped.setLegPositions(targetPosL, targetPosR, targetRPY)
            biped.oneStep()
            targetPosL[2] -= dp
            targetPosR[2] -= dp


def jump(withTorsoTwist=False):
    biped = Biped()
    CoM_height = 0.45

    targetRPY = [0.0, 0.0, 0.0]
    targetPosL = [0.0, 0.065, -CoM_height]
    targetPosR = [0.0, -0.065, -CoM_height]
    biped.positionInitialize(initializeTime=0.1)

    dp = 0.0025
    dRPY = 0.0065
    while True:
        incline = biped.getIncline()
        biped.resetIncline(incline)
        targetRPY[1] = incline

        for _ in range(60):
            biped.setLegPositions(targetPosL, targetPosR, targetRPY)
            biped.oneStep()
            targetPosL[2] += dp
            targetPosR[2] += dp
            if withTorsoTwist:
                targetRPY[2] += dRPY

        for _ in range(60):
            biped.setLegPositions(targetPosL, targetPosR, targetRPY)
            biped.oneStep()
            targetPosL[2] -= dp
            targetPosR[2] -= dp
            if withTorsoTwist:
                targetRPY[2] -= dRPY


def torsoTwist():
    biped = Biped()
    CoM_height = 0.45

    targetRPY = [0.0, 0.0, -0.25]
    targetPosL = [0.0, 0.065, -CoM_height]
    targetPosR = [0.0, -0.065, -CoM_height]
    biped.positionInitialize(initializeTime=0.1)

    dp = 0.005
    while True:
        incline = biped.getIncline()
        biped.resetIncline(incline)
        targetRPY[1] = incline

        for _ in range(100):
            biped.setLegPositions(targetPosL, targetPosR, targetRPY)
            biped.oneStep()
            targetRPY[2] += dp

        for _ in range(100):
            biped.setLegPositions(targetPosL, targetPosR, targetRPY)
            biped.oneStep()
            targetRPY[2] -= dp


def walk():
    biped = Biped()
    # CoM_height = 0.45
    # CoM_to_body = np.array([0.0, 0.0, 0.0])

    targetRPY = [0.0, 0.0, 0.0]
    pre = PreviewControl(dt=1./240., Tsup_time=0.3, Tdl_time=0.1, previewStepNum=190)
    biped.positionInitialize(initializeTime=0.2)
    CoM_trajectory = np.empty((0, 3), float)

    trjR_log = np.empty((0, 3), float)
    trjL_log = np.empty((0, 3), float)
    supPoint = np.array([0., 0.065])

    while True:
        incline = biped.getIncline()
        biped.resetIncline(incline)
        targetRPY[1] = incline

        stepHeight = biped.getStepHeight()

        # Generates one cycle trajectory
        CoM_trj, footTrjL, footTrjR = pre.footPrintAndCoM_trajectoryGenerator(inputTargetZMP=supPoint,
                                                                              inputFootPrint=supPoint,
                                                                              stepHeight=stepHeight)
        CoM_trajectory = np.vstack((CoM_trajectory, CoM_trj))
        trjR_log = np.vstack((trjR_log, footTrjR))
        trjL_log = np.vstack((trjL_log, footTrjL))

        for j in range(len(CoM_trj)):
            targetPosR = footTrjR[j] - CoM_trj[j]
            targetPosL = footTrjL[j] - CoM_trj[j]

            biped.setLegPositions(targetPosL, targetPosR, targetRPY)
            biped.oneStep()

        supPoint[0] += biped.getStride()
        supPoint[1] = -supPoint[1]


if __name__ == '__main__':
    jump(withTorsoTwist=True)
