import numpy as np
import scipy.linalg as la


def trajectoryGenerator(startPointVec, endPointVec, startVelocityVec_xy, endVelocityVec,
                        zheight, startTime, endTime, dt):

    x = trajectoryGenerator_xy(startPointVec[0], endPointVec[0], startVelocityVec_xy[0], endVelocityVec[0],
                               startTime, endTime, dt)
    y = trajectoryGenerator_xy(startPointVec[1], endPointVec[1], startVelocityVec_xy[1], endVelocityVec[1],
                               startTime, endTime, dt)
    z = trajectoryGenerator_z(zheight, startPointVec[2], endPointVec[2], endVelocityVec[2],
                              startTime, endTime, dt)

    return np.vstack((x, y, z)).T


def trajectoryGenerator_xy(startPoint, endPoint, startVel, endVel, startTime, endTime, dt):

    timeVec = np.arange(startTime, endTime, dt)

    A = np.matrix([startPoint, endPoint, startVel, endVel]).T

    B = np.matrix([[startTime**3, startTime**2, startTime, 1],
                   [endTime**3, endTime**2, endTime, 1],
                   [3 * (startTime**2), 2 * startTime, 1, 0],
                   [3 * (endTime**2), 2 * endTime, 1, 0]])

    C = la.inv(B) * A

    x = C[0] * (timeVec**3) + C[1] * (timeVec**2) + C[2] * timeVec + C[3]
    return np.array(x)[0]


def trajectoryGenerator_z(zheight, startPoint, endPoint, endVel, startTime, endTime, dt):

    heightTime = ((endTime - startTime) / 2) + startTime
    zh = startPoint + zheight
    timeVec = np.arange(startTime, endTime, dt)

    A = np.matrix([zh, endPoint, startPoint, endVel]).T

    B = np.matrix([[heightTime**3, heightTime**2, heightTime, 1],
                   [endTime**3, endTime**2, endTime, 1],
                   [startTime**3, startTime**2, startTime, 1],
                   [3 * (endTime**2), 2 * endTime, 1, 0]])

    C = la.inv(B) * A

    z = C[0] * (timeVec**3) + C[1] * (timeVec**2) + C[2] * timeVec + C[3]
    return np.array(z)[0]
