import numpy as np
import logging


def ur5_inverse_kinematics_with_orientation(x, y, z, orientation):
    # The position is straightforward
    d1 = 0.089159
    a2 = -0.42500
    a3 = -0.39225
    d6 = 0.0823

    P = np.array([x, y, z])

    theta1 = np.arctan2(y, x)

    xc = x - d6 * np.cos(theta1)
    yc = y - d6 * np.sin(theta1)
    zc = z - d1 + d6

    C = np.sqrt(xc**2 + yc**2)
    A = zc
    B = np.sqrt(C**2 + A**2)

    cos_alpha = (a2**2 + B**2 - a3**2) / (2 * a2 * B)
    alpha = np.arccos(np.clip(cos_alpha, -1.0, 1.0))
    beta = np.arctan2(A, C)
    theta2 = np.pi/2 - (alpha + beta)

    cos_gamma = (a2**2 + a3**2 - B**2) / (2 * a2 * a3)
    gamma = np.arccos(np.clip(cos_gamma, -1.0, 1.0))
    theta3 = np.pi - gamma

    # Handle the orientation with the gripper facing down
    Rz = orientation  # Rotation around the Z-axis to make the head face downward
    Rx = np.array([[1, 0, 0], [0, np.cos(Rz), -np.sin(Rz)], [0, np.sin(Rz), np.cos(Rz)]])
    R03 = np.array([[np.cos(theta1)*np.cos(theta2+theta3), -np.cos(theta1)*np.sin(theta2+theta3), np.sin(theta1)],
                    [np.sin(theta1)*np.cos(theta2+theta3), -np.sin(theta1)*np.sin(theta2+theta3), -np.cos(theta1)],
                    [np.sin(theta2+theta3), np.cos(theta2+theta3), 0]])
    R36 = R03.T @ Rx

    theta4 = np.arctan2(R36[1, 2], R36[0, 2])
    theta5 = np.arccos(R36[2, 2])
    theta6 = np.arctan2(-R36[2, 1], R36[2, 0])

    return [theta1, theta2, theta3, theta4, theta5, theta6]

