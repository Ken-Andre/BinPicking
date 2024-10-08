import numpy as np

def ur5_inverse_kinematics(x, y, z):
    # Paramètres du robot UR5 (en mètres)
    d1 = 0.089159
    a2 = -0.42500
    a3 = -0.39225
    # d4 = 0.10915
    # d5 = 0.09465
    d6 = 0.0823

    # Position de l'effecteur final
    P = np.array([x, y, z])

    # Calculer theta1
    theta1 = np.arctan2(y, x)

    # Calculer la position du poignet
    xc = x - d6 * np.cos(theta1)
    yc = y - d6 * np.sin(theta1)
    zc = z - d1 + d6

    # Calculer theta2 et theta3
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

    # Calculer theta4, theta5, theta6
    R03 = np.array([
        [np.cos(theta1)*np.cos(theta2+theta3), -np.cos(theta1)*np.sin(theta2+theta3), np.sin(theta1)],
        [np.sin(theta1)*np.cos(theta2+theta3), -np.sin(theta1)*np.sin(theta2+theta3), -np.cos(theta1)],
        [np.sin(theta2+theta3), np.cos(theta2+theta3), 0]
    ])

    R36 = R03.T @ np.eye(3)  # Supposons que l'orientation finale soit l'identité pour simplifier

    theta4 = np.arctan2(R36[1, 2], R36[0, 2])
    theta5 = np.arccos(R36[2, 2])
    theta6 = np.arctan2(-R36[2, 1], R36[2, 0])

    return [theta1, theta2, theta3, theta4, theta5, theta6]



def ur5_inverse_kinematics_with_orientation(x, y, z, rx, ry, rz):
    # Paramètres du robot UR5 (en mètres)
    d1 = 0.089159
    a2 = -0.42500
    a3 = -0.39225
    d4 = 0.10915
    d5 = 0.09465
    d6 = 0.0823

    # Position de l'effecteur final
    P = np.array([x, y, z])

    # Calculer theta1
    theta1 = np.arctan2(y, x)

    # Calculer la position du poignet
    xc = x - d6 * np.cos(theta1)
    yc = y - d6 * np.sin(theta1)
    zc = z - d1 + d6

    # Calculer theta2 et theta3
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
    # ... (Le reste de la fonction reste identique jusqu'au calcul de theta3)

    # Calculer la matrice de rotation désirée à partir des angles d'Euler
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx), np.cos(rx)]])

    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                   [0, 1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])

    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz), np.cos(rz), 0],
                   [0, 0, 1]])

    R_desired = Rz @ Ry @ Rx

    # Calculer R03 (rotation des 3 premiers joints)
    R03 = np.array([
        [np.cos(theta1) * np.cos(theta2 + theta3), -np.cos(theta1) * np.sin(theta2 + theta3), np.sin(theta1)],
        [np.sin(theta1) * np.cos(theta2 + theta3), -np.sin(theta1) * np.sin(theta2 + theta3), -np.cos(theta1)],
        [np.sin(theta2 + theta3), np.cos(theta2 + theta3), 0]
    ])

    # Calculer R36 (rotation des 3 derniers joints)
    R36 = R03.T @ R_desired

    # Extraire theta4, theta5, theta6 de R36
    theta4 = np.arctan2(R36[1, 2], R36[0, 2])
    theta5 = np.arccos(R36[2, 2])
    theta6 = np.arctan2(-R36[2, 1], R36[2, 0])

    return [theta1, theta2, theta3, theta4, theta5, theta6]