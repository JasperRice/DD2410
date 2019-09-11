#! /usr/bin/env python3
import math
import numpy

"""
    # Sifan Jiang
    # sifanj@kth.se
"""

def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]
    """
    Fill in your IK solution here and return the three joint values in q
    """
    l = [0.07, 0.3, 0.35]
    c2 = ((x - l[0])**2 + y**2 - (l[1]**2 + l[2]**2)) / (2 * l[1] * l[2])
    s2 = math.sqrt(1 - c2**2)
    q[0] = math.atan2(y, x - l[0]) - math.atan2(l[2] * s2, l[1] + l[2] * c2)
    q[1] = math.acos(c2)
    q[2] = z
    return q

def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements
    """
    Fill in your IK solution here and return the seven joint values in q
    """
    #+ <---------- Define DH Table ----------> +#
    angle_alpha = numpy.array([math.pi/2, -math.pi/2, -math.pi/2, math.pi/2, math.pi/2, -math.pi/2, 0])
    distance_a = numpy.array([0, 0, 0, 0, 0, 0, 0])
    distance_d = numpy.array([0, 0, 0.4, 0, 0.39, 0, 0])
    # 'angle_theta' is the same with 'q'
    #- <---------- Define DH Table ----------> -#

    #+ <---------- Get desired pose matrix from 'R' and 'point' ----------> +#
    n_d = numpy.array([
        [R[0][0]],
        [R[1][0]],
        [R[2][0]]
    ])
    s_d = numpy.array([
        [R[0][1]],
        [R[1][1]],
        [R[2][1]]
    ])
    a_d = numpy.array([
        [R[0][2]],
        [R[1][2]],
        [R[2][2]]
    ])
    positioin_desired = numpy.array([
        [x],
        [y],
        [z]
    ])
    #- <---------- Get desired pose matrix from 'R' and 'point' ----------> -#

    #+ <---------- Define transform matries ----------> +#
    transform_matrix_base_to_initial = numpy.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0.311],
        [0, 0, 0, 1]
    ])
    transform_matrix_last_to_end_effector = numpy.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0.078],
        [0, 0, 0, 1]
    ])
    transform_matrix_adjacent = numpy.zeros(shape = (7, 4, 4))
    # transform_matrix_initial_to_last = numpy.zeros(shape = (4, 4))
    # transform_matrix_initial_to_end_effector = numpy.zeros(shape = (4, 4))
    # transform_matrix_base_to_end_effector = numpy.zeros(shape = (4, 4))
    #- <---------- Define transform matries ----------> -#

    #+ <---------- Define parameters used in Jacobian ----------> +#
    position_p = numpy.zeros(shape = (7, 3, 1))
    # position_e = numpy.zeros(shape = (3, 1))
    jacobian_p = numpy.zeros(shape = (7, 3, 1))
    jacobian_o = numpy.zeros(shape = (7, 3, 1))
    jacobian_total = numpy.zeros(shape = (6, 7))
    #- <---------- Define parameters used in Jacobian ----------> -#

    #"""
    tolerance = 0.01
    flag = 0
    while flag == 0:
        flag = 1
        transform_matrix_initial_to_last = numpy.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        #+ <---------- Compute the transform matries ----------> +#
        for i in range(7):
            transform_matrix_adjacent[i] = numpy.array([
                [math.cos(q[i]), -math.sin(q[i]) * math.cos(angle_alpha[i]), math.sin(q[i]) * math.sin(angle_alpha[i]), distance_a[i] * math.cos(q[i])],
                [math.sin(q[i]), math.cos(q[i]) * math.cos(angle_alpha[i]), -math.cos(q[i]) * math.sin(angle_alpha[i]), distance_a[i] * math.sin(q[i])],
                [0, math.sin(angle_alpha[i]), math.cos(angle_alpha[i]), distance_d[i]],
                [0, 0, 0, 1]
            ])
            jacobian_o[i] = transform_matrix_initial_to_last[:3, [2]]
            position_p[i] = transform_matrix_initial_to_last[:3, [3]]
            transform_matrix_initial_to_last = numpy.matmul(transform_matrix_initial_to_last, transform_matrix_adjacent[i])
        transform_matrix_initial_to_end_effector = numpy.matmul(transform_matrix_initial_to_last, transform_matrix_last_to_end_effector)
        transform_matrix_base_to_end_effector =  numpy.matmul(transform_matrix_base_to_initial, transform_matrix_initial_to_end_effector)
        n_e = transform_matrix_base_to_end_effector[:3, [0]]
        s_e = transform_matrix_base_to_end_effector[:3, [1]]
        a_e = transform_matrix_base_to_end_effector[:3, [2]]
        #- <---------- Compute the transform matries ----------> -#

        #+ <---------- Compute the Jacobian and pseudo inverse Jacobian ----------> +#
        position_e = transform_matrix_initial_to_end_effector[:3, [3]]
        for i in range(7):
            jacobian_p[i] = numpy.transpose(numpy.cross(numpy.transpose(jacobian_o[i]), numpy.transpose(position_e - position_p[i])))
        for i in range(7):
            for j in range(3):
                jacobian_total[j][i] = jacobian_p[i][j][0]
                jacobian_total[j+3][i] = jacobian_o[i][j][0]
        jacobian_pseudo_inverse = numpy.matmul(numpy.transpose(jacobian_total), (numpy.linalg.inv(numpy.matmul(jacobian_total, numpy.transpose(jacobian_total)))))
        #- <---------- Compute the Jacobian and pseudo inverse Jacobian ----------> -#
        position_current = numpy.array([
            [transform_matrix_base_to_end_effector[0][3]],
            [transform_matrix_base_to_end_effector[1][3]],
            [transform_matrix_base_to_end_effector[2][3]]
        ])
        error_p = position_current - positioin_desired
        error_o = 0.5 * numpy.transpose(numpy.cross(numpy.transpose(n_e), numpy.transpose(n_d)) + numpy.cross(numpy.transpose(s_e), numpy.transpose(s_d)) + numpy.cross(numpy.transpose(a_e), numpy.transpose(a_d)))
        pose_error = numpy.append(error_p, error_o, 0)
        pose_absolute_error = numpy.absolute(pose_error)
        joint_angle_error = numpy.matmul(jacobian_pseudo_inverse, pose_error)
        for i in range(7):
            q[i] = q[i] - joint_angle_error[i][0]
        for i in range(6):
            if pose_absolute_error[i][0] > tolerance:
                flag = 0
                break
        """
        # ---------- Test ---------- #
        print("<----- Operating while loop ----->")
        print("transform_matrix_base_to_end_effector:")
        print(transform_matrix_base_to_end_effector)
        print("Position P")
        print(position_p)
        print("Jacobian P:")
        print(jacobian_p)
        print("Jacobian O:")
        print(jacobian_o)
        print("Jacobian:")
        print(jacobian_total)
        print("Jacobian Pseudo Inverse:")
        print(jacobian_pseudo_inverse)
        print("Desired position:")
        print(positioin_desired)
        print("Current position:")
        print(position_current)
        print("Pose error:")
        print(pose_error)
        print(pose_absolute_error)
        print("Joint angle error")
        print(joint_angle_error)
        print("N")
        print(n_e)
        print(n_d)
        print("S")
        print(s_e)
        print(s_d)
        print("A")
        print(a_e)
        print(a_d)
        #flag = 1
        # ---------- Test ---------- #
        """
    #"""
    return q
