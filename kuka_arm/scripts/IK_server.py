#!/usr/bin/env python

# region import modules
import numpy as np
from time import time
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
import random
from mpmath import *
from sympy import *
import time
import math
# endregion


def handle_calculate_IK(req):

    if len(req.poses) < 1:
        print "Pose message not valid" #This sometimes happens
        return -1
    else:
        # Initialize service response.
        joint_trajectory_list = []


        # Start timing to measure how long calculations took
        # Time is not working for some rason. works in IKdebug
        #start_time = time()
        print "Starting calculations"

        # region Variable name definitions
        T0_1 = 0
        T1_2 = 0
        T2_3 = 0
        T3_4 = 0
        T4_5 = 0
        T5_6 = 0
        T6_EE = 0
        Tcorr = 0
        T0_EE = 0
        R_corr = 0
        REE = 0
        # endregion

        # region Setting up DH Parameters and dictionary
        # Joint angles
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

        # Link offsets
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')

        # Link length
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')

        # Twist angles
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        r = symbols('r')
        p = symbols('p')
        y = symbols('y')

        # Setting up  DH dictionary, ripped right from the lecture
        DH_dictionary = {alpha0: 0,
                         alpha1: -np.pi / 2,
                         alpha2: 0,
                         alpha3: -np.pi / 2,
                         alpha4: np.pi / 2,
                         alpha5: -np.pi / 2,
                         alpha6: 0,
                         a0: 0,
                         a1: 0.35,
                         a2: 1.25,
                         a3: -0.054,
                         a4: 0,
                         a5: 0,
                         a6: 0,
                         d1: 0.75,
                         d2: 0,
                         d3: 0,
                         d4: 1.50,
                         d5: 0,
                         d6: 0,
                         d7: 0.303,
                         q1: q1,
                         q2: q2 - np.pi / 2,
                         q3: q3,
                         q4: q4,
                         q5: q5,
                         q6: q6,
                         q7: 0}
        # endregion

        # region Creating base rotation matrices
        R_z = Matrix([[cos(y), -sin(y), 0],
                      [sin(y), cos(y), 0],
                      [0, 0, 1]])

        R_y = Matrix([[cos(p), 0, sin(p)],
                      [0, 1, 0],
                      [-sin(p), 0, cos(p)]])

        R_x = Matrix([[1, 0, 0],
                      [0, cos(r), -sin(r)],
                      [0, sin(r), cos(r)]])
        # endregion


        # region Setting up matrices T0_1 to T6_EE, taken from the lectures

        T0_1 = Matrix([[cos(q1), -sin(q1), 0, a0],
                       [sin(q1) * cos(alpha0), cos(q1) * cos(alpha0), -sin(alpha0), -sin(alpha0) * d1],
                       [sin(q1) * sin(alpha0), cos(q1) * sin(alpha0), cos(alpha0), cos(alpha0) * d1],
                       [0, 0, 0, 1]])

        T1_2 = Matrix([[cos(q2), -sin(q2), 0, a1],
                       [sin(q2) * cos(alpha1), cos(q2) * cos(alpha1), -sin(alpha1), -sin(alpha1) * d2],
                       [sin(q2) * sin(alpha1), cos(q2) * sin(alpha1), cos(alpha1), cos(alpha1) * d2],
                       [0, 0, 0, 1]])

        T2_3 = Matrix([[cos(q3), -sin(q3), 0, a2],
                       [sin(q3) * cos(alpha2), cos(q3) * cos(alpha2), -sin(alpha2), -sin(alpha2) * d3],
                       [sin(q3) * sin(alpha2), cos(q3) * sin(alpha2), cos(alpha2), cos(alpha2) * d3],
                       [0, 0, 0, 1]])

        T3_4 = Matrix([[cos(q4), -sin(q4), 0, a3],
                       [sin(q4) * cos(alpha3), cos(q4) * cos(alpha3), -sin(alpha3), -sin(alpha3) * d4],
                       [sin(q4) * sin(alpha3), cos(q4) * sin(alpha3), cos(alpha3), cos(alpha3) * d4],
                       [0, 0, 0, 1]])

        T4_5 = Matrix([[cos(q5), -sin(q5), 0, a4],
                       [sin(q5) * cos(alpha4), cos(q5) * cos(alpha4), -sin(alpha4), -sin(alpha4) * d5],
                       [sin(q5) * sin(alpha4), cos(q5) * sin(alpha4), cos(alpha4), cos(alpha4) * d5],
                       [0, 0, 0, 1]])

        T5_6 = Matrix([[cos(q6), -sin(q6), 0, a5],
                       [sin(q6) * cos(alpha5), cos(q6) * cos(alpha5), -sin(alpha5), -sin(alpha5) * d6],
                       [sin(q6) * sin(alpha5), cos(q6) * sin(alpha5), cos(alpha5), cos(alpha5) * d6],
                       [0, 0, 0, 1]])

        T6_EE = Matrix([[cos(q7), -sin(q7), 0, a6],
                        [sin(q7) * cos(alpha6), cos(q7) * cos(alpha6), -sin(alpha6), -sin(alpha6) * d7],
                        [sin(q7) * sin(alpha6), cos(q7) * sin(alpha6), cos(alpha6), cos(alpha6) * d7],
                        [0, 0, 0, 1]])

        T0_1 = T0_1.subs(DH_dictionary)

        T1_2 = T1_2.subs(DH_dictionary)

        T2_3 = T2_3.subs(DH_dictionary)

        T3_4 = T3_4.subs(DH_dictionary)

        T4_5 = T4_5.subs(DH_dictionary)

        T5_6 = T5_6.subs(DH_dictionary)

        T6_EE = T6_EE.subs(DH_dictionary)

        # Every time I want to complain that something is tedious I remember a discussion I had with programmers who where working
        # on some architectural (construction) software. I asked about how is the structural integrity verified.
        # They said it depends on the country but .. in Japan they have a really old software that checks for structural security of the building
        # but .... ALL POINTS are introduced MANUALLY. They have people who sit at a DOS interface all day and introduce
        # (x,y,z) point that that form the basis of the sport stadium models. This is happening in 2018.

        # endregion

        # region Figuring out the T0_EE

        # I am so glad I don't have to do this by hand
        T0_EE = ((((((T0_1 * T1_2) * T2_3) * T3_4) * T4_5) * T5_6) * T6_EE)

        # As far as I understood from the video this is the part that does the correction for difference in orientation between
        # the simulator and URDF file
        R_yy = Matrix([[cos(-np.pi / 2), 0, sin(-np.pi / 2), 0],
                       [0, 1, 0, 0],
                       [-sin(-np.pi / 2), 0, cos(-np.pi / 2), 0],
                       [0, 0, 0, 1]])

        R_zz = Matrix([[cos(np.pi), -sin(np.pi), 0, 0],
                       [sin(np.pi), cos(np.pi), 0, 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])

        R_corr = R_zz * R_yy

        # And the grand transform
        T0_EE = (T0_EE * R_corr)

        # endregion


        for x in xrange(0, len(req.poses)):
            # IK code starts here

            joint_trajectory_point = JointTrajectoryPoint()

            # region Initialize thetas
            theta1 = 0
            theta2 = 0
            theta3 = 0
            theta4 = 0
            theta5 = 0
            theta6 = 0
            # endregion

            # region Figuring out thetas... Here be dragons

            # region EE and WC related math
            # Get end-effector position and orientation from request
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                req.poses[x].orientation.z, req.poses[x].orientation.w])

            # Calculating REE based on end effector data (should be equal to T0_EE)
            REE = (R_x * R_y * R_z).evalf(subs={r:roll, p:pitch, y:yaw})

            # Getting the wrist position based on EE position and orientation
            wrist_x = (px - (d6 + d7) * REE[0, 0]).subs(DH_dictionary)
            wrist_y = (py - (d6 + d7) * REE[1, 0]).subs(DH_dictionary)
            wrist_z = (pz - (d6 + d7) * REE[2, 0]).subs(DH_dictionary)

            # endregion

            # region Actually calculating thetas. Here comes a lot of trigonometry

            # Taken from course but could have figured out on my own
            theta1 = atan2(wrist_y, wrist_x)

            # The next part I had looked up Spong-RobotmodelingandControl.pdf
            # And I got help on the forum for how to handle a shift in the wrist center

            annoying_angle = atan2(wrist_z-1.94645, wrist_x)
            wrist_z = wrist_z+0.054*cos(annoying_angle)
            wrist_x = wrist_x-0.054*sin(annoying_angle)

            # Recalculating distance to the origin
            wrist_xdist = sqrt(wrist_y*wrist_y+wrist_x*wrist_x)

            # Getting sides of triangle used in theta2 an theta3 calculation
            side1 = DH_dictionary[a2]
            side2 = DH_dictionary[d4]

            # Shifting to origin to make math less confusing
            wrist_xdist = wrist_xdist - DH_dictionary[a1]
            wrist_zdist = wrist_z - DH_dictionary[d1]

            # Based on the example on page 21..
            D=(wrist_xdist*wrist_xdist + wrist_zdist*wrist_zdist - side2*side2-side1*side1)/(2*side2*side1)

            if (D>1):
                D=1

            theta3 = atan2(-sqrt(1-D*D),D)

            # From the book and help from the forum
            # I am glad this works. It would be a full time job to understand all the mechanics
            s1=((side1+side2*cos(theta3))*wrist_zdist-side2*sin(theta3)*wrist_xdist) / (wrist_xdist*wrist_xdist + wrist_zdist*wrist_zdist)
            c1=((side1+side2*cos(theta3))*wrist_xdist+side2*sin(theta3)*wrist_zdist) / (wrist_xdist*wrist_xdist + wrist_zdist*wrist_zdist)
            theta2=atan2(s1,c1)

            # And now theta2 and theta3 need to be translated pi/2. As far as I can figure this is again URDF difference
            theta3 = -1*(theta3+pi/2)
            theta2 = pi/2-theta2

            # If we have theta1, theta2 and theta3 we can calculate R0_3
            R0_3 = (T0_1 * T1_2 * T2_3).evalf(subs={q1:theta1,q2:theta2,q3:theta3})[0:3,0:3]

            # Multiply R0_EE = REE fom the left with the transpose of R0_3
            R3_6 = R0_3.transpose()[0:3,0:3] * REE[0:3, 0:3]

            # Do necessary rotations based on forum advice
            R3_6 = R3_6 * R_z.evalf(subs={y: -pi / 2})[0:3, 0:3] * R_y.evalf(subs={p: -pi / 2})[0:3, 0:3]
            theta4, theta5, theta6 = tf.transformations.euler_from_matrix(np.array(R3_6).astype(np.float64), "ryzy")

            # Restrain value in [-2, 2] interval based on forum advice
            theta5 = np.clip(theta5,-2,2)

            # These where a matrix dragons with trigonometry breadth
            # endregion
            # endregion

            # region Adding thetas to joint_trajectory
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)
            # endregion

        print "Calculations done"
        # Display time passed. I can't figure out why this is not passing
        #print ("\nTotal run time to calculate joint angles from poses is %04.4f seconds" % (time() - start_time))

        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to take motion planing values"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
