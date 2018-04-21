## Pick & Place Project
[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png
[image4]: ./misc_images/DHDiagram.PNG
[image5]: ./misc_images/theta1.PNG
[image6]: ./misc_images/spong1.PNG

## Reqirementns of the [rubric](https://review.udacity.com/#!/rubrics/972/view) 
### The point will be addressed one by one. This is the culmination of 65 h of work on the project and a 42 h stop at Khan Academy to redo the linear algebra playlist
 
---
### README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

This is the README

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.


I tried to figure out the instructions. got confused multiple times. I ended up using the values from the video walkthrough. If I had to redo it for sitation where there is no way to use literature I could do it but I am quite late with the project so I wanted to get it submitted as soon as possible.  

This is the table that I created 

Link | Alpha | A | D | Theta
--- | --- | --- | --- | ---
0_1|0| 0| 0.75 | Q1
1_2|PI/2| 0.35| 0| Q2- PI/2
2_3|0| 1.25|0|Q3
3_4|-PI/2|-0.054| 1.50|Q4
4_5|PI/2| 0|0|Q5
5_6|PI/2|  0|   0|Q6
6_EE0|   0| 0.303|0

The illustration from the course material was used

![Ain't it pretty!][image4] 

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

I have created a dictionary. It is elongated but I find it simple to find mistakes in this form

```
               DH_dictionary = { alpha0: 0,
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
```

Transform matrices about each point:

```
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
```

And doing the substitutions we get:

```
                T0_1 = T0_1.subs(DH_dictionary)

                T1_2 = T1_2.subs(DH_dictionary)

                T2_3 = T2_3.subs(DH_dictionary)

                T3_4 = T3_4.subs(DH_dictionary)

                T4_5 = T4_5.subs(DH_dictionary)

                T5_6 = T5_6.subs(DH_dictionary)

                T6_EE = T6_EE.subs(DH_dictionary)
```

They are multiplied to get the global transformation:

```
T0_EE = ((((((T0_1 * T1_2) * T2_3) * T3_4) * T4_5) * T5_6) * T6_EE)
```

Now we have to account for the twist of the gripper:

```
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
```

T0_EE is built with symbols as placeholders. From the EE position and orientation we can get REE that has values and no placeholders.

The transformations for roll, pitch and yaw to accomplish this  are:

```
                R_z = Matrix([[cos(y), -sin(y), 0],
                              [sin(y), cos(y), 0],
                              [0, 0, 1]])

                R_y = Matrix([[cos(p), 0, sin(p)],
                              [0, 1, 0],
                              [-sin(p), 0, cos(p)]])

                R_x = Matrix([[1, 0, 0],
                              [0, cos(r), -sin(r)],
                              [0, sin(r), cos(r)]])
```

Using the EE values we obtain REE:
```
                    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                        [req.poses[x].orientation.x, req.poses[x].orientation.y,
                         req.poses[x].orientation.z, req.poses[x].orientation.w])

                    REE = (R_x * R_y * R_z).evalf(subs={r: roll, p: pitch, y: yaw})
```

The next part is getting the desired information out of these matrices 

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The problem is decoupled into finding theta1-3 for teh are and theta4-6 for the wrist

##### Finding the wrist

Knowing the position and orientation of the EE we can calculate position of the WC. Having the EE to WC distance constant isa big deal simplifier. 
```
                    wrist_x = (px - (d6 + d7) * REE[0, 0]).subs(DH_dictionary)
                    wrist_y = (py - (d6 + d7) * REE[1, 0]).subs(DH_dictionary)
                    wrist_z = (pz - (d6 + d7) * REE[2, 0]).subs(DH_dictionary)
```

With WC position know we can go after theta1-3

##### Finding Theta 1 2 3

Finding theta1 was taken from the lecture, it is simple to derive based on the projection on the ground:

```
theta1 = atan2(wrist_y, wrist_x)
```
The angles theta2 and theta3  are far from straight forward. The first clue I found was from a text book called "Robot Dynamics and Control" which helped me first visual the problem with this chart:

![Theta2 & 3 First Chart][image6]

The forum told me that at this point the A of -0.053 DH parameters comes into play the bellow method is how to deal with it:

```
                    annoying_angle = atan2(wrist_z - 1.94645, wrist_x)
                    wrist_z = wrist_z + 0.054 * cos(annoying_angle)
                    wrist_x = wrist_x - 0.054 * sin(annoying_angle)
```

So, what this does is move the final wrist point according to that extra angle.  So, that means the new wrist point after this code
is set up properly like the problem above and the formulas in Spong's text book worked and my theta 1, 2, and 3 when transformed 
finally pointed to the wrist position at joint 5.

With this done I was able to use the formulas from Spong-RobotmodelingandControl.pdf

##### The Inverse Orientation Kinematics: Theta 1 2 3

To calculate the remaining three thetas, there is a nice formula that gets you almost the way there.  That formula is:
If we have theta1, theta2 and theta3 we can calculate R0_3. 
```python
R3_6 = R0_3.inverse() * R0_6
```

Multiply R0_EE = REE fom the left with the transpose of R0_3 and undoing rotations:
```python
R3_6 = R0_3.transpose()[0:3, 0:3] * REE[0:3, 0:3]
R3_6 = R3_6 * R_z.evalf(subs={y: -pi / 2})[0:3, 0:3] * R_y.evalf(subs={p: -pi / 2})[0:3, 0:3]
```

To maintain sanity the forum recommended the use of euler_from_matrix found in the tf library. this was a godsend:
```python
theta4, theta5, theta6 = tf.transformations.euler_from_matrix(np.array(R3_6).astype(np.float64), "ryzy")

```

Restricting theta5 to a range made the behaviour of the robot much more predictable:
```python
theta5 = np.clip(theta5, -2, 2)
```

All that is lest if packaging and sticking to the end of the list
```python
joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
joint_trajectory_list.append(joint_trajectory_point)
```

I am a slow learner. Once I understand something I am good for 20+ years getting to the point of doing a proper mathematical deduction of the problem would take me weeks of study.
  
### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


The results:

[![Watch it work in super slow motion]](https://youtu.be/VlwgjotCvpM)


Out of 9 runs 8 where succesful. 1 failed bt it was because the message sent by the motion planer was not in proper format:

![End of run][failed1succes8]
##### Summary:

A lot of time sweat and tears went into this. If I was not between jobs I would have not been able to do it.
The code is working and doing it's job. If I had to do this for new robot that has no walkthough video and forum to ask about it I could do it. I really hope I will never have to. Even if I get into robotics professionally (probably in the near future) I will chose a different specialization.
 
