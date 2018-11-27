## Project: Kinematics Pick & Place
---
**Steps to complete the project:**  

1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: https://github.com/AzizDulaijan/Project-Kinematics-Pick-Place/blob/master/images%20for%20project%202/Capture%20forword%20kin.PNG
[image2]: https://github.com/AzizDulaijan/Project-Kinematics-Pick-Place/blob/master/images%20for%20project%202/theta%201%2C2%2C3.PNG
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README:

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

I first launched the forward_kinematics demo to get familiar with kuku arm, and identify its joints. then after writing the forward kinematics code I compared the end effector position from different angle values between the demo and the code.

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

##### the DH parameter table:
Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | qi
1->2 | -pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | 0
3->4 |  -pi/2 | -0.054 | 1.5 | 0
4->5 | -pi/2 | 0 | 0 | 0
5->6 | -pi/2 | 0 | 0 | 0
6->EE | 0 | 0 | 0.303 | 0

Transform matrix: 
```python
TF_MAT = [[cos(θi)             ,-sin(θi)       ,0                          ,ai-1],
        [ sin(θi)*cos(αi-1), cos(θi)*cos(αi-1), -sin(αi-1), -sin(αi-1)*di],
        [ sin(θi)*sin(αi-1), cos(θi)*sin(αi-1),  cos(αi-1),  cos(αi-1)*di],
        [                   0,                   0,            0,               1]]
``` 
I calculated individual transformation matrices about each joint by substituting the DH parameters values into the Homogeneous transformation matrix between the base and the first link. Then substituting between the first and second and so on. Until the last link to the end effector as can be seen in the code below. 

```python
    # Homogeneous Transforms
    T0_1 = TF_MAT(alpha0,a0,d1,q1).subs(s)
    T1_2 = TF_MAT(alpha1,a1,d2,q2).subs(s)
    T2_3 = TF_MAT(alpha2,a2,d3,q3).subs(s)
    T3_4 = TF_MAT(alpha3,a3,d4,q4).subs(s)
    T4_5 = TF_MAT(alpha4,a4,d5,q5).subs(s)
    T5_6 = TF_MAT(alpha5,a5,d6,q6).subs(s)
    T6_G = TF_MAT(alpha6,a6,d7,q7).subs(s)
    # Create individual transformation matrices
    T0_2 = T0_1 * T1_2 
    T0_3 = T0_2 * T2_3
    T0_4 = T0_3 * T3_4
    T0_5 = T0_4 * T4_5
    T0_6 = T0_5 * T5_6
    T0_G = T0_6 * T6_G
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

#### Inverse Position Kinematics:
When substituting the orientation values of the EE to rotation matrices we can drive:

>> Rrpy = R_z(yaw) * R_y(pitch) * R_x(roll)

and because the UDRF model does not follow the DH convention, the Rrpy need to corrected with rotation about the Z axes , and Y axes.

>> R_corr = (R_z(pi)* R_y(-pi/2))

now the wrist center can be calculated 

#### inverse Orientation Kinematics:
Theta 1 to 3 calculation can be seen from the image below: 

![alt text][image2]

as for theta 4 to 6 , they can be drived in the fallowing steps:
1 - extract Rotation matrixes from the Homogeneous transformation matrix:
```python 
	R0_1 = T0_1[0:3,0:3]
	R0_2 = T0_2[0:3,0:3]
	R0_3 = T0_3[0:3,0:3]
	R0_4 = T0_4[0:3,0:3]
	R0_5 = T0_5[0:3,0:3]
	R0_6 = T0_6[0:3,0:3]
	R0_G = T0_G[0:3,0:3]
```
2- get the rotation matrix R3_6: 

>>Rrpy0_6 = Rrpy

>>Rrpy0_3 = R0_3(sub={q1: theta1, q2: theta2, q3: theta3})

>>R3_6 = inv(Rrpy0_3) * Rrpy0_6

3- calculate the matrix R3_6 angles witch acount for theta4, theta5, and theta6.
```python
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])
theta6 = atan2(-R3_6[1,1],R3_6[1,0])
```

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 
>>> insert image of 9 tubes in


###The Inverse Kinematics code:
```python
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
	    # roll(r), pitch(p), yaw(y) = end-effector orientation
            (r, p, y) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

	    # Compensate for rotation discrepancy between DH parameters and Gazebo

	    # to reduce calculation time R_corr = (R_z(pi)* R_y(-pi/2)) is pre-solved
	    R_corr = Matrix([[0,0,1],
                    	     [0,-1,0],
                             [1,0,0]])
	    # As for the Rrpy = R_z(yaw) * R_y(pitch) * R_x(roll) is also pre-solved
	    Rrpy = Matrix([[cos(y)*cos(p), cos(y)*sin(p)*sin(r)-sin(y)*cos(r), cos(y)*sin(p)*cos(r)+sin(y)*sin(r)],
                            [sin(y)*cos(p), sin(y)*sin(p)*sin(r)+cos(y)*cos(r), sin(y)*sin(p)*cos(r)-cos(y)*sin(r)],
                            [-sin(p),         cos(p)*sin(r),                             cos(p)*cos(r)]])
 	    #to distinguish between R0_6 drived above I used Rrpy0_6 for IK calculations
	    Rrpy = Rrpy * R_corr.T
            Rrpy0_6 = Rrpy
		
	    #Get nx, ny, nz to solve for wrist center
            nx = Rrpy0_6[0,2]
    	    ny = Rrpy0_6[1,2]
    	    nz = Rrpy0_6[2,2]

	    #end effector postion matrix
            EE = Matrix([[px],
                         [py],
                         [pz]])

	    # Calculate joint angles using Geometric IK method
	    #Calculate the wrist center position using end effector position and corrected orientation.
	    d7 = 0.303 #from DH table
	    wc = EE - d7 * Rrpy0_6[:,2]
				
	    a2 = 1.25 #from DH table
	    d4 = 1.504 #from DH table
	    d1 = 0.75 #from DH table
	    
	    #See solution image theta 1 to 3 for valuables naming reasons
	    B1 = wc[2] - d1
	    B2 = sqrt(wc[0]**2+wc[1]**2)- 0.35

	    B = sqrt(B2**2+B1**2) 
	    A = d4 
	    C = a2 
	
	    #Using Cosine Laws to calculate angels a,b, and c
	    a = acos((B**2+C**2 -A**2)/(2*B*C))
	    b = acos((A**2+C**2 -B**2)/(2*A*C))
	    c = acos((A**2+B**2 -C**2)/(2*A*B))

	    #joints 1 to 3 angles:
	    theta1 = atan2(wc[1],wc[0])
	    theta2 = pi/2 - a - (atan2(B1,B2))
	    theta3 = pi/2 - b - atan2(0.0054,1.5)

	    #sub drived joint angles in R0_3 matrix
	    Rrpy0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
    	    R3_6 = Rrpy0_3.T * Rrpy0_6
	
	    #joints 4 to 6 angels
    	    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    	    theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])
    	    theta6 = atan2(-R3_6[1,1],R3_6[1,0])
```

Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]

