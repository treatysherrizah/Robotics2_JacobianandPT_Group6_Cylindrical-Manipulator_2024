# *Robotics2_JacobianandPT_Group6_Cylindrical-Manipulator_2024*
 
## I. ABSTRACT 
 The Jacobian Matrix is a mathematical tool that acts like a translator between the joint velocities and end-effector velocities of a robot manipulator. The Jacobian Matrix plays a vital role in analyzing the motion of a Cylindrical Manipulator. Cylindrical Manipulator is a robot with 3 degrees of freedom (DOF): one revolute joint for rotation around the axis, and two prismatic joints for linear extension along orthogonal axes.The Jacobian Matrix Stucture for a Cylindrical Manipulator will have dimensions of 3x3, reflecting the three DOF of the end-effector (linear motion in X, Y, and Z). The first column will correspond to the revolute joint, influencing the end-effector's rotational velocity around the Z-axis and potentionally affecting its Y position depending on the current condiguration. The remaining two columns will represent the contributions of the prismatic joints to the linear velocities in X and Y directions. When it comes to Singularity Analysis, the Jacobian Matrix can become singular at specific configurations of the manipulator. This indicates a loss of dexterity, where certain end-effector motions can be achieved regardless a joint velocities. Analyzing the Jacobian helps identify these singular configurations and avoid them during robot operation. In Velocity Mapping, the Jacobian allows us to map the desired end-effector velocities (linear and rotational) to the required joint velocities for each joint manipulator. This is crucial for robot control and trajectory planning.

Path and trajectory planning are fundamental tasks in robotics, enabling robots to navigate their environment efficiently and achieve desired goals. This abstract explores these crucial concepts and their significance for robotic motion control. Path and trajectory planning are crucial for various robotic applications like Industrial robots for assembly, welding, and material handling, Autonomous mobile robots for navigation and exploration and urgical robots for precise manipulation in minimally invasive surgery. Path and trajectory planning are cornerstones of robotic motion control, enabling robots to operate effectively and safely in diverse environments. By defining the "what" and "how" of robot movement, these techniques pave the way for the successful execution of various robotic tasks.

## II. INTRODUCTION 
In robotics, precise control over a robot's movements is crucial for successful task completion. Two key concepts play a vital role in achieving this control: the Jacobian Matrix and Path & Trajectory Planning. Cylindrical manipulator, with their unique three-dimensional reach, are widely used in various industrial applications. Precise control over their movements is essential for successful task execution. This introduction explores the Jacobian Matric and its significance in Path and Trajectory Planning for these manipulator.
The Jacobian Matrix for Cylindrical Manipulator plays a vital role in translating joint velocities into end-effector velocities for cylindrical manipulator. It's a 3x3 matrix where the two rows represent the three possible motions of the end-effector (linear X, Y, and Z). The columns represent the velocities of the three joints (theta, X and Y). Each element in the matrix reflects how a specific joint's velocity contributes to a particular component of the end-effector's motion. The Jacobian helps us understand how the rotational movement (theta) of the revolute joint affects Z-axis rotation and potentially Y position (depending on the current arm configuration) and the linear extensions (X and Y) of the prismatic joints influence the end-effector's motion along the corresponding X and Y axes.
Path and Trajectory Planning work together to define the robot's desired movements. The Path Planning focuses on the broad outline of the robot's movement, defining the start and goal positions of the end-effector while avoiding obstacles in the environment. It's like planning the general route on a map. The Trajectory Planning takes the path defined in the previous step and determines the specific timing and movement details for the robot to follow along that path. It's like specifying the speed, acceleration, and precise movements for each joint to navigate the route smoothly. Path planning defines the desired waypoints for the end-effector to reach the goal location, avoiding obstacles. Trajectory planning specifies the detailed motion profile, including the exact path the end-effector follows between waypoints and the timing (speed and acceleration) for each joint movement along the path.
The Jacobian matrix serves as a bridge between the high-level path definition and the low-level joint controls for cylindrical manipulators. By leveraging the Jacobian in path and trajectory planning, we can achieve precise and efficient motion control for these robots, enabling them to perform tasks effectively in various industrial settings.


## III. JACOBIAN MATRIX OF CYLINDRICAL MANIPULATOR 
The Jacobian matrix is a mathematical tool used in various fields, but in robotics, it plays a crucial role in understanding the relationship between the  joint movements and the resulting movement of the end-effector. The Joint Movements represent the rotational speed (for revolute joints) or extension/contraction rate (for prismatic joints) of each joint in the robot's arm. The End-Effector is the part of the robot that interacts with the environment, like the gripper on a robotic arm. The End-Effector Movement describes the linear and angular velocities (or positions) of the end-effector, specifying how fast and in what direction it's moving (or located). The concept of the matrix was the first introduced by the mathematician Carl Gustav Jacob Jacobi in the 19th century

## <p align="center">The Jacobian Matrix Solution of Cylindrical Manipulator
</p>

### Illustration of Cylindrical Manipulator:
<p align="center">
  <img src="https://github.com/treatysherrizah/Robotics2_JacobianandPT_Group6_Cylindrical-Manipulator_2024/assets/157602175/36e50eaf-0e4f-4411-aff1-62b5eb374bd0" width="400"/)
</p>

### To get the Jacobian Matrix of Cylindrical Manipulator, we used this formula: 
<p align="center">
 <img src="https://github.com/treatysherrizah/Robotics2_JacobianandPT_Group6_Cylindrical-Manipulator_2024/assets/157602175/2ff0132f-f014-4cb6-bea1-2f7dc5ee55bd" width="500"/) 
</p> 

<p align="center">
<img src="https://github.com/treatysherrizah/Robotics2_JacobianandPT_Group6_Cylindrical-Manipulator_2024/assets/157602175/0cd95c8f-9b52-4511-bcf9-76198418c038" width="500"/)
</p> 

### Methods in Obtaining the Jacobian Matrix
1. Partial Derivative Method
2. Propagation Method
3. Linear Algebra Method

## <p align="center">Linear Algebra Method
<p/> 

<p align="center">
<img src="https://github.com/treatysherrizah/Robotics2_JacobianandPT_Group6_Cylindrical-Manipulator_2024/assets/157602175/3f7085c2-eacd-4384-97eb-e3b274beec74" width="400"/)
<p/> 

<p align="center">
<img src="https://github.com/treatysherrizah/Robotics2_JacobianandPT_Group6_Cylindrical-Manipulator_2024/assets/157602175/a4262505-febf-43c6-bf56-ac4c3a70342d" width="400"/)
<p/>

## Jacobian Matrix of Cylindrical Manipulator

<p align="center">
<img src="https://github.com/treatysherrizah/Robotics2_JacobianandPT_Group6_Cylindrical-Manipulator_2024/assets/157602175/321f60ed-e712-4fa4-9337-1922a6a5ddd6" width="400"/)
<p/>

<p align="center">
<img src="https://github.com/treatysherrizah/Robotics2_JacobianandPT_Group6_Cylindrical-Manipulator_2024/assets/157602175/bccc6ef8-03e0-41ff-9d51-843cf578fb88" width="400"/)
<p/>

**SUPPLEMENTARY VIDEO DISCUSSION [CLICK HERE: https://youtu.be/VQpwlS1OyQI?si=rU9y5EIOCil4TZjz]**
![White Dark Blue Futuristic Robot Desktop Wallpaper](https://github.com/treatysherrizah/Robotics2_JacobianandPT_Group6_Cylindrical-Manipulator_2024/assets/157670284/7be894b3-2ec2-471b-8e0f-fb06005ec5bd)


## IV. DIFFERENTIAL EQUATION OF CYLINDRICAL MANIPULATOR 
<p align="center">
<img src="https://github.com/treatysherrizah/Robotics2_JacobianandPT_Group6_Cylindrical-Manipulator_2024/assets/157602175/eb3f3860-1be8-4152-8837-f93587502f40" width="400"/)
<p/>

**SUPPLEMENTARY VIDEO DISCUSSION [CLICK HERE: https://youtu.be/Gbovmxfx6m0?si=nQL912VM1id0Oh1X]**
![White Dark Blue Futuristic Robot Desktop Wallpaper](https://github.com/treatysherrizah/Robotics2_JacobianandPT_Group6_Cylindrical-Manipulator_2024/assets/157670284/116de9bf-5939-44e6-abd0-653dcf188b0b)

## V. DETERMINANT OF SINGULARITY
![6f899967-0942-4de1-9e83-f381181ca9ba](https://github.com/treatysherrizah/Robotics2_JacobianandPT_Group6_Cylindrical-Manipulator_2024/assets/157670284/2acdfe72-7125-4f9b-a9c7-1b3666e0fd4f)

**SUPPLEMENTARY VIDEO DISCUSSION [CLICK HERE: https://youtu.be/Y6dr2NqUIXM?si=l0zDvYq6_9vzK8Gn]**
![White Dark Blue Futuristic Robot Desktop Wallpaper (2)](https://github.com/treatysherrizah/Robotics2_JacobianandPT_Group6_Cylindrical-Manipulator_2024/assets/157670284/4dbb11f5-e376-43f9-bf40-b46077a7a80f)

## VI. PATH AND TRAJECTORY PLANNING
### Path and trajectory planning will be used for improving the efficiency, accuracy, and coordination of motion of the cylindrical manipulator to perform tasks effectivey.


<p align="center">
<img src="https://github.com/treatysherrizah/Robotics2_JacobianandPT_Group6_Cylindrical-Manipulator_2024/assets/157602175/d124c2f7-1b44-4c38-b9eb-8186b3f235d6" width="700"/)
<p/>


<p align="center">
<img src="https://github.com/treatysherrizah/Robotics2_JacobianandPT_Group6_Cylindrical-Manipulator_2024/assets/157602175/884c0f0c-6381-4c67-a9bf-37d2e3a1db86" width="700"/)
<p/>

### Path and Trajectory Planning of Cylindrical Manipulator: Welding and Pick & Place

<p align="center">
<img src="https://github.com/treatysherrizah/Robotics2_JacobianandPT_Group6_Cylindrical-Manipulator_2024/assets/157602175/1d21cd37-da64-4152-9948-6a8f9bfba63e" width="600"/)
<p/>

<p align="center">
<img src="https://github.com/treatysherrizah/Robotics2_JacobianandPT_Group6_Cylindrical-Manipulator_2024/assets/157602175/fe5e7441-85c2-4b04-82c8-1b10c5be94f1" width="600"/)
<p/>

**SUPPLEMENTARY VIDEO DISCUSSION [CLICK HERE: ]**
![White Dark Blue Futuristic Robot Desktop Wallpaper (3)](https://github.com/treatysherrizah/Robotics2_JacobianandPT_Group6_Cylindrical-Manipulator_2024/assets/157670284/7d0970b8-0ba3-405a-b2df-0f74e79e5364)

**SUPPLEMENTARY VIDEO DISCUSSION [CLICK HERE: ]**
![White Dark Blue Futuristic Robot Desktop Wallpaper (4)](https://github.com/treatysherrizah/Robotics2_JacobianandPT_Group6_Cylindrical-Manipulator_2024/assets/157670284/e0de73eb-b71f-450a-9ec8-f0298dddc368)



## VII. REFERENCES
1. Gasparetto, A., Boscariol, P., Lanzutti, A., & Vidoni, R. (2015). Path Planning and Trajectory Planning Algorithms: A General Overview. In Mechanisms and machine science (pp. 3–27). https://doi.org/10.1007/978-3-319-14705-5_1
2. 2. Ten popular industrial robot Applications | Jabil. (n.d.). Jabil.com. https://www.jabil.com/blog/ten-popular-industrial-robot-applications.html
3. How to use Jacobian Method for path following? (n.d.). Robotics Stack Exchange. https://robotics.stackexchange.com/questions/22190/how-to-use-jacobian-method-for-path-following
4. 4. Admin. (2019, December 9). Jacobian Matrix and Determinant (Definition and Formula). BYJUS. https://byjus.com/maths/jacobian/#:~:text=Jacobian%20matrix%20is%20a%20matrix,in%20the%20transformation%20of%20coordinates.
