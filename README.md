# Robotics2_JacobianandPT_Group6_Cylindrical-Manipulator_2024

# I. ABSTRACT 
## The Jacobian Matrix is a mathematical tool that acts like a translator between the joint velocities and end-effector velocities of a robot manipulator. The Jacobian Matrix plays a vital role in analyzing the motion of a Cylindrical Manipulator. Cylindrical Manipulator is a robot with 3 degrees of freedom (DOF): one revolute joint for rotation around the axis, and two prismatic joints for linear extension along orthogonal axes.
## The Jacobian Matrix Stucture for a Cylindrical Manipulator will have dimensions of 3x3, reflecting the three DOF of the end-effector (linear motion in X, Y, and Z). The first column will correspond to the revolute joint, influencing the end-effector's rotational velocity around the Z-axis and potentionally affecting its Y position depending on the current condiguration. The remaining two columns will represent the contributions of the prismatic joints to the linear velocities in X and Y directions. When it comes to Singularity Analysis, the Jacobian Matrix can become singular at specific configurations of the manipulator. This indicates a loss of dexterity, where certain end-effector motions can be achieved regardless a joint velocities. Analyzing the Jacobian helps identify these singular configurations and avoid them during robot operation. In Velocity Mapping, the Jacobian allows us to map the desired end-effector velocities (linear and rotational) to the required joint velocities for each joint manipulator. This is crucial for robot control and trajectory planning.

# II. INTRODUCTION 

# III. JACOBIAN MATRIX OF CYLINDRICAL MANIPULATOR 
# IV. DIFFERENTIAL EQUATION OF CYLINDRICAL MANIPULATOR 
# V. PATH AND TRAJECTORY PLANNING CYLINDRICAL MANIPULATOR 
