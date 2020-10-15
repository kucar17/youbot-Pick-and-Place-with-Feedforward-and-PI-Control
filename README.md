# youbot-Pick-and-Place-with-PI-Control
## Conducted as the Capstone Project of Modern Robotics: Mechanics, Planning, and Control Specialization by Northwestern University, offered on Coursera.

The project aims to make the mobile manipulator youBot pick a cube from an initial configuration and transfer it to a desired location before dropping it off.

The program uses 3 separate functions to achieve this. TrajectoryGenerator first creates a reference trajectory for the end-effector of the youBot. After an initial configuration
is selected, FeedbackControl function calculates the error between the actual and the reference configuration and finds the control vector to be commanded. Then the NextState
function calculates the new configuration for the robot in order to move the robot towards the objective.
