# youbot-Pick-and-Place-with-PI-Control
### Conducted as the Capstone Project of Modern Robotics: Mechanics, Planning, and Control Specialization by Northwestern University, offered on Coursera.

The project aims to make the mobile manipulator youBot pick a cube from an initial configuration and transfer it to a desired location before dropping it off.

The program uses 3 separate functions to achieve this. TrajectoryGenerator first creates a reference trajectory for the end-effector of the youBot. After an initial configuration
is selected, FeedbackControl function calculates the error between the actual and the reference configuration and finds the control vector to be commanded. Then the NextState
function calculates the new configuration for the robot in order to move the robot towards the objective.

<p align="center">
![youBot](http://hades.mech.northwestern.edu/images/thumb/5/57/Youbot-capstone.png/384px-Youbot-capstone.png)
</p>
Controller Information:

The controller used to move the robot is Feedforward PI Controller. The controller is tested with different Kp and Ki gains to see the error response.

Feedforward PI with Kp = 100, Ki = 210
<p align="center">
![Feedforward PID with Kp = 100, Ki = 210.](https://github.com/kucar17/youbot-Pick-and-Place-with-PI-Control/blob/master/Results/newTask/Error%20Plot.png)
</p>
Feedforward PI with Kp = 62, Ki = 19
<p align="center">
![Feedforward PI with Kp = 62, Ki = 19](https://github.com/kucar17/youbot-Pick-and-Place-with-PI-Control/blob/master/Results/best/Error%20Plot.png)
</p>
With the 100 and 210 being Kp and Ki gains, respectively, an example response to a large initial configuration error looks like this:
<p align="center">
![System response when Kp = 100 and Ki = 210](https://github.com/kucar17/youbot-Pick-and-Place-with-PI-Control/blob/master/Results/ehee.gif?raw=true)
</p>
With gains which are not tuned very well, system may behave oscillatory or even unstable. An oscillatory response looks like this:
<p align="center">
![Oscillatory system response](https://github.com/kucar17/youbot-Pick-and-Place-with-PI-Control/blob/master/Results/eheee.gif?raw=true)
</p>
