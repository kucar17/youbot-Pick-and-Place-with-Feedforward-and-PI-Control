function newConfiguration = NextStatey(currentConfiguration, controls, delta_t, limit)

%% Assigning the properties of the robot:
l = 0.47/2;
w = 0.30/2;
r = 0.0475;

%% Assigning current and new configurations and speeds:
currentJoints = currentConfiguration(4:8)';
currentWheels = currentConfiguration(9:12)';
armJointSpeeds = controls(1:5)';
wheelSpeeds = controls(6:9)';
newJoints = currentJoints + (armJointSpeeds .* delta_t);
newWheels = currentWheels + (wheelSpeeds .* delta_t);
Tsbk = [cos(currentConfiguration(1)), -sin(currentConfiguration(1)), 0, currentConfiguration(2); sin(currentConfiguration(1)), cos(currentConfiguration(1)), 0, currentConfiguration(3); 0, 0, 1, 0.0963; 0, 0, 0, 1];

%% Calculating deltaTheta to initiate the odometry process:
deltaTheta = newWheels - currentWheels;
thetaDot = deltaTheta/delta_t;

Vb = (r/4) * [-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w); 1, 1, 1, 1; -1, 1, -1, 1] * deltaTheta;
Vb6 = [0; 0; Vb; 0];
newChassisRelative = expm(VecTose3(Vb6));
Tsbk1 = Tsbk * newChassisRelative;

phi = acos(Tsbk1(1,1));
x = Tsbk1(1, 4);
y = Tsbk1(2, 4);

chass = [phi x y];

newConfiguration = [chass'; newJoints; newWheels]';

end