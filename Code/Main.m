clc
clear

%% ROBOT PROPERTIES:
tic
Blist = [0 0 1 0 0.033 0; 0 -1 0 -0.5076 0 0; 0 -1 0 -0.3526 0 0; 0 -1 0 -0.2176 0 0; 0 0 1 0 0 0]';
Tb0 = [1, 0, 0, 0.1662; 0, 1, 0, 0; 0, 0, 1, 0.0026; 0, 0, 0, 1];
M0e = [1 0 0 0.033; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1];

l = 0.47/2;
w = 0.30/2;
r = 0.0475;
z = 0.0963;
F = (r/4) * [-1/(l + w), 1/(l + w), 1/(l + w), -1/(l + w); 1 1 1 1; -1 1 -1 1];

sizee = size(F);
m = sizee(2);
zeross = zeros(1, m);
F6 = [zeross; zeross; F; zeross];

%% FEEDFORWARD AND PI CONTROL GAINS:
delta_t = 0.01;

kp = 20;
ki = 17;
Kp = kp*eye(6);
Ki = ki*eye(6);

%% TRAJECTORY PLANNING:
Tse_initial = [0.921060994002885 0 0.389418342308651 0.829087881643154; 0 1 0 0; -0.389418342308651 0 0.921060994002885 0.648543375559790; 0 0 0 1];
Tsc_initial = [1 0 0 1; 0 1 0 0; 0 0 1 0.025; 0 0 0 1];
Tsc_final = [0 1 0 0; -1 0 0 -1; 0 0 1 0.025; 0 0 0 1];
Tce_standoff = [1 0 0 0; 0 1 0 0; 0 0 1 0.2; 0 0 0 1] * [cosd(100), 0, sind(100) 0; 0, 1, 0  0; -sind(100), 0, cosd(100), 0; 0 0 0 1];
Tce_grasp = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [cosd(100), 0, sind(100) 0; 0, 1, 0  0; -sind(100), 0, cosd(100), 0; 0 0 0 1];

movement = TrajectoryGeneratorr(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, 1);
positionSize = size(movement);
positionSize = positionSize(1) - 1;

%% ACTUAL CONFIGURATIONS AND FEEDFORWARD AND PI CONTROL:
% Setting an initial configuration
conf1 = [0.5 0.35 0.6 0 -1.0 0.5 0.1 0 0 0 0 0];
confList(1, :) = conf1(1,:);
error = [];
error2 = [];
errorSize = size(error);
errorSize = errorSize(1);
error2Size = size(error2);
error2Size = error2Size(1);

Tsb = [cos(conf1(1)), -sin(conf1(1)), 0, conf1(2); sin(conf1(1)), cos(conf1(1)), 0, conf1(3); 0, 0, 1, z; 0, 0, 0, 1 ];
T0e = FKinBody(M0e, Blist, conf1(4:8)');
Tse = Tsb * Tb0 * T0e;
X = Tse;
Tse1 = movement(1, :);
Tse11 = [Tse1(1), Tse1(2), Tse1(3), Tse1(10); Tse1(4), Tse1(5), Tse1(6), Tse1(11); Tse1(7), Tse1(8), Tse1(9), Tse1(12); 0, 0, 0, 1];
Xd1 = Tse11;
Xerr_bracket = MatrixLog6(TransInv(X) * Xd1);
Xerr = se3ToVec(Xerr_bracket);

Xerr_integral = Xerr;
for i = 1 : positionSize   
    Tse1 = movement(i, :);
    Tse2 = movement(i+1, :);
    Tse1 = [Tse1(1), Tse1(2), Tse1(3), Tse1(10); Tse1(4), Tse1(5), Tse1(6), Tse1(11); Tse1(7), Tse1(8), Tse1(9), Tse1(12); 0, 0, 0, 1];
    Tse2 = [Tse2(1), Tse2(2), Tse2(3), Tse2(10); Tse2(4), Tse2(5), Tse2(6), Tse2(11); Tse2(7), Tse2(8), Tse2(9), Tse2(12); 0, 0, 0, 1];
    
    Xd1 = Tse1;
    Xd2 = Tse2;
    
    [Vd, V, Je, controls, Xerr, Xerr_integral] = FeedbackControl(X, Xd1, Xd2, Kp, Ki, delta_t, conf1(i, 4:8), Xerr_integral);
    error(errorSize+1, :) = Xerr';
    errorSize = size(error);
    errorSize = errorSize(1);
    
    error2(error2Size+1, :) = Xerr_integral';
    error2Size = size(error2);
    error2Size = error2Size(1);    
    
    conf1(i+1,:) = NextStatey(conf1(i,:), [controls(5:9)', controls(1:4)'], delta_t, 10000);
    confList(i+1, :) = conf1(i+1, :);
    
    phi = conf1(i+1, 1);
    x = conf1(i+1, 2);
    y = conf1(i+1, 3);
    Tsb = [cos(conf1(i+1, 1)), -sin(conf1(i+1, 1)), 0, conf1(i+1, 2); sin(conf1(i+1, 1)), cos(conf1(i+1, 1)), 0, conf1(i+1, 3); 0, 0, 1, z; 0, 0, 0, 1 ];
    T0e = FKinBody(M0e, Blist, conf1(i+1, 4:8)');
    X = Tsb*Tb0*T0e;
    Xd1 = [];
    Xd2 = [];    
end

for i = 1 : positionSize + 1
    confList(i,13) = movement(i, 13);
end

disp('Generating the animation file.')
writematrix(confList, 'newTask.csv')
disp('Plotting the Error Graph')
plot(error2)
legend('wbx', 'wby', 'wbz', 'vbx', 'vby', 'vbz')
title('Error Twists (Kp = 20, Ki = 17)')
xlabel('Time (0.01 seconds)')
ylabel('Error')


toc