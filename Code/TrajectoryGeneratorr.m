function movement = TrajectoryGeneratorr(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k)

Tse_standoff = Tsc_initial * Tce_standoff;
Tse_grasp = Tsc_initial * Tce_grasp;
Tce_standoff_2 = [1 0 0 0; 0 1 0 0; 0 0 1 0.1; 0 0 0 1] * [cosd(100), 0, sind(100) 0; 0, 1, 0  0; -sind(100), 0, cosd(100), 0; 0 0 0 1];
Tse_standoff_2 = Tsc_final * Tce_standoff_2;
Tse_final = Tsc_final * [cosd(100), 0, sind(100) 0; 0, 1, 0  0; -sind(100), 0, cosd(100), 0; 0 0 0 1];

trajectory1 = ScrewTrajectory(Tse_initial, Tse_standoff, 10, 10/0.01, 3);
trajectory2 = ScrewTrajectory(Tse_standoff, Tse_grasp, 5, 5/0.01, 3);
trajectory3 = ScrewTrajectory(Tse_grasp, Tse_standoff, 5, 5/0.01, 3);
trajectory4 = ScrewTrajectory(Tse_standoff, Tse_standoff_2, 10, 10/0.01, 3);
trajectory5 = ScrewTrajectory(Tse_standoff_2, Tse_final, 5, 5/0.01, 3);
trajectory6 = ScrewTrajectory(Tse_final, Tse_standoff_2, 5, 5/0.01, 3);

for i = 1 : length(trajectory1)
    traj1(i,:) = [trajectory1{i}(1,1) trajectory1{i}(1,2) trajectory1{i}(1,3) trajectory1{i}(2,1) trajectory1{i}(2,2) trajectory1{i}(2,3) trajectory1{i}(3,1) trajectory1{i}(3,2) trajectory1{i}(3,3) trajectory1{i}(1, 4) trajectory1{i}(2, 4) trajectory1{i}(3, 4)];
end

for i = 1 : length(trajectory2)
    traj2(i,:) = [trajectory2{i}(1,1) trajectory2{i}(1,2) trajectory2{i}(1,3) trajectory2{i}(2,1) trajectory2{i}(2,2) trajectory2{i}(2,3) trajectory2{i}(3,1) trajectory2{i}(3,2) trajectory2{i}(3,3) trajectory2{i}(1, 4) trajectory2{i}(2, 4) trajectory2{i}(3, 4)];
end

for i = 1 : length(trajectory3)
    traj3(i,:) = [trajectory3{i}(1,1) trajectory3{i}(1,2) trajectory3{i}(1,3) trajectory3{i}(2,1) trajectory3{i}(2,2) trajectory3{i}(2,3) trajectory3{i}(3,1) trajectory3{i}(3,2) trajectory3{i}(3,3) trajectory3{i}(1, 4) trajectory3{i}(2, 4) trajectory3{i}(3, 4)];
end

for i = 1 : length(trajectory4)
    traj4(i,:) = [trajectory4{i}(1,1) trajectory4{i}(1,2) trajectory4{i}(1,3) trajectory4{i}(2,1) trajectory4{i}(2,2) trajectory4{i}(2,3) trajectory4{i}(3,1) trajectory4{i}(3,2) trajectory4{i}(3,3) trajectory4{i}(1, 4) trajectory4{i}(2, 4) trajectory4{i}(3, 4)];
end

for i = 1 : length(trajectory5)
    traj5(i,:) = [trajectory5{i}(1,1) trajectory5{i}(1,2) trajectory5{i}(1,3) trajectory5{i}(2,1) trajectory5{i}(2,2) trajectory5{i}(2,3) trajectory5{i}(3,1) trajectory5{i}(3,2) trajectory5{i}(3,3) trajectory5{i}(1, 4) trajectory5{i}(2, 4) trajectory5{i}(3, 4)];
end

for i = 1 : length(trajectory6)
    traj6(i,:) = [trajectory6{i}(1,1) trajectory6{i}(1,2) trajectory6{i}(1,3) trajectory6{i}(2,1) trajectory6{i}(2,2) trajectory6{i}(2,3) trajectory6{i}(3,1) trajectory6{i}(3,2) trajectory6{i}(3,3) trajectory6{i}(1, 4) trajectory6{i}(2, 4) trajectory6{i}(3, 4)];
end

traj1(:, 13) = 0;
traj2(1:200, 13) = 0;
traj2(201:300, 13) = 0;
traj3(:, 13) = 1;
traj4(:, 13) = 1;
traj5(:, 13) = 1;
traj6(1:10, 13) = 1;
traj6(11:300, 13) = 0;

movement = [traj1; traj2; traj3; traj4; traj5; traj6];
