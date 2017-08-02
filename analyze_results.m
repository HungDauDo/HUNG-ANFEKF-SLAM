clear
clc
%% load results
load('actual_pose');        %actual_pose
load('ekf_deadreckoning');  %deadreckoning
load('ekf_root');           %ekf_acc
load('ekf_acc_root');       %ekf_fuzzy

%% plot results
figure(1);
clf;
axis auto;hold on;

plot(actual_pose(1,:),actual_pose(2,:),'b-');
plot(deadreckoning(1,:),deadreckoning(2,:),'r--');
plot(ekf_acc(1,:),ekf_acc(2,:),'g.-');
plot(ekf_acc_root(1,:),ekf_acc_root(2,:),'k.-');

legend('actual pose','deadreckoning','ekf','ekf fuzzy');
xlabel('X');
ylabel('Y');

%% error comparison
figure(2);
clf;
axis auto; hold on;
% error of x coordinate
plot(deadreckoning(1,:)-actual_pose(1,:),'b.-');
plot(ekf_acc(1,:)-actual_pose(1,:),'r.-');
plot(ekf_acc_root(1,:)-actual_pose(1,:),'g.-');

legend('deadreckoning','ekf','ekf fuzzy');
xlabel('Time(s)');
ylabel('Error of X coordinate');

% error of y coordinate
figure(3);
clf;
axis auto; hold on;
plot(deadreckoning(2,:)-actual_pose(2,:),'b.-');
plot(ekf_acc(2,:)-actual_pose(2,:),'r.-');
plot(ekf_acc_root(2,:)-actual_pose(2,:),'g.-');

legend('deadreckoning','ekf','ekf fuzzy');
xlabel('Time(s)');
ylabel('Error of Y coordinate');

% error of robot orientation
figure(3);
clf;
axis auto; hold on;

plot(deadreckoning(3,:)-actual_pose(3,:),'b-');
plot(ekf_acc(3,:)-actual_pose(3,:),'r-');
plot(ekf_acc_root(3,:)-actual_pose(3,:),'g-');

legend('deadreckoning','ekf','ekf fuzzy');
xlabel('Time(s)');
ylabel('Error of robot orientation(rad)');





