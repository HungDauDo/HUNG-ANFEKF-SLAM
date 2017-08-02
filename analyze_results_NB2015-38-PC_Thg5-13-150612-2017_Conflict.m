clear
clc
%% load results
load('actual_pose');        %actual_pose
load('ekf_deadreckoning');  %deadreckoning
load('ekf_root');           %ekf_acc
load('ekf_fuzzy');          %ekf_fuzzy

%% plot results
figure(1);
clf;
axis auto;hold on;

plot(actual_pose(1,:),actual_pose(2,:),'b-');
plot(deadreckoning(1,:),deadreckoning(2,:),'r--');
plot(ekf_acc(1,:),ekf_acc(2,:),'g--');
plot(ekf_fuzzy(1,:),ekf_fuzzy(2,:),'k--');

legend('actual pose','deadreckoning','ekf','ekf fuzzy');
xlabel('X');
ylabel('Y');
