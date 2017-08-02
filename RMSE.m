clear;
clc;
close all;

load('ekf_acc_root_10');
ekf_conventional=ekf_acc_root;
load('ekf_fuzzy_10');
ekf_fuzzy=ekf_acc_root;
load('actual_pose');
ground_truth=actual_pose(:,1:749);

error_conventional=(ekf_conventional-ground_truth)./100;
error_fuzzy=(ekf_fuzzy-ground_truth)./100;

% error_conventional=(ekf_conventional)./100;
% error_fuzzy=(ekf_fuzzy)./100;

figure(1);
plot(error_conventional(1,:),'r-','LineWidth',2);hold on;
plot(error_fuzzy(1,:),'b-','LineWidth',2);hold on;
legend('RMS EKF','RMS AFEKF');
xlabel('time(s)');
ylabel('X(m)');

figure(2);
plot(error_conventional(2,:),'r-','LineWidth',2);hold on;
plot(error_fuzzy(2,:),'b-','LineWidth',2);hold on;
legend('RMS EKF','RMS AFEKF');
xlabel('time(s)');
ylabel('Y(m)');

figure(3);
plot(error_conventional(3,:),'r-','LineWidth',2);hold on;
plot(error_fuzzy(3,:),'b-','LineWidth',2);hold on;
legend('RMS EKF','RMS AFEKF');
xlabel('time(s)');
ylabel('Theta(rad)');

% MSE=mean(e(:).^2);
% rmse=sqrt(MSE);