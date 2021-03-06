close all;
cla;
clc;
clear;
worldSize =5000;
axis([-worldSize worldSize -100 worldSize]);
load('nR.mat');
plot(nR,'b--','LineWidth',2);
load('landmarks_v1.mat');

%%%%%%---------------------------------------------%%%%%%
figure(1);
for lIdx=1:length(landmarks)
        plot(landmarks(lIdx,1), landmarks(lIdx,2), 'b*','LineWidth',2);hold on;
end    

load('ekf_ground_truth_4.mat')% ANFIS
ANFIS=ekf_acc_root;
plot(ANFIS(1,:),ANFIS(2,:),'b-','LineWidth',1);hold on;

load('ground_truth_1.mat')% ground truth
ground_truth=ekf_acc_root;
plot(ground_truth(1,:),ground_truth(2,:),'k--','LineWidth',2);hold on;


xlabel('X(mm)');
ylabel('Y(mm)');
%*****************************************************%
figure(2);
load('landmarks_v1.mat');
for lIdx=1:length(landmarks)
        plot(landmarks(lIdx,1), landmarks(lIdx,2), 'b*','LineWidth',2);hold on;
end
load('ground_truth.mat'); % conventional
conventional=ekf_acc_root;
plot(conventional(1,:),conventional(2,:),'g-','LineWidth',1);hold on;
plot(ground_truth(1,:),ground_truth(2,:),'k--','LineWidth',2);hold on;

xlabel('X(mm)');
ylabel('Y(mm)');
%%%%%%%---------------------------------------------%%%%%%

error_conventional=(conventional-ground_truth)./100;
error_fuzzy=(ANFIS-ground_truth)./100;

figure(3);
plot(error_conventional(1,:),'r-','LineWidth',2);hold on;
plot(error_fuzzy(1,:),'b-','LineWidth',2);hold on;
legend('EKF',' AFEKF');
xlabel('time(s)');
ylabel('X(mm)');

figure(4);
plot(error_conventional(2,:),'r-','LineWidth',2);hold on;
plot(error_fuzzy(2,:),'b-','LineWidth',2);hold on;
legend(' EKF',' AFEKF');
xlabel('time(s)');
ylabel('Y(mm)');

% load('error_fuzzy_angle.mat');
figure(5);
plot(error_conventional(3,:),'r-','LineWidth',2);hold on;

plot(error_fuzzy(3,:)','b-','LineWidth',2);hold on;
legend(' EKF',' AFEKF');
xlabel('time(s)');
ylabel('Theta(rad)');

%%%%---------------------------------------------%%
figure(6)
load('nR_sum_122lab(3landmarks).mat');
plot(nR_sum,'b-.','LineWidth',1);hold on;
xlabel('time(s)');
ylabel('Scaling Factor');