close all;
cla;
clc;
clear;
% worldSize =5000;
% axis([-worldSize worldSize -100 worldSize]);
figure(1);
load('nR_Sum_10.mat');
plot(nR_Sum-0.7,'b-.','LineWidth',1);hold on;
xlabel('time(s)');
ylabel('Scaling Factor');

figure(2);
load('nR_Sum_30.mat');
plot(nR_Sum-0.7,'r-.','LineWidth',1);hold on;
xlabel('time(s)');
ylabel('Scaling Factor');