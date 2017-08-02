
clear;
clc;
%init
a=[-1 -0.5 0.05 0.5 1];
%trained
% a=[-0.95 -0.2 0.4 0.65 0.77];
y=ones(size(a));
bar(a,y,0.1);hold on;
xlabel('Output');
ylabel('Membership Degree');