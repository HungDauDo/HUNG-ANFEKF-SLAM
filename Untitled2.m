addpath('C:\Users\NB2015-38\CloudStation\semester_4\fuzzy_thesis\Slam_Tested_4Beacons')
load('S_acc');
load('S_noise');
A=S_acc(:,1:766);
B=S_noise(:,1:766);

tf = cellfun('isempty',B) % true for empty cells
B(tf) = {[0 0;0 0]}              % replace by a cell with a zero 

t = cellfun('isempty',A) % true for empty cells
A(t) = {[0 0;0 0]}              % replace by a cell with a zero 

DeltaS = cellfun(@minus,A,B,'Un',0)
save('DeltaS.mat','DeltaS');
