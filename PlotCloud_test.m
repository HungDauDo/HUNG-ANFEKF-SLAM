load('PClound.mat');
PCloud={};
count=0;
for i=1:16:length(PClound)-15
    count=count+1;
    PCloud={PCloud (PClound(1,i:i+15);PClound(2,i:i+15))};
    plot(cell2mat(PCloud(1,count)),(cell2matPCloud(2,count)),'r.')
end