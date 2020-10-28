%% This function plots all the points and draws a frame on each point
function DrawPlot(r,Rot,color)

[row,col]=size(r);
if color==0
    plot3(r(1,:),r(2,:),r(3,:),'--o')
end
if color==1
    plot3(r(1,:),r(2,:),r(3,:),'--ok')
end

for i=1:col   
 T = [Rot(:,:,i) r(:,i); 0 0 0 1];
    DrawFrame(T,1);
end