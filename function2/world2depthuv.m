function [depth_u,depth_v,D]=world2depthuv(camera_ex,camera_in,point_world)
%{
point_world£º


%}


%first process the camera extrinsic matric
camera_extrinsicC2W=[camera_ex;[0 0 0 1]];
camera_extrinsicW2C=inv(camera_extrinsicC2W);

%tranverse the pointXYZ in the world to that in the camera coordinate
point_world1=[point_world;1]; %you should care about the structure of the point_world data
point_camera1=camera_extrinsicW2C*point_world1;

%calculate the [u,v]in the depth image
%location=1/point_camera1(3)*camera_in*camera_extrinsicW2C(1:3,:)*point_world1;
location=camera_in*point_camera1(1:3);
depth_u=location(1)/location(3);
depth_v=location(2)/location(3);
D=(point_camera1(1)^2+point_camera1(2)^2+point_camera1(3)^2)^(0.5);

end