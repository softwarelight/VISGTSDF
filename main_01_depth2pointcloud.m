%现在这个版本是将深度图转换到世界坐标系的点云中
clear;clc
addpath('data\depth')
addpath('data\extrinsics');
addpath('function')

fopen('20130512130736.txt');
fid=fopen('20130512130736.txt');
values = textscan(fid,'%f');
values = values{1};
extrinsicsC2W = permute(reshape(values,4,3,[]),[2 1 3]);
pose1=  extrinsicsC2W(:,:,1);

dep_image1=depthRead('1.png');
dep_image2=single(dep_image1);
camera_in=importdata('a.txt');%读取相机内参

pointcloud=depth2XYZworld(camera_in,pose1,dep_image2);
%pointcloud=depth2XYZcamera(camera_in,dep_image2);
plot3(pointcloud(:,1),pointcloud(:,2),pointcloud(:,3),'.')
grid on
