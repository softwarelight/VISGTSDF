clear;clc
addpath('function')
addpath('data\depth');
addpath('data\extrinsics');

%%read the depth image
dep_image=imread('1.png');
%dep_image=single(dep_image);

%%read the camera intrinsics matrix
camera_in=importdata('a.txt');

%%read the camera extrinsicsC2W matrix
fopen('20130512130736.txt');
fid=fopen('20130512130736.txt');
values = textscan(fid,'%f');
values = values{1};
extrinsicsC2W=permute(reshape(values,4,3,[]),[2 1 3]);

[u,v]=world2depthuv(extrinsicsC2W(:,:,1),camera_in,[10;100;0])