%Now,I want to use some depth images to realize tsdf
%the thought above I don't realize
%here I only realize a 

addpath('function')

clear;clc
[x,y,z] = meshgrid(1:200,1:200,1:200);
sphere=zeros(200,200,200)-1;
for zz=50:1:150
    r=(50^2-(zz-100).^2).^(1/2);
    r1=round(r);
    for yy=100-r1:1:100+r1
        rr=(r1.^2-(yy-100).^2).^(1/2);
        rr1=round(rr);
        for xx=100-rr1:1:100+rr1
            sphere(xx,yy,zz)=1;
        end
    end
end
[f1,v1]=MarchingCubes(x,y,z,sphere,0);