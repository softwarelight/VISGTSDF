clear;clc
addpath('function')
addpath('data\depth');
addpath('data\extrinsics');


%read the depth image & camera intrinsics matrix
depth1 = depthRead('1.png');
depth2 = depthRead('2.png');
depth3 = depthRead('3.png');
camera_in = importdata('a.txt');

%%read the camera extrinsicsC2W matrix
fopen('20130512130736.txt');
fid=fopen('20130512130736.txt');
values = textscan(fid,'%f');
values = values{1};
extrinsicsC2W=permute(reshape(values,4,3,[]),[2 1 3]);


%initialise the tsdf
for x_count=0:1:200
    for y_count=0:1:200
        for z_count=0:1:200
            x=x_count-100;
            y=y_count-100;
            z=z_count;
            count=x_count+201*y_count+201^2*z+1;%here is explained in the paper of Kintinuous
            
            %%in the tsdf(count,:),it stores the xyz position and sdf      
            tsdf(count,1)=x*0.1;
            tsdf(count,2)=y*0.1;
            tsdf(count,3)=z*0.1;
            
            %calculate the sdf
            [u,v,D]=point2depthuv(camera_in,[x;y;z]);
            if u<=480&&u>=0&&v>=0&&v<=640    
                tsdf(count,4)=norm([x;y;z])-D;
            else   
                tsdf(count,4)=1;  
            end
            surface(x_count+1,y_count+1,z_count+1)=tsdf(count,4);
            %norm(cross([x,y,z]-[0,0,0],pointcloud(count1,:)))/norm([x,y,z]-[0,0,0])
            
            tsdf_grid{count}=[x,y,z,0,0];%tsdf_grid{count}=[x,y,z,tsdf_distance,tsdf_weight]
        end
    end
end

%plot the tsdf
[x,y,z] = meshgrid(1:201,1:201,1:201);
[f1,v1]=MarchingCubes(x,y,z,surface,0);
write_ply(v1,f1,'data/tsdf.ply');



