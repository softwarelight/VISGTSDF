clear;
addpath('function2');
addpath('D:\Niuniu\Examnation\TSDF\MyCode\data\depth');
addpath('data\extrinsics');

%读取内参矩阵
camera_in = importdata('a.txt');

%双边滤波参数
d = 6;
sigma = [3 0.1];

%读取外参矩阵
fopen('20130512130736.txt');
fid=fopen('20130512130736.txt');
values = textscan(fid,'%f');
values = values{1};
extrinsicsC2W=permute(reshape(values,4,3,[]),[2 1 3]);


hold on


%现在，我整个的看了一下，这个函数里面实现的功能，首先是一些类成员的初始化，然后就是TSDF整个的偏移
%里面变量：mu_grid不知道具体指的是什么，depth_max中的单位是怎么算的，为什么10就代表3m呢
tsdf_global = getDefaultTSDF(camera_in,eye(4,4));
%这个是设置了TSDF的偏移
tsdf_global.setoffset([2;0;0]);
%画世界坐标轴
OriginPose=[eye(3,3),[0;0;0]];
DrawCoordinate(OriginPose)
grid on

%通过这个可以知道需要进行TSDF的图像数量
%num_frame = size(data.depth,2);
sampling_interval = 20;
num_frame = 40;
start_id = 1;



%%    
% merge with global1;
frame_id = start_id;
while(frame_id< num_frame)%这里的num_frame是需要重合的帧数
   
    disp(frame_id);
    pose = extrinsicsC2W(:,:,frame_id);
    
    %深度图数据获取
    PicNum = int2str(frame_id); 
    PicKind = '.png';
    Picname = strcat(PicNum,PicKind);
%      Picname1 = strcat(PicNum+1,PicKind);
%       Picname2 = strcat(PicNum+2,PicKind);
    depth = depthRead(Picname);
%     depth1= depthRead(Picname1);
%     depth2= depthRead(Picname2);
%     depth=(depth+depth1+depth2)./3;
%     depth = BilateralFiltGray(double(depth), d, sigma);
   
    disp('tsdf ');
    tic;
    tsdf_global.getTSDF(depth,pose); %重要的主要在这个里面,从深度图更新网格的d以及m 
    toc;
    
    frame_id = frame_id+sampling_interval;
    
end


save('tsdf_global');
hold off


%%
%%%%%%%%%%%%%%%%%%%save to ply%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% marching cube
[y,x,z] = meshgrid(1:200,1:200,1:200);
[f1,v1]=MarchingCubes(x,y,z,tsdf_global.tsdf_value,0);

%write_ply(v1,f1,'data/tsdf_current.ply');
write_ply(v1,f1,'data/tsdf.ply');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
