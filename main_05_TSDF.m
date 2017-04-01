clear;
addpath('function2');
addpath('D:\Niuniu\Examnation\TSDF\MyCode\data\depth');
addpath('data\extrinsics');

%��ȡ�ڲξ���
camera_in = importdata('a.txt');

%˫���˲�����
d = 6;
sigma = [3 0.1];

%��ȡ��ξ���
fopen('20130512130736.txt');
fid=fopen('20130512130736.txt');
values = textscan(fid,'%f');
values = values{1};
extrinsicsC2W=permute(reshape(values,4,3,[]),[2 1 3]);


hold on


%���ڣ��������Ŀ���һ�£������������ʵ�ֵĹ��ܣ�������һЩ���Ա�ĳ�ʼ����Ȼ�����TSDF������ƫ��
%���������mu_grid��֪������ָ����ʲô��depth_max�еĵ�λ����ô��ģ�Ϊʲô10�ʹ���3m��
tsdf_global = getDefaultTSDF(camera_in,eye(4,4));
%�����������TSDF��ƫ��
tsdf_global.setoffset([2;0;0]);
%������������
OriginPose=[eye(3,3),[0;0;0]];
DrawCoordinate(OriginPose)
grid on

%ͨ���������֪����Ҫ����TSDF��ͼ������
%num_frame = size(data.depth,2);
sampling_interval = 20;
num_frame = 40;
start_id = 1;



%%    
% merge with global1;
frame_id = start_id;
while(frame_id< num_frame)%�����num_frame����Ҫ�غϵ�֡��
   
    disp(frame_id);
    pose = extrinsicsC2W(:,:,frame_id);
    
    %���ͼ���ݻ�ȡ
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
    tsdf_global.getTSDF(depth,pose); %��Ҫ����Ҫ���������,�����ͼ���������d�Լ�m 
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
