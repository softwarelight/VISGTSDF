function [ tsdf_class_1 ] = getDefaultTSDF(camera_intrinsic,pose)
%���ȶ������ʵ����tsdf_class_1
tsdf_class_1 = TSDF_CLASS();
%��ʼ��
tsdf_class_1.init(pose,camera_intrinsic);
tsdf_class_1.setoffset([0;0;0]);
end

