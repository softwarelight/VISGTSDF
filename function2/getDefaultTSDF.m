function [ tsdf_class_1 ] = getDefaultTSDF(camera_intrinsic,pose)
%首先对类进行实例化tsdf_class_1
tsdf_class_1 = TSDF_CLASS();
%初始化
tsdf_class_1.init(pose,camera_intrinsic);
tsdf_class_1.setoffset([0;0;0]);
end

