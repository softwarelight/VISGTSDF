classdef TSDF_CLASS < handle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Class: TSDF volume operating class.
%   Method:   **
%   Input:    
%   Returns:  
%             TSDF.func:      TSDF function
%             TSDF.weight:    TSDF weight
%   Author:   Jun Wang.   04/09/2016 
    
    properties
        voxel       = struct();
        tsdf_value  = 0;
        tsdf_weight = 0;
        original_pose_H = 0;
        cameraIntrinsicParam = 0;
        image_properties = [480,640];
        depth_max = 10; % 3m
        mu_grid = 10;%what does it mean?
    end
    methods
        function init(obj,original_pose,camera_intrinsic_param)
            obj.original_pose_H = original_pose;
            obj.cameraIntrinsicParam = camera_intrinsic_param;
            obj.voxel.unit = 0.02;% 
            obj.voxel.size_grid = [200,200,200]';
            %obj.voxel.size_grid = [256,256,256]';
            obj.voxel.mu_grid = obj.mu_grid;
            % x/y/z range
            obj.voxel.range(1,1) = - (obj.voxel.size_grid(1)/2-1) * obj.voxel.unit - ...
                obj.voxel.unit/2;                    
            obj.voxel.range(2,1) = - (obj.voxel.size_grid(2)/2-1) * obj.voxel.unit - ...
                obj.voxel.unit/2;
            obj.voxel.range(3,1) = - (obj.voxel.size_grid(3)/2-1) * obj.voxel.unit - ...
                obj.voxel.unit/2;
            obj.voxel.range(1,2) = obj.voxel.range(1,1) + (obj.voxel.size_grid(1)-1) * obj.voxel.unit;
            obj.voxel.range(2,2) = obj.voxel.range(2,1) + (obj.voxel.size_grid(2)-1) * obj.voxel.unit;
            obj.voxel.range(3,2) = obj.voxel.range(3,1) + (obj.voxel.size_grid(3)-1) * obj.voxel.unit;
            
            R=original_pose(1:3,1:3);
            t=original_pose(1:3,4);
            obj.voxel.rangeW=obj.voxel.range+repmat(t,1,2);
            
            obj.tsdf_value = ones([obj.voxel.size_grid(1),obj.voxel.size_grid(2),obj.voxel.size_grid(3)],'single');
            obj.tsdf_weight = zeros([obj.voxel.size_grid(1),obj.voxel.size_grid(2),obj.voxel.size_grid(3)],'single');
        end
        function setUnit(obj,unit,size_grid)
            obj.voxel.unit = unit;% 
            obj.voxel.size_grid = size_grid;
            %obj.voxel.size_grid = [256,256,256]';
            % x/y/z range
            obj.voxel.range(1,1) = - (obj.voxel.size_grid(1)/2-1) * obj.voxel.unit - ...
                obj.voxel.unit/2;                    
            obj.voxel.range(2,1) = - (obj.voxel.size_grid(2)/2-1) * obj.voxel.unit - ...
                obj.voxel.unit/2;
            obj.voxel.range(3,1) = - (obj.voxel.size_grid(3)/2-1) * obj.voxel.unit - ...
                obj.voxel.unit/2;
            obj.voxel.range(1,2) = obj.voxel.range(1,1) + (obj.voxel.size_grid(1)-1) * obj.voxel.unit;
            obj.voxel.range(2,2) = obj.voxel.range(2,1) + (obj.voxel.size_grid(2)-1) * obj.voxel.unit;
            obj.voxel.range(3,2) = obj.voxel.range(3,1) + (obj.voxel.size_grid(3)-1) * obj.voxel.unit;
            
            obj.tsdf_value = ones([obj.voxel.size_grid(1),obj.voxel.size_grid(2),obj.voxel.size_grid(3)],'single');
            obj.tsdf_weight = zeros([obj.voxel.size_grid(1),obj.voxel.size_grid(2),obj.voxel.size_grid(3)],'single');
        end
        function setoffset(obj,xyz_offset)
            obj.voxel.rangeW= obj.voxel.rangeW+repmat(xyz_offset,[1,2]);
        end
        function getTSDF(obj,depth_image,pose)
            %obj.setData(original_pose);
            %先得到了一个视锥在真实空间中坐标
            [view_frustum_w,view_frustum_c]=obj.getViewFrustumW(pose);
            %将深度图转换为世界坐标系中的坐标XYZcam
            XYZworld = depth2XYZworld(obj.cameraIntrinsicParam, pose, depth_image);
            %计算当前的volume与计算得到视锥重叠的部分~~rangeGrid就是坐标范围，不是真是的坐标
            rangeGrid = obj.getRangeGrid(view_frustum_w,view_frustum_c);
            hold on
            %DrawCoordinate(pose)
            DrawPointCloud(XYZworld);
            %DrawFrustum(pose,obj.cameraIntrinsicParam,10);
            %DrawVolume(obj.voxel.rangeW);
            %将之前的坐标范围，进行mesh，并转化为实际的坐标
            [gridCoordinateW,gridIndex] = obj.getCoordinateGrid(rangeGrid);
            %对之前得到的[gridCoordinateC,gridIndex]进行了三次的筛选，ind是深度图上可靠点的索引
            [ind,gridCoordinateW,gridIndex]=obj.getSelectedGrids(gridCoordinateW,gridIndex,XYZworld,pose);
            %更新最终的距离值
            obj.getDistance(ind,gridCoordinateW,gridIndex,XYZworld,pose);
        end
        function getTSDFPts(obj,points)
            % translate the points to depth image
            depth_image = pointcloud2Depth(points,obj.cameraIntrinsicParam,obj.image_properties);
            view_frustum_c=obj.getViewFrustumC();
            XYZcam = depth2XYZcamera(obj.cameraIntrinsicParam, depth_image);
            rangeGrid = obj.getRangeGrid(view_frustum_c);
            [gridCoordinateC,gridIndex] = obj.getCoordinateGrid(rangeGrid);
            [ind,gridCoordinateC,gridIndex]=obj.getSelectedGrids(gridCoordinateC,gridIndex,XYZcam);
            obj.getDistance(ind,gridCoordinateC,gridIndex,XYZcam);
        end
        function [view_frustum_w,view_frustum_c]=getViewFrustumW(obj,pose)
            % for ViewFrustumC
            K = obj.cameraIntrinsicParam;
            R=pose(1:3,1:3);
            t=pose(1:3,4);
            f = K(1,1);
            view_frustum_c = [0,-K(1,3),-K(1,3),K(1,3),K(1,3);0,-K(2,3),K(2,3),K(2,3),-K(2,3);0,f,f,f,f];
            view_frustum_c = view_frustum_c/K(1,1)* obj.depth_max;
            view_frustum_w = R*view_frustum_c+repmat(t,1,5);
        end
        function [ind,gridCoordinateW,gridIndex]=getSelectedGrids(obj,gridCoordinateW,gridIndex,XYZcam,pose)
            % select: in front of camera
%             isValid = find(gridCoordinateW(3,:)>0);
%             gridCoordinateW = gridCoordinateW(:,isValid);
%             gridIndex = gridIndex(isValid);          
            K = obj.cameraIntrinsicParam;
            
            pose2=inv([pose;0 0 0 1]);
            R_w2c = pose2(1:3,1:3);
            t_w2c = pose2(1:3,4);
            gridCoordinateC=R_w2c*gridCoordinateW+repmat(t_w2c,1,size(gridCoordinateW,2));
            
            % select: project
            px = round(K(1,1)*(gridCoordinateC(1,:)./gridCoordinateC(3,:)) + K(1,3));
            py = round(K(2,2)*(gridCoordinateC(2,:)./gridCoordinateC(3,:)) + K(2,3));
            isValid = (1<=px & px <= obj.image_properties(2) & 1<=py & py<= obj.image_properties(1));
            gridCoordinateW = gridCoordinateW(:,isValid);
            gridIndex = gridIndex(isValid);
            py = py(isValid);
            px = px(isValid);
            
            % select: valid depth
            ind = sub2ind([obj.image_properties(1) obj.image_properties(2)],py,px);
            isValid = XYZcam(ind+obj.image_properties(2)*obj.image_properties(1)*3)~=0;
            gridCoordinateW = gridCoordinateW(:,isValid);
            gridIndex = gridIndex(isValid);
            ind = ind(isValid);
        end
        function [rangeGrid] = getRangeGrid(obj,view_frustum_W,view_frustum_c)
            % choose a bounding box to contain viewing frustum
            % the transformRt here are not needed.
            %ViewFrustumW = transformRT(view_frustum_c,obj.cameraRtC2W);
            %range2test = [min(ViewFrustumW, [], 2) max(ViewFrustumW, [], 2)];
            range2test = [min(view_frustum_W, [], 2) max(view_frustum_W, [], 2)];        %通过这个可以把视锥的长方体框出来
%----------------------------------------------try first-----------------------------------------------------            
%           rangeGrid = (range2test - obj.voxel.rangeW(:,[1 1])) /...
%           obj.voxel.unit + 1;
%           %这句话，我已经完全理解了，其实我们呢，可以简单的在图上画一下就行了，这句话的意思就是以，voxel.rangeW坐标最小的那一点作为起点，然后找到，相对于那一点的
%           %由视锥构成的长方体的相对坐标哈~~，为什么加1？？因为三个点中间距离隔两个
%          
%             rangeGrid(:,1) = max(1,floor(rangeGrid(:,1)));
%             rangeGrid(:,2) = min(ceil (rangeGrid(:,2)),obj.voxel.size_grid);
%-----------------------------------------------try second-------------------------------------------------------
%             range2test = range2test/ obj.voxel.unit;
%             rangeGridt2test = obj.voxel.rangeW/ obj.voxel.unit;
%             rangeGrid(:,1) = max(floor(range2test(:,1)),floor(rangeGridt2test(:,1)));
%             rangeGrid(:,2) = min(ceil(range2test(:,2)),ceil(rangeGridt2test(:,2)));
%-------------------------------------------------------------------------------------------------------  
            rangeGrid = (range2test - obj.voxel.rangeW(:,[1 1])) /obj.voxel.unit + 1;
            rangeGrid(:,1) = max(1,floor(rangeGrid(:,1)));
            rangeGrid(:,2) = min(ceil (rangeGrid(:,2)),obj.voxel.size_grid);
            rangeGrid = int32(rangeGrid);
        end
        function [gridCoordinateC,gridIndex] = getCoordinateGrid(obj,rangeGrid)
            [Y,X,Z]=meshgrid(rangeGrid(2,1):rangeGrid(2,2),rangeGrid(1,1):rangeGrid(1,2),rangeGrid(3,1):rangeGrid(3,2)); % strange matlab syntax
            X = X(:)'; Y = Y(:)'; Z = Z(:)';
            gridIndex = sub2ind(obj.voxel.size_grid',X,Y,Z);
            [gridCoordinateC(1,:),gridCoordinateC(2,:),gridCoordinateC(3,:)]= obj.transGrid2PointCloud(single(X),single(Y),single(Z));
        end
        function getDistance(obj,ind,gridCoordinateW,gridIndex,XYZcam,pose)
            % compare distance between measurement and the grid
            eta = (XYZcam(ind+obj.image_properties(2)*obj.image_properties(1)*2)- gridCoordinateW(3,:)) .* ((1+ (gridCoordinateW(1,:)./gridCoordinateW(3,:)).^2 + (gridCoordinateW(2,:)./gridCoordinateW(3,:)).^2 ).^0.5);
            
            R=pose(1:3,1:3);
            Z_cam=(R*[0;0;1])';
            if Z_cam(3)<0
                eta=-eta;
            end
            
            % select: > - mu
            mu = obj.voxel.mu_grid*obj.voxel.unit;
            isValid = eta>-mu;
            eta = eta(isValid);
            gridIndex = gridIndex(isValid);
            ind = ind(isValid);
            
            
        

            
            new_value = min(1,eta/mu);
            old_weight = obj.tsdf_weight(gridIndex);
            new_weight = old_weight + 2;
            obj.tsdf_weight (gridIndex)= new_weight;
            obj.tsdf_value (gridIndex) = (obj.tsdf_value(gridIndex).*old_weight +new_value)./new_weight;
        end
        function [ x,y,z ] = transGrid2PointCloud(obj,x_grid, y_grid,z_grid)
            %TRANSGRID2POINTCLOUD Summary of this function goes here
            %   Detailed explanation goes here
            x = single((x_grid-1)*obj.voxel.unit+obj.voxel.rangeW(1,1));
            y = single((y_grid-1)*obj.voxel.unit+obj.voxel.rangeW(2,1));
            z = single((z_grid-1)*obj.voxel.unit+obj.voxel.rangeW(3,1));
            %             x = single(x_grid*obj.voxel.unit+obj.voxel.range(1,1));
            %             y = single(z_grid*obj.voxel.unit+obj.voxel.range(2,1));
            %             z = single(y_grid*obj.voxel.unit+obj.voxel.range(3,1));
            %             y = -y;
        end
        function meshIt(obj)
            figure;
            [face,vertex]=isosurface(obj.tsdf_value);
            if size(vertex,1)==0
                disp('no faces extracted');
                return;
            end
            %isosurface 锟角凤拷锟侥ｏ拷要锟斤拷锟斤拷弄锟斤拷锟斤拷锟斤拷
            %%{
            points_grids(:,1)=vertex(:,2);
            points_grids(:,2)=vertex(:,1);% 锟斤拷锟斤拷之前也锟斤拷锟斤拷一锟斤拷锟芥换points_grids(:,1)=verts(:,2);锟斤拷锟斤拷锟街诧拷锟斤拷要锟斤拷
            points_grids(:,3)=vertex(:,3);
            %}
            options.normal = compute_normal(points_grids,face);
            plot_mesh(points_grids,face,options.normal);
            %{
            %lighting gouraud;
            %shading interp;
            lightangle(-45,200);
            lightangle(0,200);
            lightangle(45,200);
            lightangle(90,200);
            lightangle(-90,200);
            %}
            %debug
            % hold on; scatter3(1,200,0,100,[1,0,0]);
            % hold on; scatter3(200,1,0,100,[0,1,0]);
            % hold on; scatter3(0,0,0,100,[0,0,0]);
            xlabel('x');ylabel('y');zlabel('z');
            view([0,0,1]);
        end
        function [points_grids,faces]=extractMesh(obj,filename_mesh)
            [faces,vertex]=isosurface(obj.tsdf_value);
            %isosurface 锟角凤拷锟侥ｏ拷要锟斤拷锟斤拷弄锟斤拷锟斤拷锟斤拷
            points_grids(:,1)=vertex(:,2);
            points_grids(:,2)=vertex(:,1);% 锟斤拷锟斤拷之前也锟斤拷锟斤拷一锟斤拷锟芥换points_grids(:,1)=verts(:,2);锟斤拷锟斤拷锟街诧拷锟斤拷要锟斤拷
            points_grids(:,3)=vertex(:,3);
            write_ply(points_grids,faces,filename_mesh);
        end
    end
end

