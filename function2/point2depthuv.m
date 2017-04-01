function [depth_u,depth_v,D]=point2depthuv(camera_in,point)
            depth_u = round(camera_in(1,1)*(point(1)./point(3)) + camera_in(1,3));
            depth_v = round(camera_in(2,2)*(point(2)./point(3)) + camera_in(2,3));
            D=(point(1).^2+point(2).^2+point(3).^2).^0.5;
end