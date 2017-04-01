% depth is a matrix
function XYZworld = depth2XYZworld(K, pose, depth)

    [image_row,image_col]  = size(depth);
%%
    depth_temp =zeros (image_row, image_col);
for i=3:image_row-3
      for j=3:image_col-3
% %method 1          
%                 b = depth(i-1:i+1, j-1:j+1);
%                depth(i,j) = sum(sum(b))/8;

%method 2 
                    b=depth(i-1:i+1,j);
                    depth_temp(i,j)=sum(sum(b))/3;
      end
end
depth = depth_temp;

%%

    
    [x,y] = meshgrid(1:image_col, 1:image_row);
    R = pose(1:3,1:3);
    t = pose(1:3,4);

    XYZcam(:,:,1) = (x-K(1,3)).*(depth)/K(1,1);
    XYZcam(:,:,2) = (y-K(2,3)).*depth/K(2,2);
    XYZcam(:,:,3) = depth;
    XYZcam(:,:,4) = depth~=0;

    XYZworld = XYZcam;

    XYZworld(:,:,1) = XYZcam(:,:,1)*R(1,1)+XYZcam(:,:,2)*R(1,2)+XYZcam(:,:,3)*R(1,3)+t(1);
    XYZworld(:,:,2) = XYZcam(:,:,1)*R(2,1)+XYZcam(:,:,2)*R(2,2)+XYZcam(:,:,3)*R(2,3)+t(2);
    XYZworld(:,:,3) = XYZcam(:,:,1)*R(3,1)+XYZcam(:,:,2)*R(3,2)+XYZcam(:,:,3)*R(3,3)+t(3);

    x = XYZworld(:,:,1);
    y = XYZworld(:,:,2);
    z = XYZworld(:,:,3);

    xyzPoints(:,1)=reshape(x,[image_row*image_col,1]);
    xyzPoints(:,2)=reshape(y,[image_row*image_col,1]);
    xyzPoints(:,3)=reshape(z,[image_row*image_col,1]);
    
    %   Delete NaN
    [I,J] = find(isnan(xyzPoints));
    xyzPoints(I,:) = [];
end
