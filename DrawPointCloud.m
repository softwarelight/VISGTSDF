function DrawPointCloud(XYZworld)

%All rights reserved 
%Written by zhangjian

    x = XYZworld(:,:,1);
    y = XYZworld(:,:,2);
    z = XYZworld(:,:,3);
    image_row=480;
    image_col=640;
    xyzPoints(:,1)=reshape(x,[image_row*image_col,1]);
    xyzPoints(:,2)=reshape(y,[image_row*image_col,1]);
    xyzPoints(:,3)=reshape(z,[image_row*image_col,1]);
    
    plot3(xyzPoints(:,1),xyzPoints(:,2),xyzPoints(:,3),'.')
    grid on
    
end