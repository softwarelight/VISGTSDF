% test mean value of depth 

depth =depthRead('21.png');
[image_row,image_col]  = size(depth);
image_col=image_col-1;
image_row=image_row-1;
% depth =eye (image_row, image_col);
for i=2:image_col-2
      for j=2:image_row-1
%            b = depth(i-1:i+1, j-1:j+1);
                b = depth(i-1:i+1, j-1:j+1);
                b(2,2)=1;
               depth(i,j) = sum(sum(b))/8;
      end
end
