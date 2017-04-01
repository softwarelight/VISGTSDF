function resultI = BilateralFiltGray(I,d,sigma)

[m,n] = size(I);
newI = ReflectEdge(I,d);
resultI = zeros(m,n);
width = 2*d+1;
%Distance
D = fspecial('gaussian',[width,width],sigma(1));
S = zeros(width,width);%pix Similarity
for i=1+d:m+d
    for j=1+d:n+d
        pixValue = newI(i-d:i+d,j-d:j+d);
        subValue = pixValue-newI(i,j);
        S = exp(-subValue.^2/(2*sigma(2)^2));
        H = S.*D;
        resultI(i-d,j-d) = sum(pixValue(:).*H(:))/sum(H(:)); 
    end
    waitbar(i/m);
end
end

function newI = ReflectEdge(I,d)
%Version：1.0——灰色图像  Time：2013/05/01
%Version：1.1——灰色/彩色图像  Time：2013/05/02
%考虑到实用性，决定不添加更多的边界处理选择，统一使用：reflect across edge

if size(I,3)==1
    newI = ReflectEdgeGray(I,d);
elseif size(I,3)==3
    newI = ReflectEdgeColor(I,d);
else 
    error('Incorrect image size')    
end
end

function newI = ReflectEdgeGray(I,d)
[m n] = size(I);
newI = zeros(m+2*d,n+2*d);
%中间部分
newI(d+1:d+m,d+1:d+n) = I;
%上
newI(1:d,d+1:d+n) = I(d:-1:1,:);
%下
newI(end-d:end,d+1:d+n) = I(end:-1:end-d,:);
%左
newI(:,1:d) = newI(:,2*d:-1:d+1);
%右
newI(:,m+d+1:m+2*d) = newI(:,m+d:-1:m+1);
end

function newI = ReflectEdgeColor(I,d)
%扩展图像边界
[m n ~] = size(I);
newI = zeros(m+2*d,n+2*d,3);
%中间部分
newI(d+1:d+m,d+1:d+n,1:3) = I;
%上
newI(1:d,d+1:d+n,1:3) = I(d:-1:1,:,1:3);
%下
newI(end-d:end,d+1:d+n,1:3) = I(end:-1:end-d,:,1:3);
%左
newI(:,1:d,1:3) = newI(:,2*d:-1:d+1,1:3);
%右
newI(:,m+d+1:m+2*d,1:3) = newI(:,m+d:-1:m+1,1:3);
end