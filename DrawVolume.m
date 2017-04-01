function DrawVolume(range)

%All rights reserved
%Written by zhangjian

x1=range(1,1);
y1=range(2,1);
z1=range(3,1);
x2=range(1,2);
y2=range(2,2);
z2=range(3,2);

x=[x1 x1 x1 x2 x1 x1;x1 x1 x2 x2 x2 x2;x2 x1 x2 x2 x2 x2;x2 x1 x1 x2 x1 x1];
y=[y1 y1 y2 y2 y1 y1;y2 y2 y2 y2 y1 y1;y2 y2 y2 y2 y1 y2;y1 y1 y2 y1 y1 y2];
z=[z1 z1 z1 z1 z1 z2;z1 z1 z1 z1 z1 z2;z1 z2 z2 z2 z2 z2;z1 z2 z2 z2 z2 z2];

for i=1:6
    h=patch(x(:,i),y(:,i),z(:,i),'g');
    set(h,'edgecolor','k','facealpha',0.5)
end

end