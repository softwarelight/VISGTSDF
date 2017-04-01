clear all
clc

size=200;
origin=[200,0,100];

x=([0 1 1 0 0 0;1 1 0 0 1 1;1 1 0 0 1 1;0 1 1 0 0 0]-0.5)*size+origin(1);
y=([0 0 1 1 0 0;0 1 1 0 0 0;0 1 1 0 1 1;0 0 1 1 1 1]-0.5)*size+origin(2);
z=([0 0 0 0 0 1;0 0 0 0 0 1;1 1 1 1 0 1;1 1 1 1 0 1]-0.5)*size+origin(3);
for i=1:6
    h=patch(x(:,i),y(:,i),z(:,i),'g');
    set(h,'edgecolor','k','facealpha',0.5)
end

h=gcf;

axis equal
axis([0 350 -150 150 0 250])
grid on
view(-33,18)