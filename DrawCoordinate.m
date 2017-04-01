function DrawCoordinate(pose)

%All right reserved 
%Written By ZhangJian
%����quiver3(1,2,3,4,5,6) ���Ե�(1,2,3)Ϊ�����һ��(4,5,6)����,������(1,2,3)һ��ָ��(5,7,9)�ļ�ͷ.

R=pose(1:3,1:3);
t=pose(1:3,4);

OriginPoint=t';
diriction1=(R*[0;0;1])';
diriction2=(R*[0;1;0])';
diriction3=(R*[1;0;0])';

point1=OriginPoint+diriction1;
point2=OriginPoint+diriction2;
point3=OriginPoint+diriction3;

hold on
quiver3(OriginPoint(1),OriginPoint(2),OriginPoint(3),diriction1(1),diriction1(2),diriction1(3),'k','filled','LineWidth',2);
quiver3(OriginPoint(1),OriginPoint(2),OriginPoint(3),diriction2(1),diriction2(2),diriction2(3),'k','filled','LineWidth',2);
quiver3(OriginPoint(1),OriginPoint(2),OriginPoint(3),diriction3(1),diriction3(2),diriction3(3),'k','filled','LineWidth',2);

text(point1(1),point1(2),point1(3),' z ');
text(point2(1),point2(2),point2(3),' y ');
text(point3(1),point3(2),point3(3),' x ');

end