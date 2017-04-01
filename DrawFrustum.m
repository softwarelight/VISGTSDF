function DrawFrustum(pose,camera_in,depth_max)

R=pose(1:3,1:3);
t=pose(1:3,4);
K=camera_in;
f=K(1,1);

view_frustum_c=[0,-K(1,3),-K(1,3),K(1,3),K(1,3);0,-K(2,3),K(2,3),K(2,3),-K(2,3);0,f,f,f,f];
view_frustum_c = view_frustum_c/K(1,1)*depth_max;
view_frustum_w = R*view_frustum_c+repmat(t,1,5);

h1=patch(view_frustum_w(1,[1 2 3]),view_frustum_w(2,[1 2 3]),view_frustum_w(3,[1 2 3]),'g');
set(h1,'edgecolor','k','facealpha',0.5)
h2=patch(view_frustum_w(1,[1 3 4]),view_frustum_w(2,[1 3 4]),view_frustum_w(3,[1 3 4]),'g');
set(h2,'edgecolor','k','facealpha',0.5)
h3=patch(view_frustum_w(1,[1 4 5]),view_frustum_w(2,[1 4 5]),view_frustum_w(3,[1 4 5]),'g');
set(h3,'edgecolor','k','facealpha',0.5)
h4=patch(view_frustum_w(1,[1 2 5]),view_frustum_w(2,[1 2 5]),view_frustum_w(3,[1 2 5]),'g');
set(h4,'edgecolor','k','facealpha',0.5)
%patch(view_frustum_w(1,[2 3 4 5]),view_frustum_w(2,[2 3 4 5]),view_frustum_w(3,[2 3 4 5]),'g');


end