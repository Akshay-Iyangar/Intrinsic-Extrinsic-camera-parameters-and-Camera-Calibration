

%finding the intrinsic parameter of camera given extrinsic parameter and
%3-D correspondences of images in the camera co-prdinates.Total 28 points
%available so the matirx is over-determined rows>column. Need to solve
%using least mean square '\' in matlab.


%find the matrix A and B  for doing this.

X=zeros(28,3);
Y=zeros(28,2);
y=zeros(28,1);
x=zeros(28,3);
for i=1:28
    
    X(i,:)= [(cam_pts_3D(1,i)/cam_pts_3D(3,i)),(cam_pts_3D(2,i)/cam_pts_3D(3,i)),1];
    Y(i,:)= [(cam_pts_3D(2,i)/cam_pts_3D(3,i)),1];
    y(i,:)=[pts_2D(2,i)];
    x(i,:)= [pts_2D(1,i),pts_2D(2,i),1];
end


P=Y'*Y;
Q=inv(P);
R=Q*Y';
F1=R*y;
F1=F1';


L=X'*X;
M=inv(L);
N=M*X';
F=N*x;
F=F';

%finding the intrinsic and extrinsic parameter given the world co-ordinates
%and the corresponding 2-D points of it.

rubik=imread('rubik_cube.jpg');

world_cordinates=[
[0 3 0 1];[1 3 0 1];[2 3 0 1];[3 3 0 1]
[0 2 0 1];[1 2 0 1];[2 2 0 1];[3 2 0 1]
[0 1 0 1];[1 1 0 1];[2 1 0 1];[3 1 0 1]
[0 0 0 1];[1 0 0 1];[2 0 0 1];[3 0 0 1]
[0 0 -1 1];[1 0 -1 1];[2 0 -1 1];[3 0 -1 1]
[0 0 -2 1];[1 0 -2 1];[2 0 -2 1];[3 0 -2 1]
[0 0 -3 1];[1 0 -3 1];[2 0 -3 1];[3 0 -3 1]
];

cores_2D=[
    [586 129 1];[756 129 1];[938 133 1];[1112 133 1]
    [574 269 1];[750 273 1];[934 273 1];[1112 275 1]
    [568 425 1];[746 429 1];[932 435 1];[1114 437 1]
    [552 583 1];[732 587 1];[930 591 1];[1120 599 1]
    [566 681 1];[738 683 1];[928 689 1];[1110 687 1]
    [580 765 1];[740 769 1];[924 769 1];[1094 775 1]
    [590 837 1];[750 841 1];[922 839 1];[1086 849 1]
    ];
    

P=CalibDLT(cores_2D',world_cordinates');

[ K, Q, t ] = P_to_KRt(P);


%calculating the 2-D co-ordinates given P and (X,Y,Z) of the world
%cordinates.
[m,n]=size(cores_2D);
reproject_2D=zeros(m,2);
E=0;


for i=1:m
    temp=P*world_cordinates(i,:)';
    reproject_2D(i,1)=temp(1)/temp(3);
    reproject_2D(i,2)=temp(2)/temp(3);
    %calculating the reproject error 
    E=E+(cores_2D(i,1)-reproject_2D(i,1))^2 + (cores_2D(i,2)-reproject_2D(i,2))^2;  
end
E=sqrt(E)/m;
%reprojecting the points on to the image.

figure;
subplot(1,2,1);
imshow(rubik);
hold on
plot(reproject_2D(:,1),reproject_2D(:,2),'+','MarkerEdgeColor','y','MarkerSize',10);
title('The re-projected points');
subplot(1,2,2);
imshow(rubik);
hold on
plot(cores_2D(:,1),cores_2D(:,2),'*','MarkerEdgeColor','g','MarkerSize',10);
title('The original points');


%calibration using planar objects.done on calib_gui



