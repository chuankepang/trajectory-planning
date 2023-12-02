%% 基本参数
a=24.23/180*pi;
a1=5/180*pi;
a2=10/180*pi;

height1=[0;0;-0.05];
height2=[0;0;-0.05];

r1=[-400.0844264551;-232.8876454473;1493.9464406547]*1e-3;
r2=[-91;-232.89;1493.54]*1e-3;

rc2=[-25.3556513049;626.1864584682;863.8715275375]*1e-3;
rp1=[-1397.1666649192;-230.8573802669;1313.8770645734]*1e-3;
rp2=[-1371.51138;-232.8876454473;987.0064763716]*1e-3;
rp3=[-1399.5666286778;-390.334344736;1404.1018315122]*1e-3;
rp4=[-1380.5666286778;-390.334344736;1090.1018315122]*1e-3;
rp5=[-1398.1666649192;-78.8573802669;1589.8770645734]*1e-3;

%% 各姿态矩阵
A10=[0 -1 0;
     0 0 1;
     -1 0 0];
A10=[1 0 0;
     0 cos(a) sin(a);
     0 -sin(a) cos(a)]*A10;

A20=[0 1 0;
     0 0 1;
     1 0 0];
A20=[1 0 0;
     0 cos(a) sin(a);
     0 -sin(a) cos(a)]*A20;
 
Ap10=[0 -1 0;
      0 0 1;
      -1 0 0];
Ap10=[1 0 0;
     0 cos(a1) sin(a1);
     0 -sin(a1) cos(a1)]*Ap10;

Ap20=[0 -1 0;
      0 0 1;
      -1 0 0];
Ap20=[1 0 0;
     0 cos(a2) sin(a2);
     0 -sin(a2) cos(a2)]*Ap20;
 
Ap30=[0 0 1;
      0 1 0;
      -1 0 0];
Ap30=[sqrt(3)/2 -0.5 0;
      0.5 sqrt(3)/2 0;
      0 0 1]*Ap30;
Ap30=[1 0 0;
      0 cos(a1) sin(a1);
      0 -sin(a1) cos(a1)]*Ap30;
 
Ap40=[0.5 sqrt(3)/2 0;
      -sqrt(3)/2 0.5 0;
      0 0 1]*Ap10;
Ap40=[1 0 0;
      0 cos(a1) sin(a1);
      0 -sin(a1) cos(a1)]*Ap40;
  
Ap50=[0 0 -1;
      0 -1 0;
      -1 0 0];
Ap50=[sqrt(3)/2 -0.5 0;
      0.5 sqrt(3)/2 0;
      0 0 1]*Ap50;
Ap50=[1 0 0;
      0 cos(a1) sin(a1);
      0 -sin(a1) cos(a1)]*Ap50;
  
Ac0=[1 0 0;
     0 0 -1;
     0 1 0];
%% 拼接T矩阵
T01=[A10' r1;
     0 0 0 1];
T02=[A20' r2;
     0 0 0 1];

rc2_above=rc2+Ac0'*height1;
rp1_above=rp1+Ap10'*height2;
rp2_above=rp2+Ap20'*height2;
rp3_above=rp3+Ap30'*height2;
rp4_above=rp4+Ap40'*height2;
rp5_above=rp5+Ap50'*height2;

T0c2=[Ac0' rc2;
      0 0 0 1];
T0p1=[Ap10' rp1;
      0 0 0 1];
T0p2=[Ap20' rp2;
      0 0 0 1];
T0p3=[Ap30' rp3;
      0 0 0 1];
T0p4=[Ap40' rp4;
      0 0 0 1];
T0p5=[Ap50' rp5;
      0 0 0 1];
T0c2_above=[Ac0' rc2_above;
            0 0 0 1];
T0p1_above=[Ap10' rp1_above;
            0 0 0 1];
T0p2_above=[Ap20' rp2_above;
            0 0 0 1];
T0p3_above=[Ap30' rp3_above;
            0 0 0 1];
T0p4_above=[Ap40' rp4_above;
            0 0 0 1];
T0p5_above=[Ap50' rp5_above;
            0 0 0 1];

T1c2=inv(T01)*T0c2;
T1p1=inv(T01)*T0p1;
T1p2=inv(T01)*T0p2;
T1p3=inv(T01)*T0p3;
T1p4=inv(T01)*T0p4;
T1p5=inv(T01)*T0p5;
T1c2_above=inv(T01)*T0c2_above;
T1p1_above=inv(T01)*T0p1_above;
T1p2_above=inv(T01)*T0p2_above;
T1p3_above=inv(T01)*T0p3_above;
T1p4_above=inv(T01)*T0p4_above;
T1p5_above=inv(T01)*T0p5_above;