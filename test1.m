    %DH������
    a0=0;       alpha0=0;      d1=0.1807;  
    a1=0;       alpha1=pi/2;   d2=0;       
    a2=0.6126;  alpha2=0;      d3=0;       
    a3=0.5713;  alpha3=0;      d4=0.1741;  
    a4=0;       alpha4=-pi/2;  d5=0.1198;  
    a5=0;       alpha5=pi/2;   d6=0.1165;  
    r=0.22;
    %
%     q=[0;pi/2;pi/2;-pi/2;-pi/2;0];
q=[-1.70595238602419,1.48165558184520,2.20717550865508,-4.80587287524728,-1.43514637869599,1.16730196878365];
    T01=[cos(q(1)) -sin(q(1)) 0 0;
         sin(q(1)) cos(q(1)) 0 0;
         0 0 1 d1;
         0 0 0 1];
    T12=[cos(q(2)) -sin(q(2)) 0 0;
         0 0 -1 0;
         sin(q(2)) cos(q(2)) 0 0;
         0 0 0 1];
    T23=[cos(q(3)) -sin(q(3)) 0 a2;
         sin(q(3)) cos(q(3)) 0 0;
         0 0 1 0;
         0 0 0 1];
    T3t=[cos(q(4)) -sin(q(4)) 0 a3;
         sin(q(4)) cos(q(4)) 0 0;
         0 0 1 0;
         0 0 0 1];
    Tt4=[1 0 0 0;
         0 1 0 0;
         0 0 1 d4;
         0 0 0 1];
    T45=[cos(q(5)) -sin(q(5)) 0 0;
         0 0 1 d5;
         -sin(q(5)) -cos(q(5)) 0 0;
         0 0 0 1];
    T56=[cos(q(6)) -sin(q(6)) 0 0;
         0 0 -1 -d6;
         sin(q(6)) cos(q(6)) 0 0;
         0 0 0 1];
    T6e=[1 0 0 0;
         0 1 0 0;
         0 0 1 r;
         0 0 0 1];
    T02=T01*T12;
    T03=T02*T23;
    T0t=T03*T3t;
    T04=T0t*Tt4;
    T05=T04*T45;
    T06=T05*T56;
    T0e=T06*T6e;
    
    z1=T01(1:3,3);
    z2=T02(1:3,3);
    z3=T03(1:3,3);
    z4=T04(1:3,3);
    z5=T05(1:3,3);
    z6=T06(1:3,3);
    
    re=T06(1:3,4);
    
    p1=[0;0;0];
    p2=T01(1:3,4);
    p3=T03(1:3,4);
    p4=T0t(1:3,4);
    p5=T04(1:3,4);
    p6=T05(1:3,4);

    J1=cross(z1,re-p1);
    J2=cross(z2,re-p2);
    J3=cross(z3,re-p3);
    J4=cross(z4,re-p4);
    J5=cross(z5,re-p5);
    J6=cross(z6,re-p6);
    
    JL=[J1 J2 J3 J4 J5 J6];
    JA=[z1 z2 z3 z4 z5 z6];
    J=[JL;JA];
    J_inverse=inv(J);