% clear all;
%% 设定指令位置、姿态、时间
%r第一列是初始位置，不要随便修改，后续列为指令位置，可任意拓展列数
r1=[-0.691100000000000;-0.174100000000000;0.481800000000000];
r2=[-0.809074103915500;-0.728357588068283;-0.0831340782625809];
r3=[-0.859074103915500;-0.728357588068283;-0.0831340782625809];
r4=[-0.809074103915500;-0.728357588068283;-0.0831340782625809];
r5=[-0.150256310815141;0.474664273999064;0.826259012017169];
r6=[-0.154030265180400;0.497093161942422;0.870786556908767];
r=[r1 r2 r3 r4 r5 r6];
%注意是A06而不是A60，A06(:,:,1)是初始姿态，不要随便修改，A06要与r一一对应
A06(:,:,1)=[0 -1 0;
            -1 0 0;
            0 0 -1];
A06(:,:,2)=[0 0 -1;
            -0.410400562604415 -0.911905355952020 0;
            -0.911905355952020 0.410400562604415 0];
A06(:,:,3)=[0 0 -1;
            -0.410400562604415 -0.911905355952020 0;
            -0.911905355952020 0.410400562604415 0];
A06(:,:,4)=[0 0 -1;
            -0.410400562604415 -0.911905355952020 0;
            -0.911905355952020 0.410400562604415 0];
A06(:,:,5)=[-0.500000000000000 0.862729915662821 -0.0754790873051733;
            -0.789733204101540 -0.418448874522590 0.448577758867171;
            0.355417312942849 0.283897220893760 0.890550897831946];
A06(:,:,6)=[-0.500000000000000 0.862729915662821 -0.0754790873051733;
            -0.789733204101540 -0.418448874522590 0.448577758867171;
            0.355417312942849 0.283897220893760 0.890550897831946];
%指令时间t要与r一一对应
t=[0 10 15 20 40 45];
step_size=0.01;
%%
trajectory(1,:)=t(1):step_size:t(length(t));
[~,command_number]=size(r);
trajectory_number=command_number-1;
flag=zeros(1,command_number);
flag(1)=1;
for ii=1:trajectory_number
    flag(ii+1)=flag(ii)+(t(ii+1)-t(ii))/step_size;
end
for ii=1:trajectory_number
    delta_t=t(ii+1)-t(ii);
    delta_r=r(:,ii+1)-r(:,ii);
    k3=10.*delta_r./(delta_t.^3);
    k4=-15.*delta_r./(delta_t.^4);
    k5=6.*delta_r./(delta_t.^5);
    delta_A=A06(:,:,ii+1)'*A06(:,:,ii);
    T=trace(delta_A);
    if(T==3)
        b3=[0;0;0];
        b4=[0;0;0];
        b5=[0;0;0];
    elseif(T==-1)
        sigma=pi;
        a1=sqrt((1+delta_A(1,1))/2);
        a2=sign(delta_A(1,2))*sqrt((1+delta_A(2,2))/2);
        a3=sign(delta_A(3,1))*sqrt((1+delta_A(3,3))/2);
        a=A06(:,:,ii+1)*[a1;a2;a3];
        delta_theta=sigma*a;
        b3=10.*delta_theta./(delta_t.^3);
        b4=-15.*delta_theta./(delta_t.^4);
        b5=6.*delta_theta./(delta_t.^5);
    else
        sigma=acos((T-1)/2);
        sin_sigma=sin(sigma);
        a1=(delta_A(2,3)-delta_A(3,2))/2/sin_sigma;
        a2=(delta_A(3,1)-delta_A(1,3))/2/sin_sigma;
        a3=(delta_A(1,2)-delta_A(2,1))/2/sin_sigma;
        a=A06(:,:,ii+1)*[a1;a2;a3];
        delta_theta=sigma*a;
        b3=10.*delta_theta./(delta_t.^3);
        b4=-15.*delta_theta./(delta_t.^4);
        b5=6.*delta_theta./(delta_t.^5);
    end
    
    time_sequence=0:step_size:delta_t;
    trajectory(2,flag(ii):flag(ii+1))=3.*k3(1).*time_sequence.^2+4.*k4(1).*time_sequence.^3+5.*k5(1).*time_sequence.^4;
    trajectory(3,flag(ii):flag(ii+1))=3.*k3(2).*time_sequence.^2+4.*k4(2).*time_sequence.^3+5.*k5(2).*time_sequence.^4;
    trajectory(4,flag(ii):flag(ii+1))=3.*k3(3).*time_sequence.^2+4.*k4(3).*time_sequence.^3+5.*k5(3).*time_sequence.^4;
    trajectory(5,flag(ii):flag(ii+1))=3.*b3(1).*time_sequence.^2+4.*b4(1).*time_sequence.^3+5.*b5(1).*time_sequence.^4;
    trajectory(6,flag(ii):flag(ii+1))=3.*b3(2).*time_sequence.^2+4.*b4(2).*time_sequence.^3+5.*b5(2).*time_sequence.^4;
    trajectory(7,flag(ii):flag(ii+1))=3.*b3(3).*time_sequence.^2+4.*b4(3).*time_sequence.^3+5.*b5(3).*time_sequence.^4;
end
save('trajectory.mat','trajectory');