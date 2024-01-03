setenv("ROS_DOMAIN_ID","15");
A=[0 1; 0 0];
B=[0;1];
C=[1 0];
desired_poles=4*[-10;-10];
L = acker(A', C', desired_poles)';
%L=[130;4000;30000];
%L=[80;1500;13000];%中间大则超调大  这个还可以
%L=[30;1000;7500];
%L=[30;1000;8500];
%L=[40;1300;10000];
%L=[80;800;12000];%这个很平滑
%L=[80;700;9000];%这个也很平滑

%L=[80;700;9000];%这个也很平滑

A0=expm(A*0.01);
B0=quadv(@(taue)expm(A*taue)*B,0,0.01);
A_bar=zeros(2,2);
B_bar=zeros(2,1);
C_bar=zeros(2,1);
A_hat=A-L*C;
A_bar=expm(A_hat*0.01);
B_bar=quadv(@(taue)expm(A_hat*taue)*B,0,0.01);
C_bar=quadv(@(taue)expm(A_hat*taue)*L,0,0.01);
time = [0:1:(size(A_bar,1)-1)]'; % 假设每行代表一个时间步
L_=timeseries(L, 0);
A_=timeseries(A, 0);
C_=timeseries(C, 0);
A0=timeseries(A0, 0);
B0=timeseries(B0, 0);
A_bar = timeseries(A_bar, 0);
B_bar = timeseries(B_bar, 0);
C_bar = timeseries(C_bar, 0);

