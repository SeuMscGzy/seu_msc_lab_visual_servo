%% 1. 定义带测量扰动的连续模型 (LTI对象)
Ts = 0.05;
plantC = ss(0,[-1 1],1,[0 0]);
plantC = setmpcsignals(plantC,'MV',1,'MD',2,'MO',1); %←注意这里是plantC

%% 2. 离散化
plantD = c2d(plantC,Ts,'zoh');

%% 3. 创建MPC对象
P = 20; M = 15;
mpcobj = mpc(plantD,0.05,P,M);

%% 4. 明确指定权重
mpcobj.Weights.ManipulatedVariables = [0.01; 0];
mpcobj.Weights.ManipulatedVariablesRate = [0; 0];
mpcobj.Weights.OutputVariables = 0.8;

%% 5. 约束指定
mpcobj.MV.Min = -4;
mpcobj.MV.Max = 4;
% 
% 
% %% 6. 检查对象
% disp(mpcobj)


