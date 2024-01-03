function [u, delta_u, mu, mu_p]  = AIC2(t_now, use_y1_real, y1_real_slow,y1_APO_fast,y2_derivative_sampling,y2_APO_fast, mu_last, mu_p_last, u_last)
e_1=4;
e_2=4;
k_l=1;
sigma_z1_inv=0.2;
sigma_z2_inv=0.2;
sigma_w1_inv=1;
sigma_w2_inv=1;
k_i=-7;
k_p=-4; 
T_c=0.01;
count=mod(t_now,0.05);
trust_array_y1=[1, 0.75, 0.5, 0, 0];%越来越不相信采样时刻的值
trust_array_y2=[0.75, 0.75, 0.75, 0.75, 0.75];%噪声较小时相信求导的数值
%trust_array=[1, 1, 1, 1, 1];%一直都偏向于相信采样时刻的值，不太相信APO的值
if(use_y1_real==0)
    mu = double(mu_last + T_c*(mu_p_last+k_l*sigma_z1_inv*(y1_APO_fast-mu_last)-k_l*sigma_w1_inv*e_1*(mu_p_last+e_1*mu_last)));
    mu_p = double(mu_p_last + T_c*(k_l*sigma_z2_inv*(y2_APO_fast-mu_p_last)-k_l*sigma_w1_inv*(mu_p_last+e_1*mu_last)-k_l*sigma_w2_inv*e_2*e_2*mu_p_last));
    delta_u = T_c*(k_i*(y1_APO_fast-mu)+k_p*(y2_APO_fast-mu_p));
    u = double( u_last - delta_u );
else
    time_inteval = 100*count+1;%表示采样后的第几次控制，有第1次，即采样时刻；第2、3、4、5次
    trust_param_y1 = trust_array_y1(int32(time_inteval));
    trust_param_y2 = trust_array_y2(int32(time_inteval));
    mu = double(mu_last + T_c*(mu_p_last+(1-trust_param_y1)*k_l*sigma_z1_inv*(y1_APO_fast-mu_last)+trust_param_y1*k_l*sigma_z1_inv*(y1_real_slow-mu_last)-k_l*sigma_w1_inv*e_1*(mu_p_last+e_1*mu_last)));
    mu_p = double(mu_p_last + T_c*((1-trust_param_y2)*k_l*sigma_z2_inv*(y2_APO_fast-mu_p_last)+trust_param_y2*k_l*sigma_z2_inv*(y2_derivative_sampling-mu_p_last)-k_l*sigma_w1_inv*(mu_p_last+e_1*mu_last)-k_l*sigma_w2_inv*e_2*e_2*mu_p_last));
    delta_u= T_c*(k_i*((1-trust_param_y1)*(y1_APO_fast-mu)+trust_param_y1*(y1_real_slow-mu))+k_p*((1-trust_param_y2)*(y2_APO_fast-mu_p)+trust_param_y2*(y2_derivative_sampling-mu_p)));
    u = double( u_last - delta_u);
end
end

