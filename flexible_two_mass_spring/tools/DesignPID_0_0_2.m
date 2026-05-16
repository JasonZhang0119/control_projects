function [Kp, Ki, Kd] = DesignPID_0_0_2(k, theta, tau_1, tau_2, tau_c)
%DESIGNPID_0_0_1  SIMC 规则整定 PID 参数
%
% 本函数依据 Skogestad/Grimholt 在 SIMC（Simple IMC）方法中的整定思想，
% 针对带纯滞后（dead time）的对象，给出一组 PID 参数 (Kp, Ki, Kd)。
%
% 该实现支持三类对象：
%   1) tau_1 ~= 0 & tau_2 == 0：一阶惯性 + 纯滞后（FOPDT）, 使用改进方法，对一阶系统加入Kd
%   2) tau_1 ~= 0 & tau_2 ~= 0：二阶惯性 + 纯滞后（SOPDT），其中 tau_2 作为微分时间常数 tau_D
%   3）tau_1 == 0 & tau_2 == 0: 积分 + 纯滞后系统
% 
% ---------
% Creator      : Jiaxuan Zhang
% Created      : 2025-12-24
% Version      : 0.0.2
% Last version : 0.0.1
% Last updated : 2025-12-24
% Change: 加入了针对（积分+纯时滞）系统计算PID参数的方法
%
% Parameters
% ----------
%   G(s) = k * exp(-theta*s) / ((tau_1*s + 1) * (tau_2*s + 1))
% 
% k : 
%     对象静态增益（process gain）。
% theta : 
%     纯滞后时间（dead time），单位与 tau_1/tau_2/tau_c 一致。
% tau_1 : 
%     一阶惯性（主）时间常数（dominant time constant）。
% tau_2 : 
%     二阶惯性（次）时间常数；当 tau_2 == 0 时视为一阶对象。
% tau_c : 
%     闭环期望时间常数（tuning knob）。tau_c 越小通常越激进，越大越保守。
%
% Returns
% -------
%   u = Kp * e + Ki * ∫ e dt + Kd * de/dt
%
% Kp : 
%     PID 比例增益。
% Ki : 
%     PID 积分增益（并联形式）
% Kd : 
%     PID 微分增益（并联形式）。
%
% References
% ----------
% [1] Skogestad, S., & Grimholt, C. (2012).
%     "The SIMC method for smooth PID controller tuning"
%     PID Control in the Third Millennium: Lessons Learned and New Approaches,
%     pp. 147–175.
% [2] C. Grimholt and S. Skogestad, 
%     "Optimal PI and PID control of first-order plus delay processes and evaluation of the original and improved SIMC rules," 
%     Journal of Process Control, 
%     vol. 70, pp. 36–46, Oct. 2018, doi: 10.1016/j.jprocont.2018.06.011.


%% integral system case
if tau_1 == 0 && tau_2 == 0

    Kc = 1 / k / (tau_c + theta);
    tau_I = 4 * (tau_c + theta);
    tau_D = 0;
    f = 1 + tau_D / tau_I;

    Kp = Kc * f;
    Ki = (Kc * f)/(tau_I * f);
    Kd = 0;

%% first order system case
elseif tau_2 ==0
    
    Kc = 1/k * tau_1 / (tau_c + theta);
    tau_I = min([tau_1, 4 * (tau_c + theta)]);
    tau_D = theta / 3;
    
    f = 1 + tau_D / tau_I;
    Kp = Kc * f;
    Ki = (Kc * f)/(tau_I * f);
    Kd = (Kc * f) * tau_D / f;

%% second order system case
else

    Kc = 1/k * tau_1 / (tau_c + theta);
    tau_I = min([tau_1 + theta/3, 4 * (tau_c + theta)]);
    tau_D = tau_2;

    Kp = Kc/tau_I * (tau_I + tau_D);
    Ki = Kc/tau_I;
    Kd = Kc/tau_I * tau_I * tau_D;

end

end

