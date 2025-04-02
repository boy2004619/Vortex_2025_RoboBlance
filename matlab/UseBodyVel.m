clear;
clc;
syms theta phi L x x_b N N_f T T_p  M N_M P_M L_M
syms theta_dot x_dot phi_dot theta_ddot x_ddot phi_ddot
syms x_b_dot x_b_ddot

%% 参数设定
% 均为标准单位制
g = 9.81;           %重力加速度

% 驱动轮
mw = 0.58;         %轮子质量
R = 0.075;          %轮子半径
Iw = 0.00823;      %轮子转动惯量

% 大腿
l_active_leg = 0.14;
m_active_leg = 0.174;
% 小腿
l_slave_leg = 0.24;
m_slave_leg = 0.180;
% 关节间距
joint_distance = 0.11;
% 摆杆
mp = (m_active_leg + m_slave_leg)*2 + 0.728;
Ip = mp*L^2/3;   % 摆杆转动惯量

% 机体
M = 12;       %机体重量
IM = 0.124;    %机体惯量,绕质心
l = -0.014;     %机体质心到电机转轴的距离

% QR设置为相同的权重
Q_cost = diag([1,1,500,100,5000,1]);
R_cost = diag([1,0.25]);
useBodyVelocity = 1;

%% 经典力学方程
if useBodyVelocity
    x_ddot = x_b_ddot - (L+L_M)*cos(theta)*theta_ddot+ (L+L_M)*sin(theta)*theta_dot^2;
end
N_M = M*(x_ddot+(L+L_M)*theta_ddot*cos(theta)-(L+L_M)*theta_dot^2*sin(theta)-l*phi_ddot*cos(phi)+l*phi_dot^2*sin(phi));
P_M = M*(g-(L+L_M)*theta_ddot*sin(theta)-(L+L_M)*theta_dot^2*cos(theta)-l*phi_ddot*sin(phi)-l*phi_dot^2*cos(phi));
N = mp*(x_ddot+L*theta_ddot*cos(theta)-L*theta_dot^2*sin(theta))+N_M;
P = mp*(g-L*theta_dot^2*cos(theta)-L*theta_ddot*sin(theta))+P_M;

eqA = x_ddot == (T-N*R)/(Iw/R+mw*R);
eqB = Ip*theta_ddot == (P*L+P_M*L_M)*sin(theta)-(N*L+N_M*L_M)*cos(theta) - T + T_p;
eqC = IM*phi_ddot == T_p + N_M*l*cos(phi) + P_M*l*sin(phi);

%% 计算雅可比矩阵
U = [T T_p].';

if useBodyVelocity
    model_sol = solve([eqA eqB eqC],[theta_ddot,x_b_ddot,phi_ddot]);
    X = [theta,theta_dot,x_b,x_b_dot,phi,phi_dot].';
    dX = [theta_dot,simplify(model_sol.theta_ddot),...
        x_b_dot,simplify(model_sol.x_b_ddot),...
        phi_dot,simplify(model_sol.phi_ddot)].';
    A_sym = subs(jacobian(dX,X),[theta theta_dot x_b_dot phi phi_dot],zeros(1,5));
    B_sym = subs(jacobian(dX,U),[theta theta_dot x_b_dot phi phi_dot],zeros(1,5));
else
    model_sol = solve([eqA eqB eqC],[theta_ddot,x_ddot,phi_ddot]);
    X = [theta,theta_dot,x,x_dot,phi,phi_dot].';
    dX = [theta_dot,simplify(model_sol.theta_ddot),...
        x_dot,simplify(model_sol.x_ddot),...
        phi_dot,simplify(model_sol.phi_ddot)].';
    A_sym = subs(jacobian(dX,X),[theta theta_dot x_dot phi phi_dot],zeros(1,5));
    B_sym = subs(jacobian(dX,U),[theta theta_dot x_dot phi phi_dot],zeros(1,5));
end

%% 计算变长度LQR
L_var = 0.1;   % 腿质心到机体转轴距离,腿长一半的初值

K=zeros(20,12);
leglen=zeros(20,1);

for i=1:20
    L_var=L_var+0.005; % 10mm线性化一次
    leglen(i)=L_var*2;
    trans_A=double(subs(A_sym,[L L_M],[L_var L_var]));
    trans_B=double(subs(B_sym,[L L_M],[L_var L_var]));
    KK=lqrd(trans_A,trans_B,Q_cost,R_cost,0.001);
    KK_t=KK.';
    K(i,:)=KK_t(:);
end

%% 用二项式拟合K,一共12个参数
K_cons=zeros(12,3);

for i=1:12
    res=fit(leglen,K(:,i),"poly2");
    K_cons(i,:)=[res.p1,res.p2,res.p3];
end

for j=1:12
    for i=1:3
        fprintf("%f,",K_cons(j,i));
    end
    fprintf("\n");
end


