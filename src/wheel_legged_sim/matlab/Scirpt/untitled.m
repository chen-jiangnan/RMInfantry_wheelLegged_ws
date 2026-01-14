clc;
clear;
%% system variable
% theta 摆杆与竖直方向的夹角        
% phi   机体与水平夹角             
% x     驱动轮位移                
% T     驱动轮输出力矩
% Tp    髋关节输出力矩
% N     驱动轮对摆杆力的水平分量
% P     驱动轮对摆杆力的竖直分量
% Nm    摆杆对机体力水平方向分量
% Pm    摆杆对机体力竖直方向分量
% Nf    地面对驱动轮摩擦力

%% system paramet
% R     驱动轮半径
% L     摆杆重心到驱动轮轴距离
% LM    摆杆重心到其转轴的距离
% l     机体重心到其转轴的距离
% mw    驱动轮转子质量
% mp    摆杆质量
% M     机体质量
% Iw    车轮转动惯量
% Ip    摆杆绕质心的转动惯量
% Im    机体绕质心转动惯量
% L1    大腿长度
% L2    小腿长度
% L3    关节电机轴距

syms theta phi x T Tp N P NM PM Nf
syms theta_dot theta_ddot phi_dot phi_ddot x_dot x_ddot 
syms R L LM l mw mp M Iw Ip IM L1 L2 L3 g mp1 mp2 mp3 mp4

L0 = 0.20;
phi0 = pi/2;

g = 9.80665;
R = 0.135/2;
l = 0.022962;
mw = 0.52417;
mp = 0.9517;
M = 12.1846;
Iw = 0.000600313;
% Ip
IM =M*l^2 + M*(0.2^2+0.1^2)/12;
L1 = 138/1000;
L2 = 262/1000;
L3 = 0.065*2;
Ip = 0.00594;

L = L0/10;
LM = L0*9/10;



%% Newton-Euler
% 对驱动轮，有：
eqn_11 = mw*x_ddot == Nf - N;
eqn_12 = Iw*x_ddot/R == T - Nf*R;

eqn_13 = subs(eqn_12,Nf,solve(eqn_11,Nf));
% 对摆杆，有:
%sin(theta)->theta_dot*cos(theta)->theta_ddot*cos(theta)-theta_dot*theta_dot*sin(theta)
%cos(theta)->-theta_dot*sin(theta)->-theta_ddot*sin(theat)-theta_dot*theta_dot*cos(theta)
eqn_14 = N - NM == mp*(x_ddot+L*(theta_ddot*cos(theta)-theta_dot*theta_dot*sin(theta)));
eqn_15 = P - PM -mp*g == mp*(L*(-theta_ddot*sin(theta)-theta_dot*theta_dot*cos(theta)));
eqn_16 = Ip*theta_ddot == (P*L + PM*LM)*sin(theta)-(N*L+NM*LM)*cos(theta)-T+Tp;

% 对机体，有：
eqn_17 = NM == M*(x_ddot+(L+LM)*(theta_ddot*cos(theta)-theta_dot*theta_dot*sin(theta)) - l*(phi_ddot*cos(phi)-phi_dot*phi_dot*sin(phi)));
eqn_18 = PM-M*g == M*((L+LM)*(  -theta_ddot*sin(theta)-theta_dot*theta_dot*cos(theta)) + l*(-phi_ddot*sin(phi)-phi_dot*phi_dot*cos(phi)));
eqn_19 = IM*phi_ddot == Tp + NM*l*cos(phi) + PM*l*sin(phi);

solve_NM = solve(eqn_17,NM);
solve_PM = solve(eqn_18,PM);
solve_N = solve(subs(eqn_14,NM,solve_NM),N);
solve_P = solve(subs(eqn_15,PM,solve_PM),P);
eqn_23 = subs(eqn_19,[NM,PM],[solve_NM,solve_PM]);
eqn_22 = subs(eqn_16,[NM,PM,N,P],[solve_NM,solve_PM,solve_N,solve_P]);
eqn_21 = subs(eqn_13,N,solve_N);

eqn30 = x_ddot == solve(eqn_21,x_ddot)
eqn31 = theta_ddot == solve(eqn_22,theta_ddot)
eqn32 = phi_ddot == solve(eqn_23,phi_ddot)

[f1,f2,f3] = solve(eqn32,eqn31,eqn30,theta_ddot,x_ddot,phi_ddot);


A = subs(jacobian([theta_dot,f1,x_dot,f2,phi_dot,f3],[theta,theta_dot,x,x_dot,phi,phi_dot]),[theta,theta_dot,x,x_dot,phi,phi_dot,T,Tp],[0,0,0,0,0,0,0,0]);
B = subs(jacobian([theta_dot,f1,x_dot,f2,phi_dot,f3],[T,Tp]),[theta,theta_dot,x,x_dot,phi,phi_dot,T,Tp],[0,0,0,0,0,0,0,0]);

A = double(A)
B = double(B)
C = diag([ 1 1 1 1 1 1]);
D = [
    0 0;
    0 0;
    0 0;
    0 0;
    0 0;
    0 0;
];

Q = diag([1 1 1 800 5000 1]);
R = diag([10 2.5]);

sys = ss(A,B,C,D);
   
LQR_K = lqr(sys,Q,R)
