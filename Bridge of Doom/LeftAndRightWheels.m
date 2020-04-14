clc, clear all

syms u t

assume(0 <= u <= 3.2)
d = 0.235; % wheel distance
beta = .32; % speed multiplier
steps = 100; % number of simulation steps

u = beta * t;

ri=4*(0.3960*cos(2.65*(u+1.4)));
rj=-4*(0.99*sin(u+1.4));
rk=0*u;

r=[ri,rj,rk];

dr=diff(r,t) %unit tangent
V = norm(dr); %linear speed

T_hat_ugly=dr./norm(dr);
T_hat=simplify(T_hat_ugly);

dT_hat = diff(T_hat,t);

omega=simplify(cross(T_hat,dT_hat));

VL = simplify(V - (omega(3) * (d / 2)));
VR = simplify(V + (omega(3) * (d / 2)));

t_num = linspace(0,3.2/beta,steps);

load('beta_34');
exp_left = [];
exp_right = [];
exp_linspeed = [];
exp_angspeed = [];
for i = 1:101
    if (1 <= i) && (i <= 100)
        slope_left = (dataset(i,2) - dataset(i+1,2)) / (dataset(i,1) - dataset(i+1,1));
        exp_left = [exp_left slope_left];
        slope_right = (dataset(i,3) - dataset(i+1,3)) / (dataset(i,1) - dataset(i+1,1));
        exp_right = [exp_right slope_right];
%         linspeed = (slope_left + slope_right) / 2;
%         exp_linspeed = [exp_linspeed linspeed];
%         angspeed = (slope_right - slope_left) / d;
%         exp_angspeed = [exp_angspeed angspeed];
    else
    end
end

exp_left = transpose(exp_left);
exp_right = transpose(exp_right);
% exp_linspeed = transpose(exp_linspeed);
% exp_angspeed = transpose(exp_angspeed);

for n=1:length(t_num)
    r_num(n,:)=double(subs(r,[t],[t_num(n)]));
    VL_num(n,:)=double(subs(VL,[t],[t_num(n)]));
    VR_num(n,:)=double(subs(VR,[t],[t_num(n)]));
    V_num(n,:)=double(subs(V,[t],[t_num(n)]));
    w_num(n,:)=double(subs(omega,[t],[t_num(n)]));
end

figure(1);
plot(t_num, VR_num,'red', t_num, VL_num,'blue', t_num+1.1, exp_right, '--r', t_num+1.1, exp_left, '--b'); hold on
legend("Right", "Left", "Right (Theoretical)", "Left (Theoretical)"); title("Neato Wheel Velocity over Time")
xlabel("Time (s)"); ylabel("Wheel Velocity (m/s)")
hold off;

% figure(2);
% plot(t_num, V_num,'red', t_num, w_num, 'blue', t_num+1.1, exp_linspeed, '--r', t_num+1.1, exp_angspeed, '--b'); hold on
% legend("Linear", "Angular", "Linear (Theoretical)", "Angular (Theoretical)"); title("Neato Velocity over Time")
% xlabel("Time (s)"); ylabel("Velocity (m/s)")
% hold off;

figure(2);
plot(t_num, V_num,'red', t_num, w_num, 'blue'); hold on
legend("Linear", "Angular")
xlabel("Time (s)"); ylabel("Velocity (m/s)")
hold off;