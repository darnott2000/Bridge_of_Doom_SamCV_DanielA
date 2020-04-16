clc, clear all

syms u t

assume(0 <= u <= 3.2)
d = 0.235; % wheel distance
beta = .32; % speed multiplier
steps = 100; % number of simulation steps

%relationship for u to t
u = beta * t;

%build the parametric
ri=4*(0.3960*cos(2.65*(u+1.4)));
rj=-4*(0.99*sin(u+1.4));
rk=0*u;
r_n=[ri,rj,rk];

dr=diff(r_n,t); %unit tangent
V = norm(dr); %linear speed

T_hat_ugly=dr./norm(dr);
T_hat=simplify(T_hat_ugly); %unit tangent

dT_hat = diff(T_hat,t);
N_hat=dT_hat/norm(dT_hat);
N_hat=simplify(N_hat);

omega=simplify(cross(T_hat,dT_hat)); %angular speed

%pre-defined formulas for VL and VR
VL = simplify(V - (omega(3) * (d / 2)));
VR = simplify(V + (omega(3) * (d / 2)));

t_num = linspace(0,3.2/beta,steps); %create even steps in terms of t

%Here we will load the experimental data received from the robot completing
%the bridge. We will then compute the slope between adjacent points to find
%the left and right wheel velocities which then allows us to find
%experimental linear ad angular speed over time.
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
        linspeed = (slope_left + slope_right) / 2;
        exp_linspeed = [exp_linspeed linspeed];
        angspeed = (slope_right - slope_left) / d;
        exp_angspeed = [exp_angspeed angspeed];
    else
    end
end
exp_left = transpose(exp_left);
exp_right = transpose(exp_right);
exp_linspeed = transpose(exp_linspeed);
exp_angspeed = transpose(exp_angspeed);

%substitute the sym values
for n=1:length(t_num)
    r_num(n,:)=double(subs(r_n,[t],[t_num(n)]));
    T_hat_num(n,:)=double(subs(T_hat,t,t_num(n)));
    N_hat_num(n,:)=double(subs(N_hat,t,t_num(n)));
    VL_num(n,:)=double(subs(VL,[t],[t_num(n)]));
    VR_num(n,:)=double(subs(VR,[t],[t_num(n)]));
    V_num(n,:)=double(subs(V,[t],[t_num(n)]));
    w_num(n,:)=double(subs(omega,[t],[t_num(n)]));
end


time = dataset(1:100,1);
dt = diff(time);

%reconstruct the path
r_n = zeros(length(time),2);
theta = zeros(length(time),1);

for n=1:length(dt)
    r_n(n+1,1)=r_n(n,1)+ exp_linspeed(n)*cos(theta(n))*dt(n);
    r_n(n+1,2)=r_n(n,2)+ exp_linspeed(n)*sin(theta(n))*dt(n);
    theta(n+1) = theta(n) + exp_angspeed(n)*dt(n);
end

figure(4)
%plotting theoretical path
hold on
plot3(r_num(:,1),r_num(:,2),r_num(:,3)), axis equal % plot the entire curve
for n = 1:100
    if mod(n,10) == 1
        quiver(r_num(n,1),r_num(n,2),T_hat_num(n,1),T_hat_num(n,2),'r') % plot the unit tangent
        quiver(r_num(n,1),r_num(n,2),N_hat_num(n,1),N_hat_num(n,2),'b') % plot the unit normal
    else
        end
end

%plotting experimental path
plot(r_n(:,1),r_n(:,2),'--m') % plot measured
quiver(r_n(1:20:length(r_n),1),r_n(1:20:length(r_n),2),cos(theta(1:20:length(r_n))),sin(theta(1:20:length(r_n))),'--y'); % plot linear velocity of measured

axis equal
legend('Predicted Robot Position','Predicted Unit Tangent', 'Predicted Unit Normal','Measured Robot Position','Measured Linear Velocity');
xlabel('x position [m]')
ylabel('y position [m]')
title('Predicted Position/Vectors vs. Measured Position/Vectors')
hold off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(1)
hold on
plot3(r_num(:,1),r_num(:,2),r_num(:,3),'c'), axis equal, hold on % plot the entire curve
title('Parametric Curve for Centerline of the Bridge')
xlabel('x');
ylabel('y');
for n = 1:100
    if mod(n,10) == 1
        quiver(r_num(n,1),r_num(n,2),T_hat_num(n,1),T_hat_num(n,2),'r') % plot the unit tangent
        quiver(r_num(n,1),r_num(n,2),N_hat_num(n,1),N_hat_num(n,2),'b') % plot the unit normal
    else
        end
end
hold off

figure(2);
plot(t_num, VR_num,'red', t_num, VL_num,'blue', t_num+1.1, exp_right, '--r', t_num+1.1, exp_left, '--b'); hold on
legend("Right", "Left", "Right (Theoretical)", "Left (Theoretical)"); title("Neato Wheel Velocity over Time")
xlabel("Time (s)"); ylabel("Wheel Velocity (m/s)")
hold off;

figure(3);
plot(t_num, V_num,'red', t_num, w_num, 'blue', t_num+1.1, exp_linspeed, '--r', t_num+1.1, exp_angspeed, '--b'); hold on
legend("Linear", "Angular", "Linear (Theoretical)", "Angular (Theoretical)"); title("Neato Velocity over Time")
xlabel("Time (s)"); ylabel("Velocity (m/s)")
hold off;