%Using the Matlab Symbolic Toolbox to solve for properties of parametric
%curves
clc, clear all

%For this demo code, we will solve for the parametric circle that is
%presented in Robo Night 1

%First we must define the symbolic variables we will being using
syms u

%Define the equation for the parametric circle. For clarity, I am first 
%creating individual equations for r in the i, j, and k direction, 
%then packing them in one vector. In this case, we are
%creating a 1x3 vector that has symbolic equations for the i, j, and k
%components of the equation, respectively. Note that the k component in
%this case is just equal to 0. We use 0*u because if we substitute a vector
%of numerical values in for the symbolic u at a later time, the dimensions
%of the vector will be consistent.
ri=4*(0.3960*cos(2.65*(u+1.4)));
rj=-4*(0.99*sin(u+1.4));
rk=0*u;
r=[ri,rj,rk];

%to take the derivative, we use the diff function in Matlab, with the
%inputs being diff(function,variable to differentiate)
dr=diff(r,u);

%To find the unit tangent we divide by the magnitude of r'=dr. To make this
%result look nice, we want to add some assumptions
assume(u,{'real','positive'});

%Solving for unit tangent vector but using simplify to make it look nice
T_hat_ugly=dr./norm(dr);
T_hat=simplify(T_hat_ugly);

%Next, we want to find the unit normal vector
dT_hat=diff(T_hat,u);
N_hat=dT_hat/norm(dT_hat);
N_hat=simplify(N_hat);

%The unit binormal vector is found from the cross product of the unit
%tangent and unit normal
B_hat=cross(T_hat,N_hat);
B_hat=simplify(B_hat);

%We can also find the curvature
kappa=norm(dT_hat)/norm(dr);
kappa=simplify(kappa);

%And Torsion!
tau=-N_hat*(diff(B_hat,u)/norm(dr))';

%If we wish to visualize this curve, we can make substitutions into our
%functions. 
u_num = linspace(0,3.2,100); % define a set of evenly spaced points between 0 and 2*pi

%we will also convert to a number of type double
hold on
for n=1:length(u_num)
    r_num(n,:)=double(subs(r,u,u_num(n)));
    T_hat_num(n,:)=double(subs(T_hat,u,u_num(n)));
    N_hat_num(n,:)=double(subs(N_hat,u,u_num(n)));
end

figure(1)
hold on
plot3(r_num(:,1),r_num(:,2),r_num(:,3)), axis equal, hold on % plot the entire curve
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