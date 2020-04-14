    
function betasweep(beta)
syms u t

assume(0 <= u <= 3.2)
d = 0.235; % wheel distance
steps = 100; % number of simulation steps

u = beta * t;

ri=4*(0.3960*cos(2.65*(u+1.4)));
rj=-4*(0.99*sin(u+1.4));
rk=0*u;

r=[ri,rj,rk];

dr=diff(r,t); %unit tangent
V = norm(dr); %linear speed

T_hat_ugly=dr./norm(dr);
T_hat=simplify(T_hat_ugly);

dT_hat = diff(T_hat,t);

omega=simplify(cross(T_hat,dT_hat));

VL = simplify(V - omega(3) * (d / 2));
VR = simplify(V + omega(3) * (d / 2));

t_num = linspace(0,3.2/beta,steps);
pub = rospublisher('/raw_vel')
message = rosmessage(pub);
for n=1:length(t_num)
    VL_num(n,:)=double(subs(VL,[t],[t_num(n)]));
    VR_num(n,:)=double(subs(VR,[t],[t_num(n)]));
end
r = rosrate(10.5);
reset(r);

for i = 1:100
    message.Data = [VL_num(i,:), VR_num(i,:)];
    send(pub, message);
    waitfor(r);
    if i>=100
    message.Data = [0, 0];
    send(pub, message);
    end
end
end
