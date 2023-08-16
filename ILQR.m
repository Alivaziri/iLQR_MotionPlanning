function [q,Q,distance] = ILQR(NominuL_u,NominuL_x,t,xref,xN,obs)
%% InitiuLization
[N,T,L,wref,wv,wa,ws,uH,uL] = Initialize(NominuL_u,NominuL_x);
D_Star = 1;
s = deg2rad(30);
P_x = zeros(4,1,N);
P_u = zeros(2,1,N);
P_xx = zeros(4,4,N);
P_ux = zeros(2,4,N);
P_uu = zeros(2,2,N);
%% Functions 
L_x = @(x,x_r,dx,dy,distance) [2*wref*(x(1)-x_r(1)) - 2/t*sum((dx)./(distance-D_Star));...
                      2*wref*(x(2)-x_r(2)) - 2/t*sum((dy)./(distance-D_Star));...
                     2*wv*(x(3)-x_r(3));0];
                 
L_u = @(u)         [2*wa*u(1) + (1/t)*(1/(uH-u(1))-1/(u(1)-uL));...
                   2*ws*u(2) + (1/t)*(1/(s-u(2))-1/(u(2)+s))];
               
L_xx = @(x,dx,dy,distance) [2*wref-(2/t)*sum(((distance-D_Star)-2.*dx.^2)./(distance-D_Star).^2),...
                     (-2/t)*sum(-2*(dx.*dy)./(distance-D_Star).^2),0,0;...
                     (-2/t)*sum(-2*(dx.*dy)./(distance-D_Star).^2),...
                     2*wref-(2/t)*sum(((distance-D_Star)-2*dy.^2)./(distance-D_Star).^2),0,0;...
                      0,0,2*wv,0;0,0,0,0];
              
L_uu = @(u)        [2*wa + (1/t)*(1/(uH-u(1))^2 + 1/(u(1)-uL)^2),0;...  
                   0,2*ws + (1/t)*(1/(s-u(2))^2 + 1/(u(2)+s)^2)];
               
Kapa = @(u)        atan(0.5*tan(u(2)));

F_x = @(x,u)         [1,0,T*cos(x(4)+Kapa(u)),-x(3)*sin(x(4)+Kapa(u));...   
                    0,1,T*sin(x(4)+Kapa(u)),x(3)*cos(x(4)+Kapa(u));... 
                     0,0,1,0;0,0,2*T*sin(Kapa(u))/L,1];
                 
F_u = @(x,u)         [1,-x(3)*T*cos(x(4)+Kapa(u))*0.5*sec(u(2))^2/(0.25*tan(u(2))^2+1);0,x(3)*T*sin(x(4)+Kapa(u))*0.5*sec(u(2))^2/(0.25*tan(u(2))^2+1);T,0;...
                      0,(2*x(3)*T/L)*cos(Kapa(u))*0.5*sec(u(2))^2/(0.25*tan(u(2))^2+1)];
                  
q = zeros(2,1,N);
Q = zeros(2,4,N);
V_x  = zeros(4,1,N);

[dx(:,N),dy(:,N),distance(:,N)] = CalculateDistance(NominuL_x(:,N),obs);
V_x(:,:,N) = L_x(xN,xref(:,N),dx(:,N),dy(:,N),distance(:,N));
V_xx(:,:,N) = L_xx(xN,dx(:,N),dy(:,N),distance(:,N));


for i = N-1:-1:1
[dx(:,i), dy(:,i), distance(:,i)] = CalculateDistance(NominuL_x(:,i),obs);
P_x(:,:,i) = L_x(NominuL_x(:,i),xref(:,i),dx(:,i),dy(:,i),distance(:,i)) +  F_x(NominuL_x(:,i),NominuL_u(:,i))'*V_x(:,:,i+1);
P_u(:,:,i) = L_u(NominuL_u(:,i)) + F_u(NominuL_x(:,i),NominuL_u(:,i))'*V_x(:,:,i+1);
P_xx(:,:,i) = L_xx(NominuL_x(:,i),dx(:,i),dy(:,i),distance(:,i)) + F_x(NominuL_x(:,i),NominuL_u(:,i))'*V_xx(:,:,i+1)*F_x(NominuL_x(:,i),NominuL_u(:,i));
P_uu(:,:,i) = L_uu(NominuL_u(:,i)) + F_u(NominuL_x(:,i),NominuL_u(:,i))'*V_xx(:,:,i+1)*F_u(NominuL_x(:,i),NominuL_u(:,i));
P_ux(:,:,i) = F_u(NominuL_x(:,i),NominuL_u(:,i))'*V_xx(:,:,i+1)*F_x(NominuL_x(:,i),NominuL_u(:,i));
q(:,:,i) = -inv(P_uu(:,:,i))*P_u(:,:,i);
Q(:,:,i) = -inv(P_uu(:,:,i))*P_ux(:,:,i);
V_x(:,:,i) = P_x(:,:,i)-P_ux(:,:,i)'*inv(P_uu(:,:,i))*P_u(:,:,i);
V_xx(:,:,i) = P_xx(:,:,i)-P_ux(:,:,i)'*inv(P_uu(:,:,i))*P_ux(:,:,i);
end
end

