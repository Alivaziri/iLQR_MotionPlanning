function [dx,dy,distance] = CalculateDistance(x_nom,obs)
L = 1.5;
w = 0.75;
n = size(obs,2);
points = @(x)  [x(1),x(1),x(1)-L/2,x(1)-L/2,x(1)-L/2,x(1)+L/2,x(1)+L/2,x(1)+L/2;...
    x(2)-w/2,x(2)+w/2,x(2)-w/2,x(2),x(2)+w/2,x(2)-w/2,x(2),x(2)+w/2];
%% Points on the car
p_car = points(x_nom);
% adjust points based on heading
for i= 1:8
    p_car(:,i) =  [cos(x_nom(4)) -sin(x_nom(4));sin(x_nom(4)) cos(x_nom(4))]*p_car(:,i);
end
%% Obstacle Points
% adjust points based on heading
for j = 1:n
    p_obs(:,:,j) = points(obs(:,j));
    for i= 1:8
        p_obs(:,i,j) =  [cos(obs(4,j)) -sin(obs(4,j));sin(obs(4,j)) cos(obs(4,j))]*p_obs(:,i,j);
    end
end
%% Distance Measurement
for j = 1:n
    for l = 1:8
        for k = 1:8
            dis (l,k,j) = norm(p_car(:,l)-p_obs(:,k,j))^2 ;
        end
    end
end
%% Minimum distance
for j = 1:n
    L = dis(:,:,j);
    [~,idx] = min(L(:));
    [l(j),m(j)] = ind2sub(size(L),idx);
end
X = zeros(2,n);
O = zeros(2,n);
for j = 1:n
    X(:,j) = p_car(:,l(j));
    O(:,j) = p_obs(:,m(j),j);
end
%% Return difference in distance
dx = abs(X(1,:)-O(1,:));
dy = abs(X(2,:)-O(2,:));
distance = (dx.^2 + dy.^2);
end
