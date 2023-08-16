clc; clear;close all;
prompt = "Which Scenario do you want me to show you? \n  \n 1.Static Obstacle Avoidance \n 2.Crowd Lane Changing \n 3.Overtaking Pass \n 4.Overtaking Yield \n \n Please enter the scenario number you prefer: ";
num_scenario = input(prompt);

%% Scenario 1
if num_scenario ==1
    Ts = 0.2;
    nx = 4;
    nu = 2;
    N = 40;
    x0 = [0,-2,8,0]';
    %% Create Reference Trajectory
    X_ref(:,1) = x0;
    u_ref= zeros(nu,N);
    for i = 2:N
        X_ref(:,i) = Dynamics(X_ref(:,i-1),u_ref(:,i));
    end
    
    %% Create Nominal Trajectory
    turn = deg2rad(20);
    u_nom = zeros(nu,N);
    u_nom = [5 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 20 20 20;
        0 0 0 0 turn turn turn 0 0 0 -turn -turn -turn 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -turn -turn -turn 0 0 0 turn turn turn 0 0 0];
    x_nom(:,1) = x0;
    for i = 1:N-1
        x_nom(:,i+1) = Dynamics(x_nom(:,i),u_nom(:,i));
    end
    
    %% ILQR
    xN = X_ref(:,N);
    obs = [20,30,40;-2,-2,-2;0,0,0;0,0,0];
    t = 0.7;
    mu = 1.2;
    alpha = 0.8;
    aH = 2;
    aL = -2;
    s = deg2rad(45);
    xout = zeros(4,N);
    uout = zeros(2,N-1);
    q = zeros(2,N);
    Q = zeros(2,4,N);
    while t < 6 %Outer Loop
        j = 2;
        xnomIn(:,:,1) = zeros(4,N);
        unomIn(:,:,1) = zeros(2,N-1);
        xnomIn(:,:,2) = x_nom;
        unomIn(:,:,2) = u_nom;
        %Inner Loop
        while norm(xnomIn(:,:,j)-xnomIn(:,:,j-1)) > 1
            %Iterative LQR
            [q,Q,dist] = ILQR(unomIn(:,:,j),xnomIn(:,:,j),t,X_ref,xN,obs);
            %Forward Pass
            xout(:,1) = x0;
            
            for i = 1:N-1
                uout(:,i) = unomIn(:,i,j) + q(:,:,i) + Q(:,:,i)*(xout(:,i)-xnomIn(:,i,j));
                %line search
                %acceleration constraints
                while uout(1,i) > aH
                    uout(1,i) = alpha*uout(1,i);
                end
                while uout(1,i) < aL
                    uout(1,i) = alpha*uout(1,i);
                end
                %steering constraints
                while uout(2,i) > s
                    uout(2,i) = alpha*uout(2,i);
                end
                while uout(2,i) < -s
                    uout(2,i) = alpha*uout(2,i);
                end
                xout(:,i+1) = Dynamics(xout(:,i),uout(:,i));
            end
            xnomIn(:,:,j+1) = xout;
            unomIn(:,:,j+1) = uout;
            %Assign iteration result
            j = j+1;
        end
        x_nom = xnomIn(:,:,j);
        u_nom = unomIn(:,:,j);
        %     figure(2)
        %     plot(x_nom(1,:),x_nom(2,:));hold on;
        t = mu*t;
    end
    %% Plotting
    % Create Road
    RoadL = -1:65;
    roadRight = 4*ones(size(RoadL));
    roadLeft = -4*ones(size(RoadL));
    roadCenter = 0*ones(size(RoadL));
    % Ploting Obstacles
    L = 1.5;
    w = 0.75;
    
    subplot(4,1,1)
    plot(RoadL,roadRight,'-k','LineWidth',2); hold on;
    plot(RoadL,roadLeft,'-k','LineWidth',2);
    plot(RoadL,roadCenter,'--k','LineWidth',1);
    for i = 1:3
        OBS(i) = polyshape([obs(1,i)+L/2 obs(1,i)+L/2 obs(1,i)-L/2 obs(1,i)-L/2],[obs(2,i)+w/2 obs(2,i)-w/2 obs(2,i)-w/2 obs(2,i)+w/2]);
        plot(OBS(i),'FaceColor','b')
    end
    for i = 1:N
        CAR(i) = polyshape([x_nom(1,i)+L/2 x_nom(1,i)+L/2 x_nom(1,i)-L/2 x_nom(1,i)-L/2],[x_nom(2,i)+w/2 x_nom(2,i)-w/2 x_nom(2,i)-w/2 x_nom(2,i)+w/2]);
        plot(CAR(i),'FaceColor','r')
    end
    title('Trajectory of the ego vehicle and the surrounding vehicle')
    xlabel('x(m)');
    ylabel('x(m)');
    % Velocity Profile
    subplot(4,1,2)
    plot(x_nom(1,:),x_nom(3,:));
    title('Speed profile of the ego vehicle')
    xlabel('x(m)');
    ylabel('v(m/s)');
    % Laterall Acceleration
    acc = u_nom(1,:).*sin(x_nom(4,1:N-1));
    subplot(4,1,3)
    plot(x_nom(1,1:N-1),acc);
    title('Lateral acceleration of the ego vehicle')
    xlabel('x(m)');
    ylabel('a(m/s^2)');
    %Minimum distance
    mdis = min(sqrt(dist));
    subplot(4,1,4)
    plot(x_nom(1,:),mdis);
    title('Distance from the ego vehicle to the nearest surrounding vehicle')
    xlabel('x(m)');
    ylabel('d(m)');
end

%% Scenario 2
if num_scenario ==2
    Ts = 0.2;
    nx = 4;
    nu = 2;
    N = 40;
    x0 = [0,-2,5,0]';
    x0_ref = [0,-2,8,0]';
    %% Create Reference Trajectory
    X_ref(:,1) = x0_ref;
    u_ref= zeros(nu,N);
    for i = 2:N
        X_ref(:,i) = Dynamics(X_ref(:,i-1),u_ref(:,i));
    end
    
    %% Create Nominal Trajectory
    turn = deg2rad(20);
    u_nom = zeros(nu,N);
    u_nom = [3 3 3 3 0 0 0 0 0 0 0 0 0 0 3 3 3 3 0 0 0 0 -3 -3 -3 -3 0 0 0 0 0 0 0 0 0 0 7 7 7;
        0 0 0 0 turn turn turn 0 0 0 -turn -turn -turn 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -turn -turn -turn 0 0 0 turn turn turn 0 0 0];
    x_nom(:,1) = x0;
    for i = 1:N-1
        x_nom(:,i+1) = Dynamics(x_nom(:,i),u_nom(:,i));
    end
    
    %% Moving Cars
    obs(:,1,1) = [0;2;3;0];
    obs(:,1,2) = [30;2;3;0];
    obs(:,1,3) = [25;-2;2;0];
    for j = 1:3
        for i = 1:N-1
            obs(:,i+1,j) = Dynamics(obs(:,i,j),u_ref(:,i));
        end
    end
    %% ILQR
    xN = X_ref(:,N);
    
    t = 0.7;
    mu = 1.2;
    alpha = 0.8;
    aH = 2.5;
    aL = -2.5;
    s = deg2rad(30);
    xout = zeros(4,N);
    uout = zeros(2,N-1);
    q = zeros(2,N);
    Q = zeros(2,4,N);
    while t < 7 %Outer Loop
        j = 2;
        xnomIn(:,:,1) = zeros(4,N);
        unomIn(:,:,1) = zeros(2,N-1);
        xnomIn(:,:,2) = x_nom;
        unomIn(:,:,2) = u_nom;
        %Inner Loop
        while norm(xnomIn(:,:,j)-xnomIn(:,:,j-1)) > 17
            [q,Q,dist] = ILQR(unomIn(:,:,j),xnomIn(:,:,j),t,X_ref,xN,obs);
            %Forward Pass
            xout(:,1) = x0;
            
            for i = 1:N-1
                uout(:,i) = unomIn(:,i,j) + q(:,:,i) + Q(:,:,i)*(xout(:,i)-xnomIn(:,i,j));
                %line search
                %acceleration constraints
                while uout(1,i) > aH
                    uout(1,i) = alpha*uout(1,i);
                end
                while uout(1,i) < aL
                    uout(1,i) = alpha*uout(1,i);
                end
                %steering constraints
                while uout(2,i) > s
                    uout(2,i) = alpha*uout(2,i);
                end
                while uout(2,i) < -s
                    uout(2,i) = alpha*uout(2,i);
                end
                xout(:,i+1) = Dynamics(xout(:,i),uout(:,i));
            end
            xnomIn(:,:,j+1) = xout;
            unomIn(:,:,j+1) = uout;
            %Assign iteration result
            j = j+1;
        end
        x_nom = xnomIn(:,:,j);
        u_nom = unomIn(:,:,j);
        
        t = mu*t;
    end
    %% Plotting
    % Create Road
    RoadL = -1:65;
    roadRight = 4*ones(size(RoadL));
    roadLeft = -4*ones(size(RoadL));
    roadCenter = 0*ones(size(RoadL));
    % Ploting Obstacles
    L = 1.5;
    w = 0.75;
    figure(1)
    subplot(4,1,1)
    plot(RoadL,roadRight,'-k','LineWidth',2); hold on;
    plot(RoadL,roadLeft,'-k','LineWidth',2);
    plot(RoadL,roadCenter,'--k','LineWidth',1);
    for i = 1:3
        for j = 1:N
            OBS(i,j) = polyshape([obs(1,j,i)+L/2 obs(1,j,i)+L/2 obs(1,j,i)-L/2 obs(1,j,i)-L/2],[obs(2,j,i)+w/2 obs(2,j,i)-w/2 obs(2,j,i)-w/2 obs(2,j,i)+w/2]);
            plot(OBS(i,j),'FaceColor','b')
        end
    end
    for i = 1:N
        CAR(i) = polyshape([x_nom(1,i)+L/2 x_nom(1,i)+L/2 x_nom(1,i)-L/2 x_nom(1,i)-L/2],[x_nom(2,i)+w/2 x_nom(2,i)-w/2 x_nom(2,i)-w/2 x_nom(2,i)+w/2]);
        plot(CAR(i),'FaceColor','r')
    end
    title('Trajectory of the ego vehicle and the surrounding vehicle')
    xlabel('x(m)');
    ylabel('x(m)');
    % Velocity Profile
    subplot(4,1,2)
    plot(x_nom(1,:),x_nom(3,:));
    title('Speed profile of the ego vehicle')
    xlabel('x(m)');
    ylabel('v(m/s)');
    % Laterall Acceleration
    acc = u_nom(1,:).*sin(x_nom(4,1:N-1));
    subplot(4,1,3)
    plot(x_nom(1,1:N-1),acc);
    title('Lateral acceleration of the ego vehicle')
    xlabel('x(m)');
    ylabel('a(m/s^2)');
    %Minimum distance
    mdis = min(sqrt(dist));
    subplot(4,1,4)
    plot(x_nom(1,:),mdis);
    title('Distance from the ego vehicle to the nearest surrounding vehicle')
    xlabel('x(m)');
    ylabel('d(m)');
end

%% Scenario 3
if num_scenario ==3
    Ts = 0.2;
    nx = 4;
    nu = 2;
    N = 40;
    x0 = [0,-2,10,0]';
    x0_ref = [0,-2,10,0]';
    %% Create Reference Trajectory
    X_ref(:,1) = x0_ref;
    u_ref= zeros(nu,N);
    for i = 2:N
        X_ref(:,i) = Dynamics(X_ref(:,i-1),u_ref(:,i-1));
    end
    
    %% Create Nominal Trajectory
    turn = deg2rad(10);
    u_nom = zeros(nu,N);
    u_nom = [-3 -3 -3 0 0 0 0 0 0 0 0 0 0 0 3 0 3 3 0 0 0 0 -3 0 -3 -3 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 turn turn turn 0 0 0 -turn -turn -turn 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -turn -turn -turn 0 0 0 turn turn turn 0 0 0];
    x_nom(:,1) = x0;
    for i = 1:N-1
        x_nom(:,i+1) = Dynamics(x_nom(:,i),u_nom(:,i));
    end
    %% Moving Cars
    obs(:,1,1) = [60;2;-9;0];
    obs(:,1,2) = [20;-2;3;0];
    for j = 1:2
        for i = 1:N-1
            obs(:,i+1,j) = Dynamics(obs(:,i,j),u_ref(:,i));
        end
    end
    %% ILQR
    xN = X_ref(:,N);
    
    t = 0.7;
    mu = 1.2;
    alpha = 0.8;
    aH = 2.5;
    aL = -2.5;
    s = deg2rad(30);
    xout = zeros(4,N);
    uout = zeros(2,N-1);
    q = zeros(2,N);
    Q = zeros(2,4,N);
    while t < 15 %Outer Loop
        j = 2;
        xnomIn(:,:,1) = zeros(4,N);
        unomIn(:,:,1) = zeros(2,N-1);
        xnomIn(:,:,2) = x_nom;
        unomIn(:,:,2) = u_nom;
        %Inner Loop
        while norm(xnomIn(:,:,j)-xnomIn(:,:,j-1)) > 10
            %Iterative LQR
            [q,Q,dist] = ILQR(unomIn(:,:,j),xnomIn(:,:,j),t,X_ref,xN,obs);
            %Forward Pass
            xout(:,1) = x0;
            
            for i = 1:N-1
                uout(:,i) = unomIn(:,i,j) + q(:,:,i) + Q(:,:,i)*(xout(:,i)-xnomIn(:,i,j));
                %line search
                %acceleration constraints
                while uout(1,i) > aH
                    uout(1,i) = alpha*uout(1,i);
                end
                while uout(1,i) < aL
                    uout(1,i) = alpha*uout(1,i);
                end
                %steering constraints
                while uout(2,i) > s
                    uout(2,i) = alpha*uout(2,i);
                end
                while uout(2,i) < -s
                    uout(2,i) = alpha*uout(2,i);
                end
                xout(:,i+1) = Dynamics(xout(:,i),uout(:,i));
            end
            xnomIn(:,:,j+1) = xout;
            unomIn(:,:,j+1) = uout;
            %Assign iteration result
            j = j+1;
        end
        x_nom = xnomIn(:,:,j);
        u_nom = unomIn(:,:,j);
        t = mu*t;
    end
    %% Plotting
    % Create Road
    RoadL = -1:65;
    roadRight = 4*ones(size(RoadL));
    roadLeft = -4*ones(size(RoadL));
    roadCenter = 0*ones(size(RoadL));
    % Ploting Obstacles
    L = 1.5;
    w = 0.75;
    figure(1)
    subplot(4,1,1)
    plot(RoadL,roadRight,'-k','LineWidth',2); hold on;
    plot(RoadL,roadLeft,'-k','LineWidth',2);
    plot(RoadL,roadCenter,'--k','LineWidth',1);
    for i = 1:2
        for j = 1:N
            OBS(i,j) = polyshape([obs(1,j,i)+L/2 obs(1,j,i)+L/2 obs(1,j,i)-L/2 obs(1,j,i)-L/2],[obs(2,j,i)+w/2 obs(2,j,i)-w/2 obs(2,j,i)-w/2 obs(2,j,i)+w/2]);
            plot(OBS(i,j),'FaceColor','b')
        end
    end
    for i = 1:N
        CAR(i) = polyshape([x_nom(1,i)+L/2 x_nom(1,i)+L/2 x_nom(1,i)-L/2 x_nom(1,i)-L/2],[x_nom(2,i)+w/2 x_nom(2,i)-w/2 x_nom(2,i)-w/2 x_nom(2,i)+w/2]);
        plot(CAR(i),'FaceColor','r')
    end
    title('Trajectory of the ego vehicle and the surrounding vehicle')
    xlabel('x(m)');
    ylabel('x(m)');
    xlim([-2 70])
    % Velocity Profile
    subplot(4,1,2)
    plot(x_nom(1,:),x_nom(3,:));
    title('Speed profile of the ego vehicle')
    xlabel('x(m)');
    ylabel('v(m/s)');
    % Laterall Acceleration
    acc = u_nom(1,:).*sin(x_nom(4,1:N-1));
    subplot(4,1,3)
    plot(x_nom(1,1:N-1),acc);
    title('Lateral acceleration of the ego vehicle')
    xlabel('x(m)');
    ylabel('a(m/s^2)');
    % Minimum distance
    mdis = min(sqrt(dist));
    subplot(4,1,4)
    plot(x_nom(1,:),mdis);
    title('Distance from the ego vehicle to the nearest surrounding vehicle')
    xlabel('x(m)');
    ylabel('d(m)');
    
end

%% Scenario 4
if num_scenario ==4
    
    Ts = 0.2;
    nx = 4;
    nu = 2;
    N = 40;
    x0 = [0,-2,10,0]';
    x0_ref = [0,-2,8,0]';
    %% Create Reference Trajectory
    X_ref(:,1) = x0_ref;
    u_ref= zeros(nu,N);
    for i = 2:N
        X_ref(:,i) = Dynamics(X_ref(:,i-1),u_ref(:,i));
    end
    
    %% Create Nominal Trajectory
    turn = deg2rad(10);
    u_nom = zeros(nu,N);
    u_nom = [-3 -3 -3 0 0 0 0 0 0 0 0 0 0 0 3 0 3 3 0 0 0 0 -3 0 -3 -3 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 turn turn turn 0 0 0 -turn -turn -turn 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -turn -turn -turn 0 0 0 turn turn turn 0 0 0];
    x_nom(:,1) = x0;
    for i = 1:N-1
        x_nom(:,i+1) = Dynamics(x_nom(:,i),u_nom(:,i));
    end
    
    %% Moving Cars
    obs(:,1,1) = [60;2;-15;0];
    obs(:,1,2) = [20;-2;3;0];
    for j = 1:2
        for i = 1:N-1
            obs(:,i+1,j) = Dynamics(obs(:,i,j),u_ref(:,i));
        end
    end
    %% ILQR
    xN = X_ref(:,N);
    
    t = 0.7;
    mu = 1.2;
    alpha = 0.8;
    aH = 2;
    aL = -2;
    s = deg2rad(30);
    xout = zeros(4,N);
    uout = zeros(2,N-1);
    q = zeros(2,N);
    Q = zeros(2,4,N);
    while t < 10 %Outer Loop
        j = 2;
        xnomIn(:,:,1) = zeros(4,N);
        unomIn(:,:,1) = zeros(2,N-1);
        xnomIn(:,:,2) = x_nom;
        unomIn(:,:,2) = u_nom;
        %Inner Loop
        while norm(xnomIn(:,:,j)-xnomIn(:,:,j-1)) > 20
            [q,Q,dist] = ILQR(unomIn(:,:,j),xnomIn(:,:,j),t,X_ref,xN,obs);
            %Forward Pass
            xout(:,1) = x0;
            
            for i = 1:N-1
                uout(:,i) = unomIn(:,i,j) + q(:,:,i) + Q(:,:,i)*(xout(:,i)-xnomIn(:,i,j));
                %line search
                %acceleration constraints
                while uout(1,i) > aH
                    uout(1,i) = alpha*uout(1,i);
                end
                while uout(1,i) < aL
                    uout(1,i) = alpha*uout(1,i);
                end
                %steering constraints
                while uout(2,i) > s
                    uout(2,i) = alpha*uout(2,i);
                end
                while uout(2,i) < -s
                    uout(2,i) = alpha*uout(2,i);
                end
                xout(:,i+1) = Dynamics(xout(:,i),uout(:,i));
            end
            xnomIn(:,:,j+1) = xout;
            unomIn(:,:,j+1) = uout;
            %Assign iteration result
            j = j+1;
        end
        x_nom = xnomIn(:,:,j);
        u_nom = unomIn(:,:,j);
        t = mu*t;
    end
    %% Plotting
    % Create Road
    RoadL = -1:65;
    roadRight = 4*ones(size(RoadL));
    roadLeft = -4*ones(size(RoadL));
    roadCenter = 0*ones(size(RoadL));
    % Ploting Obstacles
    L = 1.5;
    w = 0.75;
    figure(1)
    subplot(4,1,1)
    plot(RoadL,roadRight,'-k','LineWidth',2); hold on;
    plot(RoadL,roadLeft,'-k','LineWidth',2);
    plot(RoadL,roadCenter,'--k','LineWidth',1);
    for i = 1:2
        for j = 1:N
            OBS(i,j) = polyshape([obs(1,j,i)+L/2 obs(1,j,i)+L/2 obs(1,j,i)-L/2 obs(1,j,i)-L/2],[obs(2,j,i)+w/2 obs(2,j,i)-w/2 obs(2,j,i)-w/2 obs(2,j,i)+w/2]);
            plot(OBS(i,j),'FaceColor','b')
        end
    end
    for i = 1:N
        CAR(i) = polyshape([x_nom(1,i)+L/2 x_nom(1,i)+L/2 x_nom(1,i)-L/2 x_nom(1,i)-L/2],[x_nom(2,i)+w/2 x_nom(2,i)-w/2 x_nom(2,i)-w/2 x_nom(2,i)+w/2]);
        plot(CAR(i),'FaceColor','r')
    end
    title('Trajectory of the ego vehicle and the surrounding vehicle')
    xlabel('x(m)');
    ylabel('x(m)');
    xlim([-2 70])
    % Velocity Profile
    subplot(4,1,2)
    plot(x_nom(1,:),x_nom(3,:));
    title('Speed profile of the ego vehicle')
    xlabel('x(m)');
    ylabel('v(m/s)');
    % Laterall Acceleration
    acc = u_nom(1,:).*sin(x_nom(4,1:N-1));
    subplot(4,1,3)
    plot(x_nom(1,1:N-1),acc);
    title('Lateral acceleration of the ego vehicle')
    xlabel('x(m)');
    ylabel('a(m/s^2)');
    %Minimum distance
    mdis = min(sqrt(dist));
    subplot(4,1,4)
    plot(x_nom(1,:),mdis);
    title('Distance from the ego vehicle to the nearest surrounding vehicle')
    xlabel('x(m)');
    ylabel('d(m)');
    
end
fprintf(' Thanks! \n')