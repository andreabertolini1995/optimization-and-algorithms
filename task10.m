%l2 formulation

E = [1 0 0 0; 0 1 0 0];    
T = 80;                                    %final time (robot has to arrive)
U = 15;                                    %maximum force appliable
tau = [10 25 30 40 50 60];                 %appointed times
w = [10 20 30 30 20 10; 10 10 10 0 0 -10]; %way points
lambda = 0.1;

A = [1 0 0.1 0; 0 1 0 0.1; 0 0 0.9 0; 0 0 0 0.9];
B = [0 0; 0 0; 0.1 0; 0 0.1];

% optimization
cvx_begin quiet
   variables x(4,79) u(2,80); 
   
   %build the function
   f = 0;
   for i = 1:6
       f = f + norm(E * x(:,tau(i)) - w(:,i));
   end
        
   minimize(f);
   
   %subject to
   x1 = [0 5 0 0];        %initial point
   x80 = [15 -15 0 0];    %final point
   
   for t = 1:80
       norm(u(:,t)) <= U;
   end
   
   x(:,1) == A*x1' + B*u(:,1);            %condition on the initial point
   [15 -15 0 0]' == A*x(:,79) + B*u(:,80) %condition on the final point
   for t = 1:78
       x(:,t+1) == A*x(:,t) + B*u(:,t+1);
   end
   
cvx_end;

%plot the optimal positions of the robot
figure(1);
hold on;
plot(x(1,:), x(2,:), 'bo', 'MarkerSize', 8);
plot(x1(1), x1(2), 'bo', 'MarkerSize', 8);
plot(x80(1), x80(2), 'bo', 'MarkerSize', 8);

for i = 1:6
    plot(x(1,tau(i)), x(2,tau(i)), 'mo', 'MarkerSize', 15);
    plot(w(1,i), w(2,i), 'rs', 'MarkerSize', 15) 
end

xlim([0 35])
ylim([-15 15])
grid on;

%plot the optimal control signal u(t)
t = [1:80];
figure(2);
hold on;
plot(t, u(1,:), 'LineWidth',2);
plot(t, u(2,:), 'LineWidth',2);
legend('u1(t)','u2(t)')
ylim([-40 40])
grid on;

%report how many points are captured by the robotwaypoint = 0;
for i = 1:6
    if norm(x(1:2,tau(i)) - w(:,i)) <= 10^(-6)
        waypoint = waypoint + 1;
    end
end

result = 'The robot captures %2.0f waypoints.\n';
fprintf(result,waypoint)

