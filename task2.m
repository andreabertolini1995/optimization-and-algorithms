%l2 regularizer

E = [1 0 0 0; 0 1 0 0];    
T = 80;                                    %final time (robot has to arrive)
U = 100;                                   %maximum force appliable
tau = [10 25 30 40 50 60];                 %appointed times
w = [10 20 30 30 20 10; 10 10 10 0 0 -10]; %way points
lambda = 10;

A = [1 0 0.1 0; 0 1 0 0.1; 0 0 0.9 0; 0 0 0 0.9];
B = [0 0; 0 0; 0.1 0; 0 0.1];

% optimization
cvx_begin quiet
   variables x(4,79) u(2,80); 
   
   %build the function
   f1 = 0;
   for i = 1:6
       f1 = f1 + sum_square_abs(E * x(:,tau(i)) - w(:,i));
   end
         
   f2 = 0;
   for j = 2:80
       f2 = f2 + norm(u(:,j) - u(:,j-1)); 
   end
      
   f = f1 + lambda*f2;
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

grid on;
xlim([0 35])
ylim([-15 15])

%plot the optimal control signal u(t)
t = [1:80];
figure(2);
hold on;
plot(t, u(1,:), 'LineWidth',2);
plot(t, u(2,:), 'LineWidth',2);
legend('u1(t)','u2(t)')
ylim([-40 40])
grid on

leo = [];
for i = 2:80
    leo(i-1) = (norm(u(:,i)-u(:,i-1)));
end


%report how many times the optimal control signal changes
changes = 0;
for i = 2:80
    if norm(u(:,i)-u(:,i-1)) > 10^(-6)
        changes = changes + 1;
    end
end

result = 'The optimal control signal changes %2.0f times.\n';
fprintf(result,changes)

%report the mean deviation from the waypoints
dev = 0;
for i = 1:6
       dev = dev + norm(E * x(:,tau(i)) - w(:,i));
end
dev = dev/6;

result = 'The mean deviation from the waypoints is %5.4f.\n';
fprintf(result,dev)

