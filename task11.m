%iterative reweighting technique

E = [1 0 0 0; 0 1 0 0];    
T = 80;                                    %final time (robot has to arrive)
U = 15;                                    %maximum force appliable
epsilon = 10^(-6);                         %constant that prevents the denominator to be zero
M = 10;                                    %reweighting technique - number of times
tau = [10 25 30 40 50 60];                 %appointed times
w = [10 20 30 30 20 10; 10 10 10 0 0 -10]; %way points
lambda = 0.1;

A = [1 0 0.1 0; 0 1 0 0.1; 0 0 0.9 0; 0 0 0 0.9];
B = [0 0; 0 0; 0.1 0; 0 0.1];

% first optimization (task 10)
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

%at this point x and u are initialized with the values of task 10
% second optimization (task 11)
for m = 1:M
    y = x;   %I store the m-th value of x in a new variable (it's a different "x" from the one inside the function)
    
    %nested optimization: it returns a new values of x and u every time
    cvx_begin quiet
        variables x(4,79) u(2,80); 
   
        %build the function
        f = 0;
        for i = 1:6
            
            %computing the initial weight (not sure how u0 comes in the page)
            
            weight = 1/(norm(E * y(:,tau(i)) - w(:,i)) + epsilon);
            f = f + weight * norm(E * x(:,tau(i)) - w(:,i));     
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
     figure();
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
     figure();
     hold on;
     plot(t, u(1,:), 'LineWidth',2);
     plot(t, u(2,:), 'LineWidth',2);
     legend('u1(t)','u2(t)')
     grid on;
     ylim([-40 40])
     
     %report how many points are captured by the robot
     waypoint = 0;
     for i = 1:6
          if norm(x(1:2,tau(i)) - w(:,i)) <= 10^(-6) 
             waypoint = waypoint + 1;
          end
     end
     
     result = 'The robot captures %2.0f waypoints.\n';
     fprintf(result,waypoint);

end






