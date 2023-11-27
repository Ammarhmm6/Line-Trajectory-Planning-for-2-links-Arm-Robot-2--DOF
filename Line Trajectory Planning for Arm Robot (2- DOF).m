% Robot parameters
L1 = 0.9; % Length of link 1
L2 = 0.5; % Length of link 2
L= L1+L2;

% Input initial position
Xi = input('Enter initial X position: ');
Yi = input('Enter initial Y position: ');

% Input final position-0.31.3
Xf = input('Enter final X position: ');
Yf = input('Enter final Y position: ');

% Set up trajectory
N = input('Enter the number of divisions for the trajectory: ');
Qx = zeros(N+1, 1);
Qy = zeros(N+1, 1);

% Change in X and Y for trajectory
dX = (Xf - Xi) / N;
dY = (Yf - Yi) / N;

Qx(1) = Xi;
Qy(1) = Yi;
for i= 2:N+1
    Qx(i) = Qx(i-1)+dX;
    Qy(i) = Qy(i-1)+dY;
end
Q= [Qx, Qy];

% Inverse kinematics part
Rad2Ang = 180/pi();
Phi = zeros(N+1,1);
Theta1 = zeros(N+1,1);
Theta2 = zeros(N+1,1);

for i = 1:N+1
    b = sqrt(Qx(i)^2+Qy(i)^2);
    alpha =  acos((b^2+L1^2-L2^2)/(2*b*L1));
    beta =   acos((L1^2+L2^2-b^2)/(2*L1*L2));

    Phi(i) = atan(Qy(i)/Qx(i));
    Theta1(i) = Phi(i)-alpha;
    Theta2(i) = pi()-beta;
end

%forward kinmatics part
Theta = Theta1+Theta2;
X0 = zeros(N+1,1);
Y0 = zeros(N+1,1);
X1 = zeros(N+1,1);
Y1 = zeros(N+1,1);
X = zeros(N+1,1);
Y = zeros(N+1,1);

for i = 1:N+1
    X1(i) = L1*cos(Theta1(i));
    Y1(i) = L1*sin(Theta1(i)); 
    X(i) = X1(i)+L2*cos(Theta(i));
    Y(i) = Y1(i)+L2*sin(Theta(i));
end 

% Plotting part
figure(1)
hold on;
th = linspace(0,2*pi,360);
R = [abs(L2-L1),L];

%Plot working envelop
for j=1:2
    xa = R(j)*cos(th);
    ya = R(j)*sin(th);
    plot(xa,ya,':k');
end

%graph setting
box on;
xlabel('[m]');
ylabel('[m]');
axis equal;
axis([-L L -L L]);


% plot trajectory
for i=1:N+1
   plot(Qx(1:i),Qy(1:i),'-r','LineWidth',1.5);
   
   %arm setting
   A = [X0(i);Y0(i)];
   B = [X1(i);Y1(i)];
   C = [X(i);Y(i)];
   
   if Qx(i) < 0
       Arm = [-A,-B,-C];
   else
        Arm = [A,B,C];
   end
   
   plot(Arm(1,:),Arm(2,:),'-o','LineWidth',2,...
       'MarkerEdgeColor','k','MarkerFaceColor','c','MarkerSize',5);
   pause(2);
   
   
end
