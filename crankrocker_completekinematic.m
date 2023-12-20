%%=========================================================================
% Crank-Rocker Mechanism - Program Description
% =========================================================================
% This program perfoms a complete kinematic analysis of a fourbar 
% mechanism with respect position analysis, velocity analysis and
% acceleration analysis, using the Freudenstein equations. 
% -----------------
% Input and Outputs
% -----------------
% The inputs are the four link dimensions: crank (a), coupler (b), 
% rocker(c), and ground (d); the initial position of the crank (theta2) and
% the freeze frame angle (ffa).
% The outputs are the link angular positions for the coupler (theta3) and 
% the rocker (theta4) and the positions in space for the link joints and
% coupler point; the angular velocities for the coupler (omega3) and the 
% rocker (omega4) and the linear velocities of the link joints and coupler
% point; the angular accelerations for the coupler (alpha3) and the rocker
% (alpha4) and the linear accelerations of the link joints and coupler
% point. 
%==========================================================================
 
%% Defining all dimensions and initial conditions
a = 1.00;            % crank length [m] or suitable unit like [mm]
b = 2.06;            % coupler length [m]  
c0 = 2.33;           % rocker length [m] 
d = 2.22;            % ground length [m]  
p = 3.06;            % distance to coupler point from point A [m] 
delta3 = -31;        % angle from coupler to coupler point line [degrees]
omega2 = 10;         % crank angular velocity [rad/s]
alpha2 = 0;          % crank angular acceleration [rad/s^2]
config = 1;          % configuration, whether open [1] or crossed [2]
ffa = 120;            % freeze-frame angle [degrees]
% Point coordinates and kinematics for plots
p0 = [0;0];     % ground pin coordinates at orign O 
pC = [d;0];     % ground pin coordinates at at D
v0 = [0;0];     % velocity at origin O (zero/static)
a0 = [0;0];     % acceleration at origin O (zero/static)
% Test for Grashof condition
S = min([a,b,c0,d]);    % shortest link length
L = max([a,b,c0,d]);    % longest link length
SL = S+L;               % sum of lengths of shortest and longest links
PQ = (a+b+c0+d)-SL;     % sum of lengths of the other two links
if SL <= PQ             % Grashoff condition for fourbar mechanisms
    disp('Mechanism is Grashoff')
else
    disp('Mechanism is non-Grashoff')   
end
% Fourbar classification
if min([a,b,c0,d]) == a
    disp('This is a crank-rocker mechanism with full rotation of the crank')
elseif min([a,b,c0,d])== d
    disp('This is a drag-link mechanism with full rotation of both grounded links')
elseif min([a,b,c0,d])== b
    disp('This is a double-rocker mechanism with no link in full rotation')
end
%% Manipulated Freudenstein variables
k1 = d/a;
k2 = d/c0;
k3 = (a^2+c0^2+d^2-b^2)/(2*a*c0);
k4 = d/b;
k5 = (c0^2-d^2-a^2-b^2)/(2*a*b); 
% Setting calculation limits
n = 361;                % last angle; default step = 1
% Initializing variables (pre-allocating memory)
[theta2,theta3,theta4] = deal(zeros(1,n));   
[omega3,omega4] = deal(zeros(1,n));  
[alpha3,alpha4] = deal(zeros(1,n));
[pA,pB,pP] = deal(zeros(2,n));  
[vA,vB,vP] = deal(zeros(2,n));  
[aA,aB,aP] = deal(zeros(2,n));  
 
%% Main loop  
for i = 1:n
    theta2(i) = i-1;
    X = cos(deg2rad(theta2(i)))-k1+k4*cos(deg2rad(theta2(i)))+k5;
    Y = -2*sin(deg2rad(theta2(i)));
    Z = k1+(k4-1)*cos(deg2rad(theta2(i)))+k5;
    rcand1 = Y^2-4*X*Z;
    U = cos(deg2rad(theta2(i)))-k1-k2*cos(deg2rad(theta2(i)))+k3;
    V =-2*sin(deg2rad(theta2(i)));
    W = k1-(k2+1)*cos(deg2rad(theta2(i)))+k3;
    rcand2 = V^2-4*U*W;
    % Position Analysis
    % -----------------
    % Calculations of theta3 (open & crossed)
    if config == 1
            theta3(i) = 2*rad2deg(atan((-Y-sqrt(rcand1))/(2*X))); % open
        else
            theta3(i) = 2*rad2deg(atan((-Y+sqrt(rcand1))/(2*X))); % crossed
    end
    % Calculations of theta4 (open & crossed)
    if config == 1
            theta4(i) = 2*rad2deg(atan((-V-sqrt(rcand2))/(2*U))); % open
        else
            theta4(i) = 2*rad2deg(atan((-V+sqrt(rcand2))/(2*U))); % crossed
    end
    % Calculations of unit vectors/unit normals
    [e2,n2] = uvector(deg2rad(theta2(i)));   % unit vector/unit normal for crank
    [e3,n3] = uvector(deg2rad(theta3(i)));   % unit vector/unit normal for coupler
    [e4,n4] = uvector(deg2rad(theta4(i)));   % unit vector/unit normal for rocker
    [ePA,nPA] = uvector(deg2rad(theta3(i) + delta3)); % unit vector/normal for coupler point
    % Solving for positions of A, B and P on the linkage
    pA(:,i) = relpos(p0,a,e2);          % position of A relative to origin O
    pB(:,i) = relpos(pC,c0,e4);         % position of B relative to C
    pP(:,i) = relpos(pA(:,i),p,ePA);    % position of P relative to A
    % Velocity Analysis
    % -----------------
    % Calculation of omega3
    omega3(i) = (a*omega2/b)*sin(deg2rad(theta4(i))-deg2rad(theta2(i)))/...
        sin(deg2rad(theta3(i))-deg2rad(theta4(i)));
    % Calculation of omega4
    omega4(i) = (a*omega2/c0)*sin(deg2rad(theta2(i))-deg2rad(theta3(i)))/...
        sin(deg2rad(theta4(i))-deg2rad(theta3(i)));
    % Solving for linear velocities at A, B and P
    vA(:,i) = relvel(v0,a,omega2,n2);          % velocity of A relative to origin O
    vB(:,i) = relvel(v0,c0,omega4(i),n4);      % velocity of B relative to C
    vP(:,i) = relvel(vA(:,i),p,omega3(i),nPA); % velocity of P relative to A
    % Acceleration Analysis
    % ---------------------
    % Calculation of alpha3
    O = c0*sin(deg2rad(theta4(i)));
    P = b*sin(deg2rad(theta3(i)));
    Q = a*alpha2*sin(deg2rad(theta2(i)))+a*(omega2^2)*cos(deg2rad(theta2(i)))...
        +b*(omega3(i)^2)*cos(deg2rad(theta3(i)))-c0*(omega4(i)^2)*cos(deg2rad(theta4(i)));
    R = c0*cos(deg2rad(theta4(i)));
    S = b*cos(deg2rad(theta3(i)));
    T = a*alpha2*cos(deg2rad(theta2(i)))-a*(omega2^2)*sin(deg2rad(theta2(i)))-b*...
        (omega3(i)^2)*sin(deg2rad(theta3(i)))+c0*(omega4(i)^2)*sin(deg2rad(theta4(i)));
    alpha3(i) = (Q*R-O*T)/(O*S-P*R);
    % Calculation of alpha4
    O = c0*sin(deg2rad(theta4(i)));
    P = b*sin(deg2rad(theta3(i)));
    Q = a*alpha2*sin(deg2rad(theta2(i)))+a*(omega2^2)*cos(deg2rad(theta2(i)))+b...
        *(omega3(i)^2)*cos(deg2rad(theta3(i)))-c0*(omega4(i)^2)*cos(deg2rad(theta4(i)));
    R = c0*cos(deg2rad(theta4(i)));
    S = b*cos(deg2rad(theta3(i)));
    T = a*alpha2*cos(deg2rad(theta2(i)))-a*(omega2^2)*sin(deg2rad(theta2(i)))-b...
        *(omega3(i)^2)*sin(deg2rad(theta3(i)))+c0*(omega4(i)^2)*sin(deg2rad(theta4(i)));
    alpha4(i) = (Q*S-P*T)/(O*S-P*R);
    % Solving for linear accelerations at A, B and P
    aA(:,i) = relaccel(a0,a,omega2,alpha2,e2,n2);              % accel. of A rel. to O
    aB(:,i) = relaccel(a0,c0,omega4(i),alpha4(i),e4,n4);       % accel. of B rel. to C
    aP(:,i) = relaccel(aA(:,i),p,omega3(i),alpha3(i),ePA,nPA); % accel. of P rel. to A
    
end

%% Plots
% Plot the angular positions of the coupler (theta3) and rocker (theta4)
subplot(3,2,1)
plot(theta2,theta3,'Color','r')
hold on
plot(theta2,theta4,'Color','b')
title('Kinematics of fourbar linkage - theta3 / theta4')
xlabel('Crank angle [degrees]')
ylabel('Coupler/rocker position')
legend('theta3','theta4','Location','Southeast')
grid on
set(gca,'xtick',0:30:360)
xlim([0 360])
% Plot the positions of A, B and P
subplot(3,2,2)
plot(pA(1,:),pA(2,:),pB(1,:),pB(2,:),pP(1,:),pP(2,:));
hold on
% Plot the coupler as a triangular patch A, B and P
patch([pA(1,ffa) pB(1,ffa) pP(1,ffa)],...
      [pA(2,ffa) pB(2,ffa) pP(2,ffa)],...
      [240/255   240/255   240/255]);
% Plot the crank, coupler, rocker links and the ground
plot([p0(1) pA(1,ffa)],[p0(2) pA(2,ffa)],'Linewidth',2.5,'Color','k');
plot([pA(1,ffa) pB(1,ffa)],[pA(2,ffa) pB(2,ffa)],'Linewidth',2.5,'Color','k');
plot([pC(1) pB(1,ffa)],[pC(2) pB(2,ffa)],'Linewidth',2.5,'Color','k');
plot([p0(1) pC(1)],[p0(1) pC(2)],'Linewidth',1,'Color','k');
% Label each joint
text(p0(1),p0(2)-0.15,'O','HorizontalAlignment','center');
text(pA(1,ffa),pA(2,ffa)+0.25,'A','HorizontalAlignment','center');
text(pB(1,ffa),pB(2,ffa)+0.25,'B','HorizontalAlignment','center');
text(pP(1,ffa),pP(2,ffa)+0.25,'P','HorizontalAlignment','center');
text(pC(1),pC(2)-0.15,'C','HorizontalAlignment','center');
axis equal
grid on
title('Locii of A, B and P with freeze frame at specified angle')
xlabel('Horizontal position')
ylabel('Vertical position')
% Plot the angular velocities of the coupler (omega3) and rocker (omega4)
subplot(3,2,3)
plot(theta2,omega3,'Color','r')
hold on
plot(theta2,omega4,'Color','b')
title('Kinematics of fourbar linkage - omega3 / omega4')
xlabel('Crank angle [degrees]')
ylabel('Coupler/rocker angular velocity')
legend('omega3','omega4','Location','Southeast')
grid on
set(gca,'xtick',0:30:360)
xlim([0 360])
% Plot the linear velocity of the coupler point P. 
% The linear velocities of point A will be found by setting p = 0 and 
% delta3 = 0; and the linear velocities of point B will be found by setting
% p = b and delta3 = 0.
subplot(3,2,4)
plot(theta2,sqrt(vP(1,:).^2+vP(2,:).^2),'Color','b')
title('Kinematics of fourbar linkage - velocity of P')
xlabel('Crank angle [degrees]')
ylabel('Linear velocity')
grid on
set(gca,'xtick',0:30:360)
xlim([0 360])
% Plot the angular accelerations of the coupler (alpha3) and rocker (alpha4)
subplot(3,2,5)
plot(theta2,alpha3,'Color','r')
hold on
plot(theta2,alpha4,'Color','b')
title('Kinematics of fourbar linkage - alpha3 / alpha4')
xlabel('Crank angle [degrees]')
ylabel('Coupler/rocker angular accel.')
legend('alpha3','alpha4','Location','Southeast')
grid on
set(gca,'xtick',0:30:360)
xlim([0 360])
% Plot the angular acceleration of coupler point P
% The linear accelerations of point A will be found by setting p = 0 and 
% delta3 = 0; and the linear accelerations of point B will be found by 
% setting p = b and delta3 = 0.
subplot(3,2,6)
plot(theta2,sqrt(aP(1,:).^2+aP(2,:).^2),'Color','r')
title('Kinematics of fourbar linkage - acceleration of P')
xlabel('Crank angle [degrees]')
ylabel('Linear acceleration')
grid on
set(gca,'xtick',0:30:360)
xlim([0 360])

%%
% Function relpos.m
% This function calculates the relative position of a second point on a
% link given the first point, the length and the unit vector
% x0 = position of first point on the link
% x = position of second point on the link
% L = length of vector between first and second points
% e = unit vector between first and second points
 
function x = relpos(x0, L, e)
x = x0 + L * e;
end

%%
% Function relvel.m
% This function calculates the linear velocity at a point on the linkage
% given the velocity of the initial point
% v0 = velocity of first point on the link
% v = velocity of second point on the link
% L = length of vector between the first and second points
% omega = angular velocity of link
% n = unit normal to vector between the first and second points
 
function v = relvel(v0, L, omega, n)
v = v0 + omega * L * n;
end

%%
% Function relaccel.m
% This function calculates the linear acceleration at a point on the linkage
% given the acceleration of the initial point
% a0 = acceleration of first point on the link
% a = acceleration of second point on the link
% L = length of vector between the first and second points
% omega = angular velocity of link
% alpha = angular acceleration of link
% e = unit vector between the first and second points
% n = unit normal to vector between the first and second points
 
function a = relaccel(a0, L, omega, alpha, e, n)
a = a0 + L*alpha*n - L*omega^2*e;
end

%%
% Function uvector.m
% This function calculates, for a given angle, both the unit vector and unit normal
% theta = given angle of the unit vector
% e = unit vector in the direction of theta
% n = unit normal perpendicular to the direction of theta
 
function [e,n] = uvector(theta)
e = [cos(theta); sin(theta)];
n = [-sin(theta); cos(theta)];
end