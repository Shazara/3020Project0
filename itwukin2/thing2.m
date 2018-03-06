function [sys,x0,str,ts] = thing2(t,x,u,flag)
%DSFUNC An example MATLAB file S-function for defining a discrete system.  
%   Example MATLAB file S-function implementing discrete equations: 
%      x(n+1) = Ax(n) + Bu(n)
%      y(n)   = Cx(n) + Du(n)
%   
%   See sfuntmpl.m for a general S-function template.
%
%   See also SFUNTMPL.
    
%   Copyright 1990-2009 The MathWorks, Inc.


switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts] = mdlInitializeSizes();

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,                                                
    sys = mdlUpdate(t,x,u); 

  %%%%%%%%%%
  % Output %
  %%%%%%%%%%
  case 3,                                                
    sys = mdlOutputs(t,x);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,                                                
    sys = []; % do nothing

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

%end dsfunc

%
%=======================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=======================================================================
%
function [sys,x0,str,ts] = mdlInitializeSizes()

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 12;
sizes.NumOutputs     = 6; % y
sizes.NumInputs      = 4; % u  
sizes.DirFeedthrough = 0; % D=0
sizes.NumSampleTimes = 1;

sys = simsizes(sizes); %system size, state size
x0  = zeros(12,1);   %initial states
str = [];
ts  = [1e-6 0]; 

% end mdlInitializeSizes

%
%=======================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=======================================================================
%
function sys = mdlUpdate(t,x,u)
g=9.81;
m=0.5;
Iy=5e-3;
Iz=5e-3;
Ix=5e-3;

%Next I will deal with the rotor inputs

%Next I will map the states from the inputs x
%Linear Velocity Components
b=7.2e-5;
l=0.25;
Jm=3.4e-5;
F1=b*(u(1)^2 + u(2)^2 + u(3)^2 + u(4)^2);
F2=b*l*(u(3)^2 - u(4)^2);
F3=b*l*(u(1)^2 - u(2)^2);
F4=b*l*(u(4)^2 +u(3)^2 - u(1)^2 - u(2)^2);

FR= u(3) + u(4) - u(1) - u(2);

U=x(1); %This is effectively U
v=x(2); %V
w=x(3); %W

%Angular Velocity Components
p=x(4); %P
q=x(5); %Q
r=x(6); %R

%Euler Angles
phi=x(7);
theta=x(8);
psi=x(9);
dt=1e-6;

%Linear Positions
X=x(10);
y=x(11);
z=x(12);

sys(1)= U + dt*(((r*v - q*w) + g*sin(theta)));%dU
sys(2)= v + dt*((p*w - r*U - g*cos(theta)*sin(phi)));%dV  
sys(3)= w + dt*((q*U - p*v - g*cos(theta)*cos(phi) + F1/m));%dW
sys(4)= p + dt*(((Iy - Iz)*q*r/Ix + F2/Ix)- (Jm/Ix)*q*FR); %dP
sys(5)= q + dt*(((Iz - Ix)*p*r/Iy + F3/Iy) + (Jm/Ix)*p*FR); %dQ
sys(6)= r + dt*(((Ix - Iy)*p*q/Iz + F4/Iz)); %dR
sys(7)= phi + dt*((p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r));    %dPHI
sys(8)= theta + dt*((cos(phi)*q - sin(phi)*r));  %dTHE 
sys(9)= psi + dt*(((sin(phi)*q)/cos(theta) + (cos(phi)*r)/cos(theta)));    %dPSI
sys(10)= X + dt*(((cos(psi)*cos(theta))*U + ((cos(psi)*sin(theta)*sin(phi))-(sin(psi)*cos(phi)))*v+((cos(psi)*sin(theta)*cos(phi))+(sin(psi)*sin(phi)))*w));      %dX
sys(11)= y + dt*((sin(psi)*cos(theta))*U + ((sin(psi)*sin(theta)*sin(phi))+(cos(psi)*cos(phi)))*v+ ((sin(psi)*sin(theta)*cos(phi)) - (cos(psi)*sin(phi)))*w);      %dY
sys(12)= z + dt*((-sin(theta)*U + cos(theta)*sin(phi)*v + cos(theta)*cos(phi)*w));      %dZ


%end mdlUpdate

%
%=======================================================================
% mdlOutputs
% Return the output vector for the S-function
%=======================================================================
%
function sys = mdlOutputs(t,x)

sys = [ x(7) x(8) x(9) x(10) x(11) x(12) ];

%end mdlOutputs

