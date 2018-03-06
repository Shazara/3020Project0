function [sys,x0,str,ts] = continuous_func_V2(t,x,U,flag,Ix,Iy,Iz,m)
%CSFUNC An example MATLAB file S-function for defining a continuous system.  
%   Example MATLAB file S-function implementing continuous equations: 
%      x' = Ax + Bu
%      y  = Cx + Du
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
    [sys,x0,str,ts]=mdlInitializeSizes();

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,U,Ix,Iy,Iz,m);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x);

  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  case { 2, 4, 9 },
    sys = [];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end
% end csfunc

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes()

sizes = simsizes;
sizes.NumContStates  = 12;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
%x0=[U V W P Q R Phi The Psi X Y Z]
x0  = [0 0 0 0 0 0 0 0 0 0 0 0];
str = [];
ts  = [0 0];

% end mdlInitializeSizes
%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,U,Ix,Iy,Iz,m)

g=9.81;

%Next I will deal with the rotor inputs

%Next I will map the states from the inputs x
%Linear Velocity Components
u=x(1); %This is effectively U
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

%Linear Positions
X=x(10);
y=x(11);
z=x(12);


%Next we will have the dynamic state equations
sys(1)=((r*v - q*w) + g*sin(theta));%dU
sys(2)=(p*w - r*u - g*cos(theta)*sin(phi));%dV  
sys(3)=(q*u - p*v - g*cos(theta)*cos(phi) + U(1)/m);%dW
sys(4)=((Iy - Iz)*q*r/Ix + U(2)/Ix); %dP
sys(5)=((Iz - Ix)*p*r/Iy + U(3)/Iy); %dQ
sys(6)=((Ix - Iy)*p*q/Iz + U(4)/Iz); %dR
sys(7)=(p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r);    %dPHI
sys(8)=(cos(phi)*q - sin(phi)*r);  %dTHE 
sys(9)=((sin(phi)*q)/cos(theta) + (cos(phi)*r)/cos(theta));    %dPSI
sys(10)=((cos(psi)*cos(theta))*u + ((cos(psi)*sin(theta)*sin(phi))-(sin(psi)*cos(phi)))*v+((cos(psi)*sin(theta)*cos(phi))+(sin(psi)*sin(phi)))*w);      %dX
sys(11)=(sin(psi)*cos(theta))*u + ((sin(psi)*sin(theta)*sin(phi))+(cos(psi)*cos(phi)))*v+ ((sin(psi)*sin(theta)*cos(phi)) - (cos(psi)*sin(phi)))*w;      %dY
sys(12)=(-sin(theta)*u + cos(theta)*sin(phi)*v + cos(theta)*cos(phi)*w);      %dZ
% Rough rule to impose a "ground" boundary...could easily be improved...
if ((z<=0) && (w<=0)) % better  version then before?
    sys(12) = 0;
end
% 
% %Next we will have the dynamic state equations
% sys(1)=((r*v - q*w) + g*sin(theta));%dU
% sys(2)=(p*w - r*u - g*cos(theta)*sin(phi));%dV  
% sys(3)=(q*u - p*v - g*cos(theta)*cos(phi) + U(1)/m);%dW
% sys(4)=((Iy - Iz)*q*r/Ix + U(2)/Ix); %dP
% sys(5)=((Iz - Ix)*p*r/Iy + U(3)/Iy); %dQ
% sys(6)=((Ix - Iy)*p*q/Iz + U(4)/Iz); %dR
% sys(7)=p;    %dPHI
% sys(8)=q;  %dTHE 
% sys(9)=r;    %dPSI
% sys(10)=u;      %dX
% sys(11)=v;      %dY
% sys(12)=w;      %dZ

% end mdlDerivatives
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x)

sys = x';

% end mdlOutputs
