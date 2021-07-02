function [sys,x0,str,ts] = example_sfunction(t,x,u,flag,MWS)

if flag == 1 % dynamics
sys = 0; % state derivative

elseif flag == 3 % output equation
    MWS
sys = 1;

elseif flag == 0 % initialization
sizes                = simsizes;
sizes.NumContStates  = 1;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 1;
sizes.NumInputs      = 0;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
str = [];
ts  = [0 0];
x0 = 0; % initial condition

else % nothing
sys = [] ;
end