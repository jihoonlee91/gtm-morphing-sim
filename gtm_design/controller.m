function [sys,x0,str,ts] = controller(t,x,u,flag)

if flag == 1 % dynamics
% Inputs
AirData = u(1:6);
MIDG = u(7:15);
MAG3 = u(16:21);
SurfacePos = u(20:33);
Engines = u(34:39);

%
alpha_deg = AirData(1);
beta_deg = AirData(1);
qbar_psf = AirData(1);
KTAS = AirData(1);
TEMPAMB_degF = AirData(1);
altitude_ft = AirData(1);
phi_deg = MIDG(1);
theta_deg = MIDG(2);
psi_deg = MIDG(3);
NAV_latitude_deg = MIDG(4);
NAV_longitude_deg = MIDG(5);
NAV_altitude_deg = MIDG(6);
NAV_Vn_mps = MIDG(7);
NAV_Ve_mps = MIDG(8);
NAV_Vd_mps = MIDG(9);
p_deg = MAG3(1);
q_deg = MAG3(2);
r_deg = MAG3(3);
Ax_g = MAG3(4);
Ay_g = MAG3(5);
Az_g = MAG3(6);
ELLOB = SurfacePos(1);
ELLIB = SurfacePos(2);
ELROB = SurfacePos(3);
ELRIB = SurfacePos(4);
AILL = SurfacePos(5);
AILR = SurfacePos(6);
RUDU = SurfacePos(7);
RUDL = SurfacePos(8);
FLAPLIB = SurfacePos(9);
FLAPLOB = SurfacePos(10);
FLAPRIB = SurfacePos(11);
FLAPROB = SurfacePos(12);
RPML = Engines(1);
PUMPVL = Engines(2);
RPMR = Engines(3);
PUMPVR = Engines(4);

%
mph2mps = 2.23694;
deg2rad = pi/180;
ft2m = 0.3048;
knots2mps = 0.514444;

% States
NAV_Vn = NAV_Vn_mph*mph2mps;
NAV_Ve = NAV_Ve_mph*mph2mps;
NAV_Vd = NAV_Vd_mph*mph2mps;
TAS = KTAS*knots2mps;
alpha = alpha_deg*deg2rad;
beta = beta_deg*deg2rad;
p = p_deg*deg2rad;
q = q_deg*deg2rad;
r = r_deg*deg2rad;
phi = phi_deg*deg2rad;
mu = phi;
theta = theta_deg*deg2rad;
psi = psi_deg*deg2rad;
altitude = altitude_ft*ft2m;

% Flight path angles (no wind assumption)
chi   = atan2(NAV_Ve,NAV_Vn); % HFPA [rad]
gamma = asin(-NAV_Vd/TAS); % VFPA [rad]

% State vectors
x1 = [TAS;chi;gamma];
x2 = [mu;alpha;beta];
x3 = [p;q;r];

%
e1 = x1_ref - x1;
e2 = x2_ref - x2;
e3 = x3_ref - x3;

sys = [e1;e2;e3]; % state derivative

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

elseif flag == 3 % output equation
% Inputs
AirData = u(1:6);
MIDG = u(7:15);
MAG3 = u(16:21);
SurfacePos = u(20:33);
Engines = u(34:39);

%
alpha_deg = AirData(1);
beta_deg = AirData(1);
qbar_psf = AirData(1);
KTAS = AirData(1);
TEMPAMB_degF = AirData(1);
altitude_ft = AirData(1);
phi_deg = MIDG(1);
theta_deg = MIDG(2);
psi_deg = MIDG(3);
NAV_latitude_deg = MIDG(4);
NAV_longitude_deg = MIDG(5);
NAV_altitude_deg = MIDG(6);
NAV_Vn_mps = MIDG(7);
NAV_Ve_mps = MIDG(8);
NAV_Vd_mps = MIDG(9);
p_deg = MAG3(1);
q_deg = MAG3(2);
r_deg = MAG3(3);
Ax_g = MAG3(4);
Ay_g = MAG3(5);
Az_g = MAG3(6);
ELLOB = SurfacePos(1);
ELLIB = SurfacePos(2);
ELROB = SurfacePos(3);
ELRIB = SurfacePos(4);
AILL = SurfacePos(5);
AILR = SurfacePos(6);
RUDU = SurfacePos(7);
RUDL = SurfacePos(8);
FLAPLIB = SurfacePos(9);
FLAPLOB = SurfacePos(10);
FLAPRIB = SurfacePos(11);
FLAPROB = SurfacePos(12);
RPML = Engines(1);
PUMPVL = Engines(2);
RPMR = Engines(3);
PUMPVR = Engines(4);

%
mph2mps = 2.23694;
deg2rad = pi/180;
ft2m = 0.3048;
knots2mps = 0.514444;

% States
NAV_Vn = NAV_Vn_mph*mph2mps;
NAV_Ve = NAV_Ve_mph*mph2mps;
NAV_Vd = NAV_Vd_mph*mph2mps;
TAS = KTAS*knots2mps;
alpha = alpha_deg*deg2rad;
beta = beta_deg*deg2rad;
p = p_deg*deg2rad;
q = q_deg*deg2rad;
r = r_deg*deg2rad;
phi = phi_deg*deg2rad;
mu = phi;
theta = theta_deg*deg2rad;
psi = psi_deg*deg2rad;
altitude = altitude_ft*ft2m;

% Flight path angles (no wind assumption)
chi   = atan2(NAV_Ve,NAV_Vn); % HFPA [rad]
gamma = asin(-NAV_Vd/TAS); % VFPA [rad]

% State vectors
x1 = [TAS;chi;gamma];
x2 = [mu;alpha;beta];
x3 = [p;q;r];

%
e2 = x2_ref - x2;
e3 = x3_ref - x3;

%
e2_int = x(4:6);
e3_int = x(7:9);

e1_prev = u(40:42);
e2_prev = u(43:45);
e3_prev = u(46:48);

Timestep = 1/200;
e2_dot = (e2-e2_prev)/Timestep;
e3_dot = (e3-e3_prev)/Timestep;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Velocity loop PID gains
KP1 = 3*eye(3)*1e-1;
KI1 = eye(3)*1e-4;
KD1 = eye(3)*1e-5;

% Attitude loop PID gains
KP2 = 3*eye(3)*1e0;
KI2 = eye(3)*1e-1;
KD2 = eye(3)*1e-3;

% Angular velocity loop PID gains
KP3 = eye(3)*1e1;
KI3 = eye(3)*1e0;
KD3 = eye(3)*1e-2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

e1 = x1_ref - x1;
e1_int = x(1:3);
e1_dot = (e1-e1_prev)/Timestep;
x1_ref_dot = KP1*e1+KI1*e1_int+KD1*e1_dot;

TAS_ref_dot = 


x2_ref_dot = KP2*e2+KI2*e2_int+KD2*e2_dot;
x3_ref_dot = KP3*e3+KI3*e3_int+KD3*e3_dot;


T_req = 


sys = zeros(19,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

elseif flag == 0 % initialization
sizes                = simsizes;
sizes.NumContStates  = 9;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 19;
sizes.NumInputs      = (6+9+6+12+6)+9;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
str = [];
ts  = [0 0];
x0 = zeros(10,1); % initial condition

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

else % nothing
sys = [ ] ;
end