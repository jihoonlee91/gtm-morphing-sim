function plot_result(simout)

% time
t = simout.surf.ELLOB.Time;

% surf
ELLOB = simout.surf.ELLOB.Data;
ELLIB = simout.surf.ELLIB.Data;
ELROB = simout.surf.ELROB.Data;
ELRIB = simout.surf.ELRIB.Data;
AILL = simout.surf.AILL.Data;
AILR = simout.surf.AILR.Data;
RUDU = simout.surf.RUDU.Data;
RUDL = simout.surf.RUDL.Data;
THROTL = simout.surf.THROTL.Data;
THROTR = simout.surf.THROTR.Data;

% aux
gamma = simout.aux.gamma.Data;
alpha = simout.aux.alpha.Data;
beta = simout.aux.beta.Data;
tas = simout.aux.tas.Data;
Vn = simout.aux.Vn.Data;
Ve = simout.aux.Ve.Data;
Vd = simout.aux.Vd.Data;
Ax = simout.aux.Ax.Data;
Ay = simout.aux.Ay.Data;
Az = simout.aux.Az.Data;

% eom
ub = simout.eom.ub.Data;
vb = simout.eom.vb.Data;
wb = simout.eom.wb.Data;
pb = simout.eom.pb.Data;
qb = simout.eom.qb.Data;
rb = simout.eom.rb.Data;
latitude = simout.eom.latitude.Data;
longitude = simout.eom.longitude.Data;
altitude = simout.eom.altitude.Data;
phi = simout.eom.phi.Data;
theta = simout.eom.theta.Data;
psi = simout.eom.psi.Data;

%%
r2d = 180/pi;

figure(1)
plot(t,phi*r2d,t,theta*r2d,t,psi*r2d)
legend('\phi','\theta','\psi')
xlabel('Time [s]')
ylabel('Euler Angle [deg]')
grid on
grid minor

figure(2)
plot(t,pb*r2d,t,qb*r2d,t,rb*r2d)
legend('p','q','r')
xlabel('Time [s]')
ylabel('Angular Velocity [deg/s]')
grid on
grid minor

figure(3)
plot(t,alpha,t,beta,t,gamma)
legend('\alpha','\beta','\gamma')
xlabel('Time [s]')
ylabel('[deg]')
grid on
grid minor
