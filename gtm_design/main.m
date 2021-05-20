% load nominal starting point into simulation model workspace
loadmws(init_design('GTM_T2'));

% Trim to nominal condition
[MWS,Xt,Tc,Err]=trimgtm(struct('tas',100, 'gamma',0));

% Load Simulation Variables (at trim condition) into Model Workspace
loadmws(MWS);

[tsim,xsim,ysim]=sim('gtm_design',[0 30]);

plot_result(simout)