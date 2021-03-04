% load nominal starting point into simulation model workspace
loadmws(init_design('GTM_T2'));

% Trim to nominal condition
[MWS,Xt,Tc,Err]=trimgtm(struct('tas',100, 'gamma',0));

% Load Simulation Variables (at trim condition) into Model Workspace
loadmws(MWS);

% Construct same doublet sequence via simulink
set_param('gtm_design/Input Generator/Doublet Generator','timeon','[2 12 7]'); 
set_param('gtm_design/Input Generator/Doublet Generator','pulsewidth','[0.5 0.5 0.5]');
set_param('gtm_design/Input Generator/Doublet Generator','amplitude',sprintf('[%f %f %f]',a(1),a(3),a(2)));
[tsim,xsim,ysim]=sim('gtm_design',[0 15]);

% Turn simulink doublet generation off
set_param('gtm_design/Input Generator/Doublet Generator','amplitude','[0 0 0]');