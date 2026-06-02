% =========================================================================
% TITLE: Pitch rate SAS design
%
% REFERENCES:
% The design is adapted from / based on:
% Book: Aircraft Control and Simulation: Dynamics, Controls Design, and Autonomous Systems
% Authors: Brian L. Stevens, Frank L. Lewis, and Eric N. Johnson
% Edition: 3rd Edition (2015)
% =========================================================================

% Matrices need to be obtained from aircraft simulation i.e JSBSim FDM

% Trim and Linearization @ XXXX ft and XXXX KTS
A_ac_long = A_ac(1:4,1:4);
B_ac_long = B_ac(1:4,[1,3]);     % [throttle, elevator]
C_ac_long = [0 0 1 0; 0 0 0 1];  % outputs [theta, q]
D_ac_long = zeros(2,1);
sys_long  = ss(A_ac_long,B_ac_long(:,2),C_ac_long,D_ac_long);

% Actuator Dynamics
aa = -10; ba = 10;
ca = -1; da = 0; % with ca=-1,sign change for plant (a/c negative tf)
actua = ss(aa,ba,ca,da);

% Actuator + Plant
syst_with_actua = series(actua,sys_long); 
[a,b,c,d] = ssdata(syst_with_actua);
% root locus - Tuning  k_q;
rlocus(a,b,c(2,:),0)
% Closed-loop with pitch SAS
k_q = 1; % default value
acl = a - b*[0  k_q]*c; 
% Augmented system with pitch SAS
q_closed = ss(acl,b,c(1,:),0);

% Simulate doublet elevator test
t = 0:0.02:200;
u = [0.1*ones(1,51),-0.1*ones(1,50),zeros(1,9900)]';
[y,t,x] = lsim(q_closed,u,t);
figure(1)
plot(t,y)