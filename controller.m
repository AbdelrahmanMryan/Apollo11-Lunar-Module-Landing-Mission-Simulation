linearization
%% controller design
wc = 0.1;%Cut off frequency
% Attitude Control - Outer Loop
C_phi = pidtune(G_Mx2phi,'pdf',wc);
C_tht = pidtune(G_My2tht,'pdf',wc);
C_psi = pidtune(G_Mz2psi,'pdf',wc);
% Rate Control Inner Loop
C_p = pidtune(G_Mx2p,'p',wc*1.05);
C_q = pidtune(G_My2q,'p',wc*1.05);
C_r = pidtune(G_Mz2r,'p',wc*1.05);
% Velocity Controller
C_Vx = pidtune(-G_Fx2Vx,'p',0.05);
C_Vy = pidtune(-G_Fy2Vy,'p',0.08);
C_Vz = pidtune(G_Fz2Vz,'pi',0.05);
% Altitude Controller
C_Alt = pidtune(G_Fz2Alt,'pdf',1);
% Closed  loop response
% Outer Loop
CL_phi = feedback(C_phi*G_Mx2phi,1);
CL_tht = feedback(C_tht*G_My2tht,1);
CL_psi = feedback(C_psi*G_Mz2psi,1);
% Inner Loop
CL_p = feedback(C_p*G_Mx2p,1);
CL_q = feedback(C_q*G_My2q,1);
CL_r = feedback(C_r*G_Mz2r,1);
% Velocity
CL_Vx = feedback(C_Vx*G_Fx2Vx,1);
CL_Vy = feedback(C_Vy*G_Fy2Vy,1);
CL_Vz = feedback(C_Vz*G_Fz2Vz,1);
% Altitude
CL_Alt = feedback(C_Alt*G_Fz2Alt,1);
% visualize response

% step(CL_tht)
% hold on
% step(CL_psi)
% step(CL_Alt)

