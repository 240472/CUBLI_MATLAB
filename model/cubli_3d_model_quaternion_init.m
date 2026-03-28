

qs_init = [sqrt(2/3*(3-sqrt(3)))/2, -sqrt(1/3*(3+sqrt(3)))/2, sqrt(1/3*(3+sqrt(3)))/2, 0]';
qu_init = [sqrt(2/3*(3+sqrt(3)))/2, sqrt(1/3*(3-sqrt(3)))/2, -sqrt(1/3*(3-sqrt(3)))/2, 0]';

ms = 0.4;
mw = 0.15;
l = 0.15;
Is_xx = 2e-3;
Iw_xx = 1.25e-4;
Iw_yy = 4e-5;

tauc = 2.46e-3;
bw = 1.06e-5;
cd = 1.7e-8;

b = 0;

Is_G = [Is_xx 0 0; 0 Is_xx 0; 0 0 Is_xx];
rs = [l/2; l/2; l/2];

Iw1_G = [Iw_xx 0 0; 0 Iw_yy 0; 0 0 Iw_yy];
Iw2_G = [Iw_yy 0 0; 0 Iw_xx 0; 0 0 Iw_yy];
Iw3_G = [Iw_yy 0 0; 0 Iw_yy 0; 0 0 Iw_xx];

rw1 = [0; l/2; l/2];
rw2 = [l/2; 0; l/2];
rw3 = [l/2; l/2; 0];

rs_skew = [0 -rs(3) rs(2); rs(3) 0 -rs(1); -rs(2) rs(1) 0];
rw1_skew = [0 -rw1(3) rw1(2); rw1(3) 0 -rw1(1); -rw1(2) rw1(1) 0];
rw2_skew = [0 -rw2(3) rw2(2); rw2(3) 0 -rw2(1); -rw2(2) rw2(1) 0];
rw3_skew = [0 -rw3(3) rw3(2); rw3(3) 0 -rw3(1); -rw3(2) rw3(1) 0];

Is_O = Is_G + ms*(rs_skew*rs_skew');
Iw1_O = Iw1_G + mw*(rw1_skew*rw1_skew');
Iw2_O = Iw2_G + mw*(rw2_skew*rw2_skew');
Iw3_O = Iw3_G + mw*(rw3_skew*rw3_skew');

Iw = [Iw_xx 0 0; 0 Iw_xx 0; 0 0 Iw_xx];
Ic_line = (Is_O + Iw1_O + Iw2_O + Iw3_O) - Iw;

g = [0;0;9.81];

mc_line = ms + 2*mw;

invV = [0 0 0 1; 0 0 1 0; 0 -1 0 0; -1 0 0 0];
GAMMA = [1 1 -1 0; 1 -1 0 1; -1 0 -1 1; 0 1 1 1];

qu_skew = [0 -qu_init(4) qu_init(3); qu_init(4) 0 -qu_init(2); -qu_init(3) qu_init(2) 0];
G_qu = [-qu_init(2:4) qu_init(1)*eye(3,3)-qu_skew];

GAMMA_qu = GAMMA*qu_init;

GAMMA_skew = [0 -GAMMA_qu(4) GAMMA_qu(3); GAMMA_qu(4) 0 -GAMMA_qu(2); -GAMMA_qu(3) GAMMA_qu(2) 0];
G_GAMMA = [-GAMMA_qu(2:4) GAMMA_qu(1)*eye(3,3)-GAMMA_skew];

F = bw*eye(3,3);
B = b*eye(3,3);
K = mc_line*9.81*l*(G_qu*GAMMA-G_GAMMA);




A = [zeros(4,4) zeros(4,3) (1/2)*G_qu' zeros(4,3);
    zeros(3,4) zeros(3,3) zeros(3,3) eye(3,3);
    inv(Ic_line)*K zeros(3,3) -inv(Ic_line)*B inv(Ic_line)*F;
    zeros(3,4) zeros(3,3) zeros(3,3) -inv(Iw)*F];

B = [zeros(4,3); zeros(3,3); inv(Ic_line); inv(Iw)];