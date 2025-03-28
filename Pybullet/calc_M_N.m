
% computeM_N Computes the mass matrix M and gravity term N as symbolic functions
%            of the joint vector q.
%
%   q should be a 6x1 vector:
%      

% Define symbolic variables for the joints
syms q1 q2 q3 q4 q5 q6 dq1 dq2 dq3 dq4 dq5 dq6 t1 t2 t3 t4 t5 t6 real

% Map the input vector q to the individual joint symbols
q = [q1; q2; q3; q4; q5; q6];
dq = [dq1;dq2;dq3;dq4;dq5;dq6];
tau = [t1;t2;t3;t4;t5;t6];
x = [q;dq];
% Gravity and fixed transforms
g = 9.81;
Rws = eye(3);
Tws = [1 0 0 0; 0 1 0 0; 0 0 1 1; 0 0 0 1];

% Base transform
pb = [0;0;0];
Rs = eye(3); 
Ts = eye(4);
pb_com = [0;0;-0.0170];
Ts_com = [1 0 0 0; 0 1 0 0; 0 0 1 -0.0170; 0 0 0 1];
Ts_com = Ts * Ts_com;
Twb_com = Tws * Ts_com;

%% Joint 1
disp('j1');
psb = [0;0;0];
% getR_zT_z should return the rotation and transformation matrix for a z-axis rotation.
[Rsb, Tsb] = getR_zT_z(q1, psb);
p1_com = [0;0;-0.0739];
Tb1_com = [1 0 0 0; 0 1 0 0; 0 0 1 -0.0739; 0 0 0 1];
Ts1_com = Tsb * Tb1_com;
Ts1_com = simplify(Ts1_com);
Twb = Tws * Tsb;
Tw1_com = Tws * Ts1_com;

%% Joint 2
disp('j2');
pb1 = [0;0;-0.1478];
% getR_yT_y returns the rotation and transform for a y-axis rotation.
[Rb1, Tb1] = getR_yT_y(q2, pb1);
p2_com = [0.13;0;0];
T12_com = [1 0 0 0.13; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Ts2_com = Tsb * Tb1 * T12_com;
Ts2_com = simplify(Ts2_com);
Ts1 = Tsb * Tb1;
Tw1 = Tws * Ts1;
Tw2_com = Tws * Ts2_com;

%% Joint 3
disp('j3');
p12 = [0.26;0;0];
% getR_yT_y for joint 3:
[R12, T12] = getR_yT_y(q3, p12);
p3_com = [0.13;0;0];
T23_com = [1 0 0 0.13; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Ts2 = Ts1 * T12;
Ts3_com = Tsb * Tb1 * T12 * T23_com;
Ts3_com = simplify(Ts3_com);
Tw2 = Tw1 * T12;
Tw3_com = Tws * Ts3_com;

%% Link 4
disp('j4');
p23 = [0.26; 0; 0];  % Note: p23 is given as [0.26; 0; 0]
% getR_xT_x returns the rotation and transform for an x-axis rotation.
[R23, T23] = getR_xT_x(q4, p23);
p4_com = [0.0349;0;0];
T34_com = [1 0 0 0.0349; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Ts3 = Ts2 * T23;
Ts4_com = Tsb * Tb1 * T12 * T23 * T34_com;
Ts4_com = simplify(Ts4_com);
Tw3 = Tw2 * T23;
Tw4_com = Tws * Ts4_com;

%% Link 5
disp('j5');
p34 = [0.0698;0;0];
% getR_zT_z for link 5:
[R34, T34] = getR_zT_z(q5, p34);
p5_com = [0.005;0;0];
T45_com = [1 0 0 0.005; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Ts4 = Ts3 * T34;
Ts5_com = Tsb * Tb1 * T12 * T23 * T34 * T45_com;
Ts5_com = simplify(Ts5_com);
Tw4 = Tw3 * T34;
Tw5_com = Tws * Ts5_com;

%% Link 6
disp('j6');
p45 = [0.01;0;0];
% getR_yT_y for link 6:
[R45, T45] = getR_yT_y(q6, p45);
p6_com = [0.013;0;0];
T56_com = [1 0 0 0.013; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Ts6_com = Tsb * Tb1 * T12 * T23 * T34 * T45 * T56_com;
Ts6_com = simplify(Ts6_com);
Ts5 = Ts4 * T45;
Tw5 = Tw4 * T45;
Tw6_com = Tws * Ts6_com;

%% End Effector
disp('EE');
p5E = [0.026;0;0];
T5e = [eye(3), p5E; zeros(1,3), 1];
Tse = Ts5 * T5e;
Twe = Tw5 * T5e;

%% Mass Matrices
disp('mass setup');
M_Base = diag([0.85, 0.85, 0.85, 0.00226, 0.00226, 0.0041388]);
M_l1   = diag([0.49, 0.49, 0.49, 0.000932, 0.00104, 0.000185]);
M_l2   = diag([0.99, 0.99, 0.99, 0.000375, 0.00747, 0.00725]);
M_l3   = diag([0.98, 0.98, 0.98, 0.000375, 0.00738, 0.00715]);
M_l4   = diag([0.027, 0.027, 0.027, 4.322e-6, 4.317e-6, 2.702e-6]);
M_l5   = diag([0.0072, 0.0072, 0.0072, 4.807e-7, 5.229e-7, 5.229e-7]);
M_l6   = diag([0.027, 0.027, 0.027, 4.322e-6, 2.702e-6, 4.317e-6]);
M_EE   = diag([0.047, 0.047, 0.047, 1.626e-5, 1.015e-5, 1.015e-6]);

%% Jacobians
disp('jacobi ans');
disp('1');
Ts1_com_inv = inv(Ts1_com);
disp('2');
Ts2_com_inv = inv(Ts2_com);
disp('3');
Ts3_com_inv = inv(Ts3_com);
disp('4');
Ts4_com_inv = inv(Ts4_com);
disp('5');
Ts5_com_inv = inv(Ts5_com);
disp('6');
Ts6_com_inv = inv(Ts6_com);
disp('7');
Tse_inv = inv(Tse);
disp('Ts6_com:');
disp(Ts6_com);
disp('Ts6_com_inv:');
disp(Ts6_com_inv);

% Compute Jacobian blocks using the helper function g2twist.
% It is assumed that g2twist takes a 4x4 matrix Vhat and returns a 6x1 twist vector.
Jb_sl1 = [ g2twist(Ts1_com_inv * diff(Ts1_com, q1)), ...
           g2twist(Ts1_com_inv * diff(Ts1_com, q2)), ...
           g2twist(Ts1_com_inv * diff(Ts1_com, q3)), ...
           g2twist(Ts1_com_inv * diff(Ts1_com, q4)), ...
           g2twist(Ts1_com_inv * diff(Ts1_com, q5)), ...
           g2twist(Ts1_com_inv * diff(Ts1_com, q6)) ];
disp('1 computed');

Jb_sl2 = [ g2twist(Ts2_com_inv * diff(Ts2_com, q1)), ...
           g2twist(Ts2_com_inv * diff(Ts2_com, q2)), ...
           g2twist(Ts2_com_inv * diff(Ts2_com, q3)), ...
           g2twist(Ts2_com_inv * diff(Ts2_com, q4)), ...
           g2twist(Ts2_com_inv * diff(Ts2_com, q5)), ...
           g2twist(Ts2_com_inv * diff(Ts2_com, q6)) ];
disp('2 computed');

Jb_sl3 = [ g2twist(Ts3_com_inv * diff(Ts3_com, q1)), ...
           g2twist(Ts3_com_inv * diff(Ts3_com, q2)), ...
           g2twist(Ts3_com_inv * diff(Ts3_com, q3)), ...
           g2twist(Ts3_com_inv * diff(Ts3_com, q4)), ...
           g2twist(Ts3_com_inv * diff(Ts3_com, q5)), ...
           g2twist(Ts3_com_inv * diff(Ts3_com, q6)) ];
disp('3 computed');

Jb_sl4 = [ g2twist(Ts4_com_inv * diff(Ts4_com, q1)), ...
           g2twist(Ts4_com_inv * diff(Ts4_com, q2)), ...
           g2twist(Ts4_com_inv * diff(Ts4_com, q3)), ...
           g2twist(Ts4_com_inv * diff(Ts4_com, q4)), ...
           g2twist(Ts4_com_inv * diff(Ts4_com, q5)), ...
           g2twist(Ts4_com_inv * diff(Ts4_com, q6)) ];
disp('4 computed');

Jb_sl5 = [ g2twist(Ts5_com_inv * diff(Ts5_com, q1)), ...
           g2twist(Ts5_com_inv * diff(Ts5_com, q2)), ...
           g2twist(Ts5_com_inv * diff(Ts5_com, q3)), ...
           g2twist(Ts5_com_inv * diff(Ts5_com, q4)), ...
           g2twist(Ts5_com_inv * diff(Ts5_com, q5)), ...
           g2twist(Ts5_com_inv * diff(Ts5_com, q6)) ];
disp('5 computed');

Jb_sl6 = [ g2twist(Ts6_com_inv * diff(Ts6_com, q1)), ...
           g2twist(Ts6_com_inv * diff(Ts6_com, q2)), ...
           g2twist(Ts6_com_inv * diff(Ts6_com, q3)), ...
           g2twist(Ts6_com_inv * diff(Ts6_com, q4)), ...
           g2twist(Ts6_com_inv * diff(Ts6_com, q5)), ...
           g2twist(Ts6_com_inv * diff(Ts6_com, q6)) ];
disp('6 computed');

Jbse = [ g2twist(Tse_inv * diff(Tse, q1)), ...
         g2twist(Tse_inv * diff(Tse, q2)), ...
         g2twist(Tse_inv * diff(Tse, q3)), ...
         g2twist(Tse_inv * diff(Tse, q4)), ...
         g2twist(Tse_inv * diff(Tse, q5)), ...
         g2twist(Tse_inv * diff(Tse, q6)) ];
disp('7 computed');

%% Final Mass Matrix and Gravity Term
disp('finals');
M_fin = Jb_sl1.' * M_l1 * Jb_sl1 + ...
        Jb_sl2.' * M_l2 * Jb_sl2 + ...
        Jb_sl3.' * M_l3 * Jb_sl3 + ...
        Jb_sl4.' * M_l4 * Jb_sl4 + ...
        Jb_sl5.' * M_l5 * Jb_sl5 + ...
        Jb_sl6.' * M_l6 * Jb_sl6 + ...
        Jbse.'   * M_EE * Jbse   + M_Base;
disp('M computed');

% Note: Adjust indices as MATLAB uses 1-based indexing.
N_fin = M_Base(1,1)*g*pb_com(3) + M_l1(1,1)*g*Ts1_com(3,4) + ...
        M_l2(1,1)*g*Ts2_com(3,4) + M_l3(1,1)*g*Ts3_com(3,4) + ...
        M_l4(1,1)*g*Ts4_com(3,4) + M_l5(1,1)*g*Ts5_com(3,4) + ...
        M_l6(1,1)*g*Ts6_com(3,4) + M_EE(1,1)*g*Tse(3,4);
disp('N Computed');
%% Final step
M = simplify(M_fin);
N = simplify(gradient(N_fin, q));  
C_fin = coriolis(M, q, dq);
C = simplify(C_fin);
% matlabFunction(C, 'File','compute_C', 'Vars', {x});
% matlabFunction(M, 'File','compute_M', 'Vars', {x});
% matlabFunction(N, 'File','compute_N', 'Vars', {x});
%% Get StateSpace Matrices?
ddq = M\(tau - C*dq - N);
