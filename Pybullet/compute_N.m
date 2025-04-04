function N = compute_N(in1)
%compute_N
%    N = compute_N(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    25-Mar-2025 20:01:13

q2 = in1(2,:);
q3 = in1(3,:);
q4 = in1(4,:);
q5 = in1(5,:);
q6 = in1(6,:);
t2 = cos(q2);
t3 = cos(q3);
t4 = cos(q4);
t5 = cos(q5);
t6 = cos(q6);
t7 = sin(q2);
t8 = sin(q3);
t9 = sin(q4);
t10 = sin(q5);
t11 = sin(q6);
t12 = t2.*t3.*t5.*t6.*1.543113e-2;
t13 = t2.*t4.*t8.*t11.*1.543113e-2;
t14 = t3.*t4.*t7.*t11.*1.543113e-2;
t15 = t5.*t6.*t7.*t8.*1.543113e-2;
t17 = t2.*t6.*t8.*t9.*t10.*1.543113e-2;
t18 = t3.*t6.*t7.*t9.*t10.*1.543113e-2;
t21 = t2.*t3.*t5.*7.61256e-3;
t22 = t5.*t7.*t8.*7.61256e-3;
t23 = t2.*t3.*1.5906136086;
t24 = t7.*t8.*1.5906136086;
t27 = t2.*t8.*t9.*t10.*7.61256e-3;
t28 = t3.*t7.*t9.*t10.*7.61256e-3;
t16 = -t12;
t19 = -t17;
t20 = -t18;
t25 = -t21;
t26 = -t23;
t29 = -t27;
t30 = -t28;
mt1 = [0.0;t2.*(-4.03810992)+t13+t14+t15+t16+t19+t20+t22+t24+t25+t26+t29+t30;t13+t14+t15+t16+t19+t20+t22+t24+t25+t26+t29+t30;(cos(q2+q3).*(t4.*t10.*8.570980594833886e+20+t9.*t11.*1.737390782947642e+21+t4.*t6.*t10.*1.737390782947642e+21))./1.125899906842624e+23];
mt2 = [((t6.*1.737390782947642e+21+8.570980594833886e+20).*(t2.*t8.*t10+t3.*t7.*t10+t2.*t3.*t5.*t9-t5.*t7.*t8.*t9))./1.125899906842624e+23];
mt3 = [t2.*t3.*t4.*t6.*(-1.543113e-2)+t4.*t6.*t7.*t8.*1.543113e-2+t2.*t5.*t8.*t11.*1.543113e-2+t3.*t5.*t7.*t11.*1.543113e-2-t2.*t3.*t9.*t10.*t11.*1.543113e-2+t7.*t8.*t9.*t10.*t11.*1.543113e-2];
N = [mt1;mt2;mt3];
end
