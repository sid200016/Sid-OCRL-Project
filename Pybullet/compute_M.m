function M = compute_M(in1)
%compute_M
%    M = compute_M(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    25-Mar-2025 20:01:12

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
t12 = t2.^2;
t13 = t3.^2;
t14 = t4.^2;
t15 = t5.^2;
t16 = t6.^2;
t17 = t6.*3.146e-5;
t18 = t3.*t5.*2.0176e-4;
t19 = t8.*t10.*2.0176e-4;
t23 = t6.*t9.*(-3.146e-5);
t24 = t5.*5.118496e-4;
t25 = t2.*t4.*t5.*2.0176e-4;
t26 = t3.*t4.*t10.*2.0176e-4;
t28 = t2.*t9.*t10.*2.0176e-4;
t29 = t4.*t7.*t10.*2.0176e-4;
t33 = t3.*t5.*t9.*(-2.0176e-4);
t37 = t4.*t10.*t11.*1.573e-5;
t38 = t5.*t9.*t11.*1.573e-5;
t39 = t7.*t9.*t11.*4.0898e-4;
t41 = t3.*t4.*t6.*4.0898e-4;
t42 = t3.*t5.*t6.*4.0898e-4;
t43 = t4.*t5.*t6.*1.573e-5;
t44 = t2.*t6.*t8.*(-3.146e-5);
t45 = t3.*t6.*t7.*(-3.146e-5);
t46 = t2.*t4.*t11.*4.0898e-4;
t47 = t2.*t6.*t9.*4.0898e-4;
t48 = t3.*t9.*t11.*4.0898e-4;
t49 = t4.*t8.*t11.*4.0898e-4;
t50 = t5.*t8.*t11.*4.0898e-4;
t51 = t6.*t8.*t10.*4.0898e-4;
t53 = t4.*t10.*2.559248e-4;
t54 = t5.*t9.*2.559248e-4;
t58 = t4.*t9.*t10.*t11.*3.146e-5;
t63 = t4.*t6.*5.187754e-4;
t66 = t9.*t11.*5.187754e-4;
t67 = t5.*t6.*1.0375508e-3;
t68 = t2.*t4.*t5.*t6.*4.0898e-4;
t69 = t4.*t5.*t6.*t10.*(-3.146e-5);
t72 = t3.*4.21569356e-2;
t73 = t4.*t6.*t7.*t8.*(-3.146e-5);
t76 = t4.*t6.*t7.*t10.*4.0898e-4;
t77 = t2.*t3.*t10.*t11.*1.573e-5;
t78 = t2.*t5.*t8.*t11.*1.573e-5;
t79 = t2.*t6.*t8.*t10.*1.573e-5;
t80 = t3.*t5.*t7.*t11.*1.573e-5;
t81 = t3.*t6.*t7.*t10.*1.573e-5;
t85 = t7.*t8.*t10.*t11.*1.573e-5;
t92 = t3.*t4.*t6.*t10.*(-4.0898e-4);
t93 = t3.*t5.*t6.*t9.*(-4.0898e-4);
t94 = t2.*t4.*t10.*t11.*(-4.0898e-4);
t95 = t2.*t6.*t9.*t10.*(-4.0898e-4);
t100 = t5.*t6.*t9.*5.187754e-4;
t102 = t6.*1.767952721791663e+21;
t104 = t2.*t3.*t4.*t5.*t11.*1.573e-5;
t105 = t2.*t3.*t5.*t6.*t9.*1.573e-5;
t109 = t4.*t5.*t7.*t8.*t11.*1.573e-5;
t110 = t5.*t6.*t7.*t8.*t9.*1.573e-5;
t113 = t7.*t8.*t9.*t10.*2.559248e-4;
t114 = t4.*t6.*t10.*(-5.187754e-4);
t117 = t6.*1.450836421397256e+22;
t124 = t2.*t3.*t4.*t5.*2.559248e-4;
t125 = t7.*t8.*t9.*t10.*t11.*(-1.573e-5);
t126 = t2.*t3.*t9.*t10.*2.559248e-4;
t129 = t4.*t5.*t7.*t8.*2.559248e-4;
t132 = t4.*t7.*t8.*t11.*5.187754e-4;
t133 = t6.*t7.*t8.*t9.*5.187754e-4;
t136 = t2.*t3.*t4.*t11.*5.187754e-4;
t137 = t2.*t3.*t6.*t9.*5.187754e-4;
t153 = t2.*t3.*t4.*t10.*t11.*(-5.187754e-4);
t154 = t2.*t3.*t6.*t9.*t10.*(-5.187754e-4);
t155 = t4.*t5.*t6.*t7.*t8.*(-5.187754e-4);
t158 = t9.*2.86849e-5;
t162 = t4.*t5.*4.9187e-5;
t164 = t2.*t8.*6.16119e-5;
t165 = t3.*t7.*6.16119e-5;
t166 = t4.*t5.*t10.*1.51422e-5;
t170 = t2.*t8.*t10.*4.9187e-5;
t171 = t3.*t7.*t10.*4.9187e-5;
t173 = t4.*t7.*t8.*2.86849e-5;
t176 = t2.*t3.*t4.*2.86849e-5;
t178 = t2.*t3.*t5.*t9.*4.9187e-5;
t181 = t5.*t7.*t8.*t9.*4.9187e-5;
t182 = t2.*t3.*t4.*t9.*1.50778e-5;
t183 = t4.*t7.*t8.*t9.*1.50778e-5;
t186 = t4.*t6.*t10.*t11.*2.1085e-5;
t187 = t5.*t6.*t9.*t11.*2.1085e-5;
t190 = t4.*t6.*t9.*t10.*t11.*4.217e-5;
t196 = t2.*t3.*t6.*t10.*t11.*2.1085e-5;
t197 = t2.*t5.*t6.*t8.*t11.*2.1085e-5;
t198 = t3.*t5.*t6.*t7.*t11.*2.1085e-5;
t200 = t2.*t3.*t5.*t9.*t10.*1.51422e-5;
t203 = t6.*t7.*t8.*t10.*t11.*2.1085e-5;
t204 = t5.*t7.*t8.*t9.*t10.*1.51422e-5;
t215 = t2.*t3.*t4.*t5.*t6.*t11.*2.1085e-5;
t221 = t4.*t5.*t6.*t7.*t8.*t11.*2.1085e-5;
t227 = t6.*t7.*t8.*t9.*t10.*t11.*(-2.1085e-5);
t20 = t9.*t17;
t21 = -t19;
t22 = t9.*t19;
t30 = t14.*t17;
t31 = t15.*t17;
t32 = -t26;
t34 = t2.*t8.*t17;
t35 = t3.*t7.*t17;
t36 = -t28;
t40 = t6.*t14.*(-3.146e-5);
t52 = -t38;
t55 = t2.*t3.*t4.*t17;
t59 = -t48;
t60 = -t49;
t61 = -t50;
t62 = -t51;
t64 = -t53;
t65 = -t54;
t82 = -t58;
t83 = t10.*t48;
t84 = t9.*t51;
t88 = -t66;
t96 = -t77;
t97 = -t78;
t98 = -t80;
t99 = t10.*t63;
t101 = t10.*t66;
t103 = t2.*t3.*t4.*t23;
t106 = t9.*t77;
t107 = t2.*t8.*t38;
t108 = t3.*t7.*t38;
t112 = t2.*t3.*t10.*t11.*t14.*3.146e-5;
t115 = -t100;
t116 = t7.*t8.*t10.*t11.*t14.*3.146e-5;
t122 = -t109;
t123 = -t110;
t127 = t2.*t8.*t53;
t128 = t3.*t7.*t53;
t130 = t2.*t8.*t66;
t131 = t3.*t7.*t66;
t135 = t2.*t3.*t5.*t10.*t23;
t138 = -t126;
t139 = -t129;
t140 = -t132;
t141 = -t133;
t144 = t2.*t3.*t5.*t63;
t150 = t4.*t7.*t8.*t15.*t23;
t151 = t10.*t132;
t152 = t10.*t133;
t156 = t102+1.318942201270233e+21;
t157 = t117+4.536700003767759e+22;
t159 = -t158;
t160 = t14.*1.50778e-5;
t161 = t16.*2.1085e-5;
t163 = t15.*1.51422e-5;
t168 = -t164;
t169 = -t165;
t172 = -t166;
t174 = t9.*t16.*(-2.1085e-5);
t175 = t14.*t16.*4.217e-5;
t177 = -t173;
t189 = -t181;
t191 = -t183;
t192 = -t187;
t199 = -t190;
t201 = t2.*t8.*t166;
t202 = t3.*t7.*t166;
t205 = t4.*t7.*t8.*t9.*t16.*4.217e-5;
t206 = t4.*t5.*t10.*t16.*(-2.1085e-5);
t207 = t4.*t7.*t8.*t16.*(-2.1085e-5);
t208 = -t196;
t209 = -t197;
t210 = -t198;
t211 = -t200;
t212 = t2.*t3.*t4.*t9.*t16.*4.217e-5;
t218 = t9.*t196;
t219 = t2.*t8.*t187;
t220 = t3.*t7.*t187;
t225 = t4.*t7.*t8.*t9.*t15.*(-1.51422e-5);
t226 = -t221;
t228 = t2.*t3.*t6.*t10.*t11.*t14.*4.217e-5;
t229 = t6.*t7.*t8.*t10.*t11.*t14.*4.217e-5;
t86 = t2.*t8.*t31;
t87 = t3.*t7.*t31;
t89 = t2.*t3.*t4.*t20;
t90 = t15.*t30;
t91 = t4.*t7.*t8.*t20;
t119 = t4.*t5.*t10.*t34;
t120 = t4.*t5.*t10.*t35;
t121 = t5.*t7.*t8.*t10.*t20;
t134 = -t116;
t147 = t2.*t8.*t99;
t148 = t3.*t7.*t99;
t167 = t9.*t161;
t179 = t2.*t8.*t163;
t180 = t3.*t7.*t163;
t184 = -t175;
t185 = t15.*t161;
t188 = t14.*t163;
t193 = t2.*t3.*t4.*t161;
t194 = t4.*t5.*t10.*t161;
t213 = t2.*t3.*t4.*t9.*t163;
t223 = -t212;
t234 = -t229;
t235 = t2.*t3.*t5.*t10.*t174;
t238 = t4.*t7.*t8.*t15.*t174;
t239 = (t10.*t157)./9.223372036854776e+26;
t240 = t5.*t11.*t156.*1.192622389734055e-26;
t242 = t43+t63+t101+t162;
t244 = t23+t37+t65+t115+t159+t174+t186;
t245 = t52+t64+t69+t88+t114+t172+t192+t206;
t248 = t47+t79+t81+t94+t105+t123+t137+t141+t151+t153+t170+t171+t178+t189;
t142 = t15.*t89;
t216 = t2.*t8.*t185;
t217 = t3.*t7.*t185;
t224 = t14.*t185;
t231 = t2.*t8.*t194;
t232 = t3.*t7.*t194;
t233 = t5.*t7.*t8.*t10.*t167;
t236 = t2.*t3.*t4.*t15.*t167;
t241 = -t239;
t243 = t41+t61+t83+t242;
t246 = t21+t33+t62+t93+t244;
t247 = t32+t59+t92+t245;
t250 = t25+t55+t68+t73+t97+t98+t106+t124+t125+t139+t144+t155+t176+t177+t193+t207+t209+t210+t218+t227;
t249 = t17+t18+t22+t24+t40+t42+t60+t67+t72+t82+t84+t90+t160+t161+t184+t188+t199+t224+3.5153434018e-2;
t251 = t36+t44+t45+t46+t86+t87+t95+t104+t113+t121+t122+t135+t136+t138+t140+t152+t154+t168+t169+t179+t180+t204+t211+t215+t216+t217+t226+t233+t235;
t252 = t85+t91+t96+t103+t107+t108+t112+t119+t120+t127+t128+t130+t131+t134+t142+t147+t148+t150+t182+t191+t201+t202+t203+t205+t208+t213+t219+t220+t223+t225+t228+t231+t232+t234+t236+t238;
t253 = t29+t39+t76+t252;
et1 = t12.*6.2666420082e-2-t13.*3.4501899918e-2+t24+t30+t31+t58+t67-t160-t161+t163+t175+t185+t190+t3.*t12.*8.43138712e-2-t5.*t12.*5.118496e-4;
et2 = t5.*t13.*(-5.118496e-4)+t12.*t13.*6.9003799836e-2-t12.*t15.*3.02844e-5-t13.*t15.*3.02844e-5+t12.*t17;
et3 = t14.*t15.*(-1.51422e-5)+t13.*t17+t12.*t40+t13.*t40+t15.*t40+t12.*t90+t13.*t90+t12.*t160+t12.*t161+t13.*t160+t13.*t161+t12.*t188+t13.*t188+t12.*t224+t13.*t224-t2.*t7.*t8.*8.43138712e-2+t3.*t5.*t12.*4.0352e-4-t5.*t6.*t12.*1.0375508e-3-t5.*t6.*t13.*1.0375508e-3;
et4 = t5.*t12.*t13.*1.0236992e-3-t6.*t12.*t13.*6.292e-5-t6.*t12.*t15.*6.292e-5-t6.*t13.*t15.*6.292e-5-t12.*t13.*t14.*3.01556e-5+t12.*t13.*t15.*6.05688e-5;
et5 = t12.*t13.*t16.*(-4.217e-5)-t12.*t14.*t16.*4.217e-5-t12.*t15.*t16.*4.217e-5;
et6 = t13.*t14.*t16.*(-4.217e-5)-t13.*t15.*t16.*4.217e-5-t14.*t15.*t16.*2.1085e-5;
et7 = t2.*t3.*t7.*t8.*(-6.9003799836e-2)-t2.*t5.*t7.*t8.*4.0352e-4+t2.*t4.*t7.*t11.*1.0375508e-3+t3.*t4.*t8.*t11.*1.0375508e-3+t3.*t5.*t6.*t12.*8.1796e-4-t2.*t7.*t9.*t10.*5.118496e-4-t3.*t8.*t9.*t10.*5.118496e-4;
et8 = t4.*t8.*t11.*t12.*(-8.1796e-4)+t5.*t6.*t12.*t13.*2.0751016e-3+t8.*t9.*t10.*t12.*4.0352e-4+t6.*t12.*t13.*t14.*6.292e-5+t6.*t12.*t13.*t15.*1.2584e-4-t12.*t13.*t14.*t15.*3.02844e-5;
et9 = t12.*t13.*t14.*t16.*8.434e-5+t12.*t13.*t15.*t16.*8.434e-5-t2.*t3.*t5.*t7.*t8.*1.0236992e-3+t2.*t3.*t6.*t7.*t8.*6.292e-5-t2.*t3.*t4.*t7.*t11.*8.1796e-4-t2.*t5.*t6.*t7.*t8.*8.1796e-4;
et10 = t2.*t4.*t5.*t7.*t11.*3.146e-5+t2.*t3.*t7.*t9.*t10.*4.0352e-4+t3.*t4.*t5.*t8.*t11.*3.146e-5-t2.*t5.*t7.*t9.*t10.*3.02844e-5+t2.*t3.*t7.*t8.*t14.*3.01556e-5-t2.*t6.*t7.*t9.*t10.*1.0375508e-3;
et11 = t2.*t3.*t7.*t8.*t15.*(-6.05688e-5)-t3.*t5.*t8.*t9.*t10.*3.02844e-5+t2.*t3.*t7.*t8.*t16.*4.217e-5-t3.*t6.*t8.*t9.*t10.*1.0375508e-3;
et12 = t2.*t4.*t7.*t11.*t13.*(-2.0751016e-3)-t3.*t4.*t8.*t11.*t12.*2.0751016e-3+t2.*t7.*t9.*t10.*t13.*1.0236992e-3+t3.*t8.*t9.*t10.*t12.*1.0236992e-3+t6.*t8.*t9.*t10.*t12.*8.1796e-4-t4.*t9.*t10.*t11.*t12.*3.146e-5-t4.*t9.*t10.*t11.*t13.*3.146e-5-t6.*t12.*t13.*t14.*t15.*6.292e-5;
et13 = t12.*t13.*t14.*t15.*t16.*(-4.217e-5)-t2.*t3.*t5.*t6.*t7.*t8.*2.0751016e-3+t2.*t4.*t5.*t6.*t7.*t11.*4.217e-5+t2.*t3.*t6.*t7.*t9.*t10.*8.1796e-4;
et14 = t3.*t4.*t5.*t6.*t8.*t11.*4.217e-5-t2.*t5.*t6.*t7.*t9.*t10.*6.292e-5-t2.*t3.*t6.*t7.*t8.*t14.*6.292e-5-t2.*t3.*t6.*t7.*t8.*t15.*1.2584e-4-t3.*t5.*t6.*t8.*t9.*t10.*6.292e-5-t2.*t4.*t5.*t7.*t11.*t13.*6.292e-5-t3.*t4.*t5.*t8.*t11.*t12.*6.292e-5;
et15 = t2.*t5.*t7.*t9.*t10.*t13.*6.05688e-5+t2.*t6.*t7.*t9.*t10.*t13.*2.0751016e-3+t3.*t5.*t8.*t9.*t10.*t12.*6.05688e-5+t3.*t6.*t8.*t9.*t10.*t12.*2.0751016e-3;
et16 = t2.*t3.*t7.*t8.*t14.*t15.*3.02844e-5-t2.*t5.*t7.*t9.*t10.*t16.*4.217e-5-t2.*t3.*t7.*t8.*t14.*t16.*8.434e-5;
et17 = t2.*t3.*t7.*t8.*t15.*t16.*(-8.434e-5)-t3.*t5.*t8.*t9.*t10.*t16.*4.217e-5-t4.*t6.*t9.*t10.*t11.*t12.*4.217e-5;
et18 = t4.*t6.*t9.*t10.*t11.*t13.*(-4.217e-5)+t4.*t9.*t10.*t11.*t12.*t13.*6.292e-5+t2.*t3.*t7.*t8.*t15.*t175-t2.*t4.*t5.*t6.*t7.*t11.*t13.*8.434e-5;
et19 = t3.*t4.*t5.*t6.*t8.*t11.*t12.*(-8.434e-5)+t2.*t5.*t6.*t7.*t9.*t10.*t13.*1.2584e-4+t3.*t5.*t6.*t8.*t9.*t10.*t12.*1.2584e-4+t2.*t3.*t6.*t7.*t8.*t14.*t15.*6.292e-5+t2.*t5.*t7.*t9.*t10.*t13.*t16.*8.434e-5;
et20 = t3.*t5.*t8.*t9.*t10.*t12.*t16.*8.434e-5+t4.*t6.*t9.*t10.*t11.*t12.*t13.*8.434e-5-t2.*t3.*t4.*t7.*t8.*t9.*t10.*t11.*6.292e-5;
et21 = t2.*t3.*t4.*t6.*t7.*t8.*t9.*t10.*t11.*(-8.434e-5)+8.85498511818e-1;
mt1 = [et1+et2+et3+et4+et5+et6+et7+et8+et9+et10+et11+et12+et13+et14+et15+et16+et17+et18+et19+et20+et21,t253,t252,t251,t250,t248,t253];
mt2 = [t3.*8.43138712e-2+t17+t24+t40+t67+t82+t90+t160+t161+t184+t188+t199+t224+t3.*t5.*4.0352e-4+t3.*t5.*t6.*8.1796e-4-t4.*t8.*t11.*8.1796e-4+t8.*t9.*t10.*4.0352e-4+t6.*t8.*t9.*t10.*8.1796e-4+9.82916754018e-1,t249,t247,t246,t243,t252,t249];
mt3 = [t17+t24+t40+t67+t82+t90+t160+t161+t184+t188+t199+t224+8.85153434018e-1,t245,t244,t242,t251,t247,t245];
mt4 = [t17-t163-t6.*t15.*3.146e-5-t15.*t16.*2.1085e-5+2.3216119e-3,t240,t241,t250,t246,t244,t240,t17+t161+2.2886849e-3,0.0,t248,t243,t242];
mt5 = [t241,0.0,4.187987e-3];
M = reshape([mt1,mt2,mt3,mt4,mt5],6,6);
end
