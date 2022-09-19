syms arpha beta d l1 l2
p1 = (-(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 - 4*l2^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))/(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta)))

x = (l1 * cos(arpha)) / 2 - (l1 * cos(beta)) / 2 - (l1 * sin(arpha) * p1^0.5) / 2 + (l1 * sin(beta) * p1^0.5) / 2;
y = -(l1 * sin(arpha)) / 2 - (l1 * sin(beta)) / 2 - (d * p1^0.5) / 2 - (l1 * cos(arpha) * p1^0.5) / 2 - (l1 * cos(beta) * p1^0.5) / 2;

J =jacobian([x;y],[arpha beta])

simplify(J)
%赋值验证一下
arpha  = 0;
beta  = 0; 
d = 0.1;
l1 = 0.2;
l2 = 0.3;

jac = [[                                                                                                                                                                                                                                                                                                                               (2*l1^2*l2^2*sin(beta)*(l1*sin(arpha + beta) + d*sin(arpha)))/((-(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 - 4*l2^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))/(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta)))^(1/2)*(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))^2) - (l1*cos(arpha)*(-(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 - 4*l2^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))/(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta)))^(1/2))/2 - (2*l1^2*l2^2*sin(arpha)*(l1*sin(arpha + beta) + d*sin(arpha)))/((-(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 - 4*l2^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))/(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta)))^(1/2)*(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))^2) - (l1*sin(arpha))/2,                                                                                                                                                                                                                                                                                                                               (l1*sin(beta))/2 + (l1*cos(beta)*(-(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 - 4*l2^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))/(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta)))^(1/2))/2 - (2*l1^2*l2^2*sin(arpha)*(l1*sin(arpha + beta) + d*sin(beta)))/((-(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 - 4*l2^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))/(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta)))^(1/2)*(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))^2) + (2*l1^2*l2^2*sin(beta)*(l1*sin(arpha + beta) + d*sin(beta)))/((-(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 - 4*l2^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))/(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta)))^(1/2)*(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))^2)]
      [(l1*sin(arpha)*(-(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 - 4*l2^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))/(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta)))^(1/2))/2 - (l1*cos(arpha))/2 - (2*l1^2*l2^2*cos(arpha)*(l1*sin(arpha + beta) + d*sin(arpha)))/((-(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 - 4*l2^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))/(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta)))^(1/2)*(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))^2) - (2*l1^2*l2^2*cos(beta)*(l1*sin(arpha + beta) + d*sin(arpha)))/((-(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 - 4*l2^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))/(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta)))^(1/2)*(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))^2) - (2*d*l1*l2^2*(l1*sin(arpha + beta) + d*sin(arpha)))/((-(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 - 4*l2^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))/(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta)))^(1/2)*(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))^2), (l1*sin(beta)*(-(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 - 4*l2^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))/(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta)))^(1/2))/2 - (l1*cos(beta))/2 - (2*l1^2*l2^2*cos(arpha)*(l1*sin(arpha + beta) + d*sin(beta)))/((-(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 - 4*l2^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))/(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta)))^(1/2)*(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))^2) - (2*l1^2*l2^2*cos(beta)*(l1*sin(arpha + beta) + d*sin(beta)))/((-(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 - 4*l2^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))/(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta)))^(1/2)*(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))^2) - (2*d*l1*l2^2*(l1*sin(arpha + beta) + d*sin(beta)))/((-(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 - 4*l2^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))/(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta)))^(1/2)*(2*l1^2*cos(arpha + beta) + d^2 + 2*l1^2 + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))^2)]]

j = inv(jac')

jac * [1;2]




