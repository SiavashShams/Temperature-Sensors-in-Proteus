X=[-100	0	100	200	300	400	500	600	700	800];
Y=[60.27	100	138.5	175.9	212	247	280.9	313.7	345.23	375.67];
A=[X.^3' X.^2' X.^1' X.^0'];
theta=inv(A'*A)*A'*Y';
syms x
f(x) = theta(1)*x^3+theta(2)*x^2+theta(3)*x+theta(4);
ff(x)=theta(3)*x+theta(4);
g = finverse(f);
g= vpa(g,2);
gg=finverse(ff);
gg=vpa(gg,4)
XX=[100 138.5	175.9	212	247	280.9	313.7	345.23	375.67];
AA=[XX.^3' XX.^2'];
YY=[0 0 8 11 29 35 56 75 100];
theta2=inv(AA'*AA)*AA'*YY';

