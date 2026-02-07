function [Ad, Bd] = matrix_exp(A, B, dt)
n = size(A,1);
m = size(B,2);

M = [A, B;
     zeros(m,n), zeros(m,m)];

E = expm(M*dt);

Ad = E(1:n, 1:n);
Bd = E(1:n, n+1:n+m);
end
