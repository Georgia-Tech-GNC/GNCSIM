function u0 = finite_Ricatti(A_d, B_d, Q, R, x, x_ref, N)
n = size(A_d,1);
m = size(B_d,2);

e0 = x - x_ref;                  % n×1
c  = (A_d - eye(n)) * x_ref;       % n×1 (constant forcing term)

P = Q;                          % n×n
s = zeros(n,1);                  % n×1
K0 = zeros(m,n);
d0 = zeros(m,1);

for k = N-1:-1:0
    G = R + B_d.' * P * B_d;                       % m×m
    K = G \ (B_d.' * P * A_d);                     % m×n
    d = G \ (B_d.' * (P*c + s));                  % m×1

    if k == 0
        K0 = K;
        d0 = d;
    end

    % Riccati update
    P_next = Q + A_d.'*P*A_d - A_d.'*P*B_d*K;

    % Linear term update (for affine offset)
    s_next = A_d.'*(P*c + s) - A_d.'*P*B_d*d;

    P = P_next;
    s = s_next;
end

u0 = -K0*e0 - d0;                 % m×1
end
