function T = thrust_at(t, arr, burn_time)
% Linear clamp+interpolate on your arr table (Nx2: [time, thrust])
    if t > burn_time
        T = 0;           % post-burnout
        return
    end
    n = size(arr,1);
    if t <= arr(1,1), T = arr(1,2); return; end
    if t >= arr(n,1), T = arr(n,2); return; end

    % find segment
    j = find(arr(:,1) <= t, 1, 'last');
    if j == n, T = arr(n,2); return; end
    t0 = arr(j,1);  t1 = arr(j+1,1);
    y0 = arr(j,2);  y1 = arr(j+1,2);

    if t1 == t0
        T = y1;  % degenerate
    else
        s = (t - t0) / (t1 - t0);
        T = (1-s)*y0 + s*y1;
    end
end
