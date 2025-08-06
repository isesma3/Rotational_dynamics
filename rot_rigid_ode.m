function dydt = rot_rigid_ode(~, y, inert_mom)

    % Extract angular velocities
    p = y(1); q = y(2); r = y(3);

    % Extract quaternion components
    q0 = y(4); q1 = y(5); q2 = y(6); q3 = y(7);

    % Moments of inertia
    i_xx = inert_mom(1, 1); i_yy = inert_mom(2, 2); i_zz = inert_mom(3, 3);

    % Compute angular accelerations
    dp = ((i_yy - i_zz) / i_xx) * q * r;
    dq = ((i_zz - i_xx) / i_yy) * r * p;
    dr = ((i_xx - i_yy) / i_zz) * p * q;

    % Quaternion kinematics
    quatMatrix = 1/2*[0, -p, -q, -r;
        p, 0, r, -q;
        q, -r, 0, p;
        r, q, -p, 0];

    quatDeriv = quatMatrix*[q0; q1; q2; q3];

    dydt = [dp; dq; dr; quatDeriv]; % Return derivatives
end