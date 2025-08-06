function dydt = attitude_ode(t, y, omega, t_vals)
    % Initial attitude angles
    phi = y(1);
    theta = y(2);

    % Obtain angular velocities on body frame
    p = interp1(t_vals, omega(:,1), t, 'linear', 'extrap');
    q = interp1(t_vals, omega(:,2), t, 'linear', 'extrap');
    r = interp1(t_vals, omega(:,3), t, 'linear', 'extrap');
    
    % Matrix to convert from angular velocities on body frame to attitude
    % angular speeds
    transMatr = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
         0, cos(phi), -sin(phi);
         0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

    % Compute attitude angular speeds
    dAttAngles = transMatr*[p; q; r];

    dydt = dAttAngles;
end