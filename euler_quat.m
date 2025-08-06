function eulerAng = euler_quat(quatEvol)

    % Read quaternion evolution
    q0 = quatEvol(:, 1);
    q1 = quatEvol(:, 2);
    q2 = quatEvol(:, 3);
    q3 = quatEvol(:, 4);

    nTimes = length(quatEvol(:, 1));
    eulerAng = zeros(nTimes, 3);

    for i = 1:nTimes
        eulerAng(i, 1) = atan2(2*(q2(i)*q3(i) + q0(i)*q1(i)), q0(i)^2 - q1(i)^2 - q2(i)^2 + q3(i)^2);
        eulerAng(i, 2) = - asin(2*(q1(i)*q3(i) - q0(i)*q2(i)));
        eulerAng(i, 3) = atan2(2*(q1(i)*q2(i) + q0(i)*q3(i)), q0(i)^2 + q1(i)^2 - q2(i)^2 - q3(i)^2);
    end

end