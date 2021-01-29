function h_nsf = front_left_toe_height(robot)
    % front left toe height constraint
    p_front_left_toe = getCartesianPosition(robot, robot.ContactPoints.front_left_toe);
    h_nsf = UnilateralConstraint(robot, p_front_left_toe(3), 'frontLeftToeHeight', 'x');
end