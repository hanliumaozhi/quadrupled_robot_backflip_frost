function h_nsf = front_right_toe_height(robot)
    % front right toe height constraint
    p_front_right_toe = getCartesianPosition(robot, robot.ContactPoints.front_right_toe);
    h_nsf = UnilateralConstraint(robot, p_front_right_toe(3), 'frontRightToeHeight', 'x');
end