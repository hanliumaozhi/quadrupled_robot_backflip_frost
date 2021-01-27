function h_nsf = front_left_ground_reaction_force(robot)
% front left toe ground reaction force
GRF = robot.Inputs.ConstraintWrench.fFrontLeftToe;
h_nsf = UnilateralConstraint(robot, GRF(3), 'frontLeftToeGroundReactionForce', 'fFrontLeftToe');

end