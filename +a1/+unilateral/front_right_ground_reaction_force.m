function h_nsf = front_right_ground_reaction_force(robot)
% front left toe ground reaction force
GRF = robot.Inputs.ConstraintWrench.fFrontRightToe;
h_nsf = UnilateralConstraint(robot, GRF(3), 'frontRightToeGroundReactionForce', 'fRightLeftToe');

end