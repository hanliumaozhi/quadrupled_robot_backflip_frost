function h_nsf = rear_right_ground_reaction_force(robot)
% rear left toe ground reaction force
GRF = robot.Inputs.ConstraintWrench.fRearRightToe;
h_nsf = UnilateralConstraint(robot, GRF(3), 'RearRightToeGroundReactionForce', 'fRearRightToe');

end