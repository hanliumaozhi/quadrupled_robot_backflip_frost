function h_nsf = rear_left_ground_reaction_force(robot)
% rear left toe ground reaction force
GRF = robot.Inputs.ConstraintWrench.fRearLeftToe;
h_nsf = UnilateralConstraint(robot, GRF(3), 'RearLeftToeGroundReactionForce', 'fRearLeftToe');

end