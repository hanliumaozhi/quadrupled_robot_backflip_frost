function bounds = GetBound(robot)

model_bounds = robot.getLimits();
bounds = struct();

model_bounds.states.x.lb(1:3) = [-10,-10,-10];
model_bounds.states.x.ub(1:3) = [10,10,10];

model_bounds.states.x.lb(4:6) = deg2rad(-5);
model_bounds.states.x.ub(4:6) = deg2rad(5);

% not pitch
model_bounds.states.x.lb(5) = -10;
model_bounds.states.x.ub(5) = 10;


%% Jump
bounds.AllLeg = model_bounds;

% time is VirtualConstraint
bounds.AllLeg.time.t0.lb = 0;
bounds.AllLeg.time.t0.ub = 0;
bounds.AllLeg.time.t0.x0 = 0;

bounds.AllLeg.time.tf.lb = 0.4;
bounds.AllLeg.time.tf.ub = 0.4;
bounds.AllLeg.time.tf.x0 = 0.4;

bounds.AllLeg.time.duration.lb = 0.4;
bounds.AllLeg.time.duration.ub = 0.4;
bounds.AllLeg.time.duration.x0 = 0.4;

bounds.AllLeg.time.kp = 100;
bounds.AllLeg.time.kd = 20;


% contact constraint force
bounds.AllLeg.inputs.ConstraintWrench.fFrontLeftToe.lb = -10000;
bounds.AllLeg.inputs.ConstraintWrench.fFrontLeftToe.ub = 10000;
bounds.AllLeg.inputs.ConstraintWrench.fFrontLeftToe.x0 = 10;

bounds.AllLeg.inputs.ConstraintWrench.fFrontRightToe.lb = -10000;
bounds.AllLeg.inputs.ConstraintWrench.fFrontRightToe.ub = 10000;
bounds.AllLeg.inputs.ConstraintWrench.fFrontRightToe.x0 = 10;

bounds.AllLeg.inputs.ConstraintWrench.fRearLeftToe.lb = -10000;
bounds.AllLeg.inputs.ConstraintWrench.fRearLeftToe.ub = 10000;
bounds.AllLeg.inputs.ConstraintWrench.fRearLeftToe.x0 = 10;

bounds.AllLeg.inputs.ConstraintWrench.fRearRightToe.lb = -10000;
bounds.AllLeg.inputs.ConstraintWrench.fRearRightToe.ub = 10000;
bounds.AllLeg.inputs.ConstraintWrench.fRearRightToe.x0 = 10;

% contact constraint position
bounds.AllLeg.params.pFrontLeftToe.lb = [-1;-1;0];
bounds.AllLeg.params.pFrontLeftToe.ub = [1;1;0];
bounds.AllLeg.params.pFrontLeftToe.x0 = [0;0;0];

bounds.AllLeg.params.pFrontRightToe.lb = [-1;-1;0];
bounds.AllLeg.params.pFrontRightToe.ub = [1;1;0];
bounds.AllLeg.params.pFrontRightToe.x0 = [0;0;0];

bounds.AllLeg.params.pRearLeftToe.lb = [-1;-1;0];
bounds.AllLeg.params.pRearLeftToe.ub = [1;1;0];
bounds.AllLeg.params.pRearLeftToe.x0 = [0;0;0];

bounds.AllLeg.params.pRearRightToe.lb = [-1;-1;0];
bounds.AllLeg.params.pRearRightToe.ub = [1;1;0];
bounds.AllLeg.params.pRearRightToe.x0 = [0;0;0];



bounds.AllLeg.params.atime.lb = -10*ones(6*12,1);
bounds.AllLeg.params.atime.ub = 10*ones(6*12,1);
bounds.AllLeg.params.atime.x0 = zeros(6*12,1);

bounds.AllLeg.params.ptime.lb = [bounds.AllLeg.time.tf.lb, bounds.AllLeg.time.t0.lb];
bounds.AllLeg.params.ptime.ub = [bounds.AllLeg.time.tf.ub, bounds.AllLeg.time.t0.ub];
bounds.AllLeg.params.ptime.x0 = [bounds.AllLeg.time.t0.x0, bounds.AllLeg.time.tf.x0];



end
