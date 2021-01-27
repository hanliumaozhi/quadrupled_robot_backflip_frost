function [ y2 ] = actuated_joints( robot, load_path )
% Virtual contraint consisting of all actuated joints

% phase variable: time
t = SymVariable('t');
p = SymVariable('p',[2,1]);
tau = (t-p(2))/(p(1)-p(2));

% relative degree two outputs:
% front right
y_front_right_hip = robot.States.x('FR_hip_joint');
y_front_right_thight = robot.States.x('FR_thigh_joint');
y_front_right_calf = robot.States.x('FR_calf_joint');

% front left
y_front_left_hip = robot.States.x('FL_hip_joint');
y_front_left_thight = robot.States.x('FL_thigh_joint');
y_front_left_calf = robot.States.x('FL_calf_joint');

% rear right
y_rear_right_hip = robot.States.x('RR_hip_joint');
y_rear_right_thight = robot.States.x('RR_thigh_joint');
y_rear_right_calf = robot.States.x('RR_calf_joint');

% rear left
y_rear_left_hip = robot.States.x('RL_hip_joint');
y_rear_left_thight = robot.States.x('RL_thigh_joint');
y_rear_left_calf = robot.States.x('RL_calf_joint');

ya_2 = [y_front_right_hip;
        y_front_right_thight;
        y_front_right_calf;
        y_front_left_hip;
        y_front_left_thight;
        y_front_left_calf;
        y_rear_right_hip;
        y_rear_right_thight;
        y_rear_right_calf;
        y_rear_left_hip;
        y_rear_left_thight;
        y_rear_left_calf];

y2_label = {'front_right_hip',...
    'front_right_thight',...
    'front_right_calf',...
    'front_left_hip',...
    'front_left_thight',...
    'front_left_calf',...
    'rear_right_hip',...
    'rear_right_thight',...
    'rear_right_calf',...
    'rear_left_hip',...
    'rear_left_thight',...
    'rear_left_calf'};

y2 = VirtualConstraint(robot, ya_2, 'time', 'DesiredType', 'Bezier', 'PolyDegree',5,...
    'RelativeDegree',2,'OutputLabel',{y2_label},'PhaseType','TimeBased',...
    'PhaseVariable',tau,'PhaseParams',p,'Holonomic',true, 'LoadPath', load_path);

end

