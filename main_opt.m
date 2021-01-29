%% Main script for A1
% song liu

%% Setting up path
clear; close all; clc;
% Restore search path to factory-installed state, Start up file for MATLAB program.
%restoredefaultpath; matlabrc;  

% specify the path to the FROST
% only for my computer you need change it
frost_path  = '../frost-dev';
addpath(frost_path);
frost_addpath; % initialize FROST
cur = utils.get_root_path();
export_path = fullfile(cur, 'gen/opt'); % path to export compiled C++ and MEX files
utils.init_path(export_path);
%% robot model settings
%cur = utils.get_root_path();
%urdf = fullfile(cur,'urdf','casia_nnn.urdf'); % Build full file name from parts 
urdf = fullfile(cur,'urdf','a1.urdf'); % Build full file name from parts
% some options

% if 'delay_set' is true, the computation of system dynamics (Coriolis
% vector) will be delayed. Delaying this operation will save significant
% loading time.
delay_set = false;
OMIT_CORIOLIS = true;
LOAD = false;
% if 'load_sym' is true, it will load symbolic expressions from previously
% save external files instead of re-compute them. It reduce the loading
% time by 7-10 faster. 
% Set it to false for the first time, and save expressions after loaded the
% model. 
% % % load_sym  = false; % if true, it will load symbolic expression from 
% % % if load_sym    
% % %     load_path = 'gen/sym'; % path to export binary Mathematica symbolic expression (MX) files
% % %     utils.init_path(load_path);
% % % else
% % %     load_path   = []; 
% % % end
LOAD_PATH = fullfile(cur, 'gen/sym');
%% load robot model
% load the robot model
robot = sys.LoadModel(urdf);  % call  loadDynamics()
if LOAD
    loadDynamics(robot, LOAD_PATH, delay_set, {},'OmitCoriolisSet',OMIT_CORIOLIS);
    %robot.loadDynamics(LOAD_PATH, OMIT_CORIOLIS);
    system = sys.LoadSystem(robot, LOAD_PATH);
else
    robot.configureDynamics('DelayCoriolisSet',OMIT_CORIOLIS,'OmitCoriolisSet',OMIT_CORIOLIS);
    %robot.configureDynamics('DelayCoriolisSet',OMIT_CORIOLIS);
    [system, domains, guards] = sys.LoadSystem(robot, '');
end
% load hybrid system

%% Load optimization problem
% get the boundary values, needs to be manually set all boundaries.
%%%%%%%%%%%
bounds = a1.GetBound(robot);
%%%%%%%%%%%
num_grid.AllLeg = 10;
% load problem
nlp = HybridTrajectoryOptimization('backflip',system, num_grid, [],'EqualityConstraintBoundary',1e-4);
nlp.Phase(1).Plant.UserNlpConstraint = @a1.callback.allleg;
nlp.update; 

nlp.configure(bounds);
nlp.update;

%% Compile stuff if needed (only need to run for the first time)
%compileObjective(nlp,[],[],export_path);
%compileConstraint(nlp,[],[],export_path, {'dynamics_equation'});

compileObjective(nlp,[],[],export_path);
compileConstraint(nlp,[],[],export_path);


%% Save expression (only need to run for the first time)
load_path   = fullfile(cur, 'gen/sym');
utils.init_path(load_path);
system.saveExpression(load_path); % run this after loaded the optimization problem



%% you can update bounds without reloading the problem. It is much much faster!!!
%%%%%%%%%%%%%%%%%%%%%%%%
bounds = a1.GetBound(robot);
%%%%%%%%%%%%%%%%%%%%%%%%%
nlp.configure(bounds);
nlp.update;


%% update initial condition if use pre-existing gaits
% param = load('local/tmp_gait.mat');
% cassie.updateInitCondition(nlp,param.gait);



%% solve (two ways to call "solve")
% 1. If use the typical variables values as the initial guess
% 2. If use existing solutoin as the initial guess
%[gait, sol, info] = opt.solve(nlp, sol); % if use previous solution "sol"
% 3. warm start using use existing solutoin as the initial guess
%[gait, sol, info] = opt.solve(nlp, sol, info); % if use previous solution "sol"
[gait, sol, info, total_time] = a1.solve(nlp);


 
%% save
save('local/backflip.mat','gait','sol','info','bounds');



%% animation
ANIM_PATH = fullfile(cur, 'gen', 'animator');
if ~exist(ANIM_PATH,'dir')
    mkdir(ANIM_PATH); 
end
anim = plot.a1_load_animation(robot, gait, [], 'ExportPath', ANIM_PATH, 'SkipExporting', false); % set 'SkipExporting' = false, only for the first time!



%% you can check the violation of constraints/variables and the value of each cost function by calling the following functions.
tol = 1e-3;
checkConstraints(nlp,sol,tol,'local/constr_check.txt') % 
checkVariables(nlp,sol,tol,'local/var_check.txt') % 
checkCosts(nlp,sol,'local/cost_check.txt') % 



%% you can also plot the states and torques w.r.t upper/lower bounds.
%plot.plotOptStates(robot,nlp,gait);
plot.plotOptTorques(robot,nlp,gait);

PlotTrajectories(gait(1).states.x, gait(1).states.dx, {robot.Joints.Name});


