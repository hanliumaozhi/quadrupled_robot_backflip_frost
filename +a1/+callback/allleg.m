function allleg(nlp, bounds, varargin)
    domain = nlp.Plant;
    
    %% Add Virtual Constraints
    domain.VirtualConstraints.time.imposeNLPConstraint(nlp, [bounds.time.kp, bounds.time.kd], [1,1]);
    
    %% Tau Boundary [0,1]
    addNodeConstraint(nlp, a1.constraints.tau0(nlp), ...
        [{'T'}, domain.VirtualConstraints.time.PhaseParamName], 'first', 0, 0, 'Nonlinear');
    addNodeConstraint(nlp, a1.constraints.tauF(nlp), ...
        [{'T'}, domain.VirtualConstraints.time.PhaseParamName], 'last', 0, 0, 'Nonlinear');
    
    %% other constraints
    
    % com x
%     for k =1:21
%        addNodeConstraint(nlp, a1.constraints.com_x(nlp), ...
%            {'x'}, k, 0, 0, 'Nonlinear');
%     end
    
    % com y
    for k =1:21
       addNodeConstraint(nlp, a1.constraints.com_y(nlp), ...
           {'x'}, k, 0, 0, 'Nonlinear');
    end
    
    % com z // only for first node which base_link 
    addNodeConstraint(nlp, a1.constraints.com_z(nlp), ...
           {'x'}, 1, bounds.init_com_z, bounds.init_com_z, 'Nonlinear');
       
       
     addNodeConstraint(nlp, a1.constraints.com_z(nlp), ...
           {'x'}, 21, bounds.end_com_z, bounds.end_com_z, 'Nonlinear');
       
    % momentum z
%     for k =1:21
%     addNodeConstraint(nlp, a1.constraints.momentum_z(nlp), ...
%            {'x', 'dx'}, k, bounds.momentum_z.lb, bounds.momentum_z.ub, 'Nonlinear');
%     end
       
%     addNodeConstraint(nlp, a1.constraints.momentum_x(nlp), ...
%            {'x', 'dx'}, 21, bounds.momentum_x.lb, bounds.momentum_x.ub, 'Nonlinear');
       
%    % momentum pitch
%     addNodeConstraint(nlp, a1.constraints.momentum_ptich(nlp), ...
%            {'x', 'dx'}, 21, 1, 1, 'Nonlinear');
       
    addNodeConstraint(nlp, a1.constraints.toe_x(nlp), ...
           {'x'}, 1, 0, 0, 'Nonlinear');
       
       addNodeConstraint(nlp, a1.constraints.toe_y(nlp), ...
           {'x'}, 1, 0, 0, 'Nonlinear');
       
       addNodeConstraint(nlp, a1.constraints.vel_x(nlp), ...
           {'dx'}, 21, -1.8, -1.8, 'Nonlinear');
       
%     % knee z
%     
%      for k=21:21
%        addNodeConstraint(nlp, a1.constraints.fl_knee_z(nlp), ...
%            {'x'}, k, bounds.knee_z.lb, bounds.knee_z.ub, 'Nonlinear');
%        addNodeConstraint(nlp, a1.constraints.fr_knee_z(nlp), ...
%            {'x'}, k, bounds.knee_z.lb, bounds.knee_z.ub, 'Nonlinear');
%        addNodeConstraint(nlp, a1.constraints.rl_knee_z(nlp), ...
%            {'x'}, k, bounds.knee_z.lb, bounds.knee_z.ub, 'Nonlinear');
%        addNodeConstraint(nlp, a1.constraints.rr_knee_z(nlp), ...
%            {'x'}, k, bounds.knee_z.lb, bounds.knee_z.ub, 'Nonlinear');
%      end
    
   
       
    
    %% cost
     addRunningCost(nlp, a1.costs.torque(nlp), 'u');
     %addRunningCost(nlp, a1.costs.q_error(nlp), 'x');
    
end