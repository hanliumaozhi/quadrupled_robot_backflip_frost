function rearleg(nlp, bounds, varargin)
    domain = nlp.Plant;
    
    %% Add Virtual Constraints
    domain.VirtualConstraints.time.imposeNLPConstraint(nlp, [bounds.time.kp, bounds.time.kd], [1,1]);
    
    %% Tau Boundary [0,1]
    addNodeConstraint(nlp, a1.constraints.tau0(nlp), ...
        [{'T'}, domain.VirtualConstraints.time.PhaseParamName], 'first', 0, 0, 'Nonlinear');
    addNodeConstraint(nlp, a1.constraints.tauF(nlp), ...
        [{'T'}, domain.VirtualConstraints.time.PhaseParamName], 'last', 0, 0, 'Nonlinear');
    
    %% user constraints
    for k =1:21
       addNodeConstraint(nlp, a1.constraints.pitch(nlp), ...
           {'x'}, k, bounds.pitch_arr(k)-0.01, bounds.pitch_arr(k)+0.01, 'Nonlinear');
    end
    %% cost
     addRunningCost(nlp, a1.costs.torque(nlp), 'u');
end