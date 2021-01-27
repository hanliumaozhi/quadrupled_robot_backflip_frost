function [sys, domains, guards] = LoadSystem(robot, load_path, varargin)

    all_leg = a1.domain.all_leg_support(robot, load_path);
    
    domains = [all_leg];
    guards = [];
    
    sys = HybridSystem('a1');
    sys = addVertex(sys, {'AllLeg'}, 'Domain', {all_leg});

    
end
