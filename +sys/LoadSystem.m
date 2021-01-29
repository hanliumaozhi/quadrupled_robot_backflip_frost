function [sys, domains, guards] = LoadSystem(robot, load_path, varargin)

    rear_leg = a1.domain.rear_leg_support(robot, load_path);
    
    domains = [rear_leg];
    guards = [];
    
    sys = HybridSystem('a1');
    sys = addVertex(sys, {'RearLeg'}, 'Domain', {rear_leg});
    
end
