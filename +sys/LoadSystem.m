function [sys, domains, guards] = LoadSystem(robot, load_path, varargin)

    all_leg = a1.domain.all_leg_support(robot, load_path);
    lift_a = a1.domain.lifta(robot, load_path);
    rear_leg = a1.domain.rear_leg_support(robot, load_path);
    
    domains = [all_leg, rear_leg];
    guards = [lift_a];
    
    sys = HybridSystem('a1');
    sys = addVertex(sys, {'AllLeg', 'RearLeg'}, 'Domain', {all_leg, rear_leg});
    
    sys = addEdge(sys, 'AllLeg', 'RearLeg');
    sys = setEdgeProperties(sys, 'AllLeg', 'RearLeg', 'Guard', lift_a);
    
end
