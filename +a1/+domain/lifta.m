function guard = lifta(model, load_path, varargin)

    
    % Default domain name
    Name = 'LiftA';
           
    % get next domain
    domain = a1.domain.rear_leg_support(model, load_path);

    
    
    guard = RigidImpact(Name, domain, 'RearLeftToeGroundReactionForce');
    
    
    % 因为这里不需要碰撞，直接初始化guard 使变换前后 位置速度相等
    
    guard.configure(load_path);
    
end