function domain = rear_leg_support(model, load_path, varargin)
    % construct the all leg contact

    % Parse inputs
    p = inputParser;
    p.addOptional('name', 'RearLegSupport')
    p.parse(varargin{:});
    parser_results = p.Results;
    
    % make a copy of the robot model
    domain = copy(model);
    
    rear_right_contact = a1.holonomic.rear_right_toe_contact(domain);
    domain = addContact(domain, rear_right_contact.frame, rear_right_contact.fric_coef, rear_right_contact.geometry, load_path);
    
    rear_left_contact = a1.holonomic.rear_left_toe_contact(domain);
    domain = addContact(domain, rear_left_contact.frame, rear_left_contact.fric_coef, rear_left_contact.geometry, load_path);
    
    % add guard
    
    %1. grf z more than zero
    domain = addEvent(domain, a1.unilateral.rear_left_ground_reaction_force(domain));
    domain = addEvent(domain, a1.unilateral.rear_right_ground_reaction_force(domain));
    %2. free leg z motre than zero
    domain = addEvent(domain, a1.unilateral.front_left_toe_height(domain));
    domain = addEvent(domain, a1.unilateral.front_right_toe_height(domain)); 
    
    % Virtual constraints
    domain = addVirtualConstraint(domain, a1.virtual.actuated_joints(domain, load_path));
        
    % Set the name of the new copy
    domain.setName(parser_results.name);
end