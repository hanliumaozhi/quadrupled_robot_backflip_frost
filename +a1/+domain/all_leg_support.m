function domain = all_leg_support(model, load_path, varargin)
    % construct the all leg contact
    
    % Parse inputs
    p = inputParser;
    p.addOptional('name', 'AllLegSupport')
    p.parse(varargin{:});
    parser_results = p.Results;
    

    % make a copy of the robot model
    domain = copy(model);
    
    % Add contacts
    
    front_right_contact = a1.holonomic.front_right_toe_contact(domain);
    domain = addContact(domain, front_right_contact.frame, front_right_contact.fric_coef, front_right_contact.geometry, load_path);
    
    front_left_contact = a1.holonomic.front_left_toe_contact(domain);
    domain = addContact(domain, front_left_contact.frame, front_left_contact.fric_coef, front_left_contact.geometry, load_path);
    
    rear_right_contact = a1.holonomic.rear_right_toe_contact(domain);
    domain = addContact(domain, rear_right_contact.frame, rear_right_contact.fric_coef, rear_right_contact.geometry, load_path);
    
    rear_left_contact = a1.holonomic.rear_left_toe_contact(domain);
    domain = addContact(domain, rear_left_contact.frame, rear_left_contact.fric_coef, rear_left_contact.geometry, load_path);
    
    
    % add guard
    
    %1. grf z more than zero
    domain = addEvent(domain, a1.unilateral.front_left_ground_reaction_force(domain));
    domain = addEvent(domain, a1.unilateral.front_right_ground_reaction_force(domain));
    domain = addEvent(domain, a1.unilateral.rear_left_ground_reaction_force(domain));
    domain = addEvent(domain, a1.unilateral.rear_right_ground_reaction_force(domain)); 
  


    % Virtual constraints
    domain = addVirtualConstraint(domain, a1.virtual.actuated_joints(domain, load_path));
        
    % Set the name of the new copy
    domain.setName(parser_results.name);
   
end