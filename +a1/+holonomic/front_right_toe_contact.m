function contact = front_right_toe_contact(robot)
    % The right toe contact point
    contact.frame = ToContactFrame(robot.ContactPoints.front_right_toe,...
        'PointContactWithFriction');
    contact.fric_coef.mu = 0.6;
    contact.fric_coef.gamma = 100;  
end