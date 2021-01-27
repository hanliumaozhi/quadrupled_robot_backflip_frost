function contact = front_left_toe_contact(robot)
    % The left toe contact point
    contact.frame = ToContactFrame(robot.ContactPoints.front_left_toe,...
        'PointContactWithFriction');
    contact.fric_coef.mu = 0.6;
    contact.fric_coef.gamma = 100;  
end