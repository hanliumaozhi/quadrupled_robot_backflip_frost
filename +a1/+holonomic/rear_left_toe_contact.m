function contact = rear_left_toe_contact(robot)
    % The left toe contact point
    contact.frame = ToContactFrame(robot.ContactPoints.rear_left_toe,...
        'PointContactWithFriction');
    contact.fric_coef.mu = 0.6;
    contact.fric_coef.gamma = 100;
    contact.geometry.RefFrame = eye(3);
end