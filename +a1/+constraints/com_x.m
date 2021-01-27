function [com_x_fun] = com_x(nlp)

com = getComPosition(nlp.Plant);
expr = com(1);
com_x_fun = SymFunction(['com_x_', nlp.Plant.Name], expr, {nlp.Plant.States.x});

end