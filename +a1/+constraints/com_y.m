function [com_y_fun] = com_y(nlp)

com = getComPosition(nlp.Plant);
expr = com(2);
com_y_fun = SymFunction(['com_y_', nlp.Plant.Name], expr, {nlp.Plant.States.x});

end