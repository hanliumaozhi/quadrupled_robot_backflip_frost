function [com_y_fun] = com_z(nlp)

com = getComPosition(nlp.Plant);
expr = com(3);
com_y_fun = SymFunction(['com_z_', nlp.Plant.Name], expr, {nlp.Plant.States.x});
end