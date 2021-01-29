function [x_constr_fun] = x_constr(nlp, x_index)
expr = nlp.Plant.States.x(x_index);
x_constr_fun = SymFunction(['x_constr_', num2str(x_index) , nlp.Plant.Name], expr, {nlp.Plant.States.x});
end