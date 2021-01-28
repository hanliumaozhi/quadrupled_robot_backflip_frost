function [momentum_pitch_fun] = momentum_pitch(nlp)
momentum = getMomentum(nlp.Plant);
expr = momentum(5);
momentum_pitch_fun = SymFunction(['momentum_pitch_', nlp.Plant.Name], expr, {nlp.Plant.States.x, nlp.Plant.States.dx});
end