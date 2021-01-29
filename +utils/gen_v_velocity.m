total_time = 0.3;
num_node = 21;
end_velocity = -1.8;
time_int_ = total_time/(num_node-1);
time = (0:time_int_:total_time)';
acc = end_velocity/total_time;
vel_arr = time.*acc;