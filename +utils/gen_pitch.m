acc_arr = time.*35;
postion_arr = zeros(21,1);
for i=1:21
    postion_arr(i) = -35*(time(i)*time(i))/2;
end