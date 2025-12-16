p_below = normcdf(0.132, 0, 1);
p_above = 1 - p_below;
percentage = p_above * 100;
fprintf('Percentage of values above 0.132: %.2f%%\n', percentage);