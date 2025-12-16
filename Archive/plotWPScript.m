time = out.x.Time;
data = zeros(2, length(time));
data = [out.x.Data(:, :, 1);
        out.y.Data(:, :, 1)];

for i = 1:length(time)

    data = [data, [out.x.Data(:, :, i); out.y.Data(:, :, i)]];
end

% disp(data)

normed_dist = zeros(1, length(time))
normed_dist(1) = norm(data(:, 1));

for k = 1:length(data)
    normed_dist = [normed_dist, norm(data(:, k))];
end
waypoints_x = out.waypoints.signals.values(:, 1);
waypoints_y = out.waypoints.signals.values(:, 2);

waypoints = [waypoints_x(find(waypoints_x(:, 1)), 1), waypoints_y(find(waypoints_y(:, 1)), 1)];
plot(waypoints(:,1), waypoints(:,2), 'LineWidth', 1.5,'DisplayName', 'Reference Points');
xlabel('Y-axis on RR');
ylabel('X-axis on RR');
hold on
plot(-data(2, :)+5, -data(1, :), 'LineWidth', 1.5,'DisplayName', 'Current Loc. Points');
legend()
