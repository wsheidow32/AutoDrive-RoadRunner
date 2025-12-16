%% Current Position Waypoints

Real_Points = out.realPosition.signals.values(:, 1:2);
% realPos_Points = unique(realPosition_Points, 'rows');

x_CurrentPos = Real_Points(:, 1);
y_CurrentPos = Real_Points(:, 2);

Real_XY = [x_CurrentPos, y_CurrentPos];

%% Reference Waypoints
Reference_Points = out.ReferencePoints.signals.values(:, :, 1);

x_RefPoints = Reference_Points(:, 1);
y_RefPoints = Reference_Points(:, 2);

%% INS Waypoints
INS_Points = out.insPosition.signals.values(:, 1:2);

y_INSPos = -1*INS_Points(:, 1);
x_INSPos = INS_Points(:, 2);

%% Plots & Figures

figure;
p1 = plot(-y_RefPoints, x_RefPoints, 'LineWidth',2);
hold on
p2 = plot(-y_CurrentPos, x_CurrentPos, '-r', 'LineWidth', 0.5);
hold on
p3 = plot(-y_INSPos, x_INSPos, '-g', 'LineStyle','-', 'LineWidth', 2);

% legend('Current Position')
legend('Reference', 'Current', 'INS Current');
fontsize(legend, 14, 'points')


%%% Manipulation

%% Manipulate

idx = x_CurrentPos < 0;

hold on
count = 0;
myplot = plot(-y_CurrentPos(idx),x_CurrentPos(idx),'*k');

for ind = idx
    datatip(myplot, -y_CurrentPos(idx), x_CurrentPos(idx));
    count = count + 1;
end
realPos_xy = zeros(length(real_xy), 2);
realPos_xy(1, 1:2) = real_xy(1, 1:2);

%%
plot(realPos_xy(:, 1), realPos_xy(:, 2), '-r', 'LineWidth', 0.5);

%% Difference
diff = zeros(length(real_xy), 2);
diff(1, 1:2) = real_xy(1, 1:2);

for j = 2:length(diff)
    diff(j, 1:2) = real_xy(j, 1:2) - real_xy(j-1, 1:2);
end

disp(diff(1:20, 1:2))


%% 
realPos_xy = zeros(length(real_xy), 2);
count = 2;
for len = 2:length(diff)
    if diff(len, 1) >= 0
        realPos_xy(count, 1:2) = real_xy(len, 1:2);
        count = count + 1;
    end
end
% for i = 1:length(realest_xy)
%     if floor(real_xy(1)) ~= -22
%         realest_xy(i, :) = real_xy(i, :);
%     end
% end

%% Close Figures
close;

%% Metrics, Error Calculations
error = [sqrt((x_RefPoints(110) - x3).^2 + (y_RefPoints(110) - y3).^2)];