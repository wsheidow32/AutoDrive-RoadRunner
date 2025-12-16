
figure; subplot(3, 1, 1)

plot(out.safedist, 'Linewidth', 2);
hold on
plot(out.reldist, 'Linewidth', 2);
xline(4.9, '--', 'LineWidth',3);
xline(16.3, '--','LineWidth',3);
xline(21.5, '--','LineWidth',3);
xline(24.4, '--','LineWidth',3);
hold off
legend('Safe Distance', 'Relative Distance');
ylabel('Distance (m)');
xlabel('Time(s)');
ylim([0 105]);
grid on
title('Safe Distance vs. Relative Distance plot');
subplot(3,1,2)

plot(out.curvel, 'Linewidth', 2);
hold on 
plot(out.setvel, 'Linewidth', 2);
yline(10, '--c','LineWidth',3);
xline(4.9, '--','LineWidth',3);
xline(16.3, '--','LineWidth',3);
xline(21.5, '--','LineWidth',3);
xline(24.4, '--','LineWidth',3);
legend('Current Velocity', 'Lead Velocity', 'Set Velocity');
hold off

ylabel('Velocity (m/s)');
xlabel('Time(s)');
grid on
ylim([-1.5 12]);
title('Various Velocity plots');
subplot(3,1,3)

plot(out.relvel, 'Linewidth', 2)
hold on 
xline(4.9, '--','LineWidth',3);
xline(16.3, '--','LineWidth',3);
xline(21.5, '--','LineWidth',3);
xline(24.4, '--','LineWidth',3);
hold off
legend('Relative velocity');
ylabel('Velocity (m/s)');
xlabel('Time(s)');
ylim([-10.5 12]);
grid on
title('Relative velocity plot');