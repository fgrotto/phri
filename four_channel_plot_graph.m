figure();
subplot(3,1,1);
hold on;
grid on;
plot(out.xd.Time(1:end), out.xd.data(1:end),'LineWidth', 1.2);
hold on, plot(out.xs.Time(1:end), out.xs.data(1:end),'LineWidth', 1.2);
hold on, plot(out.xm.Time(1:end), out.xm.data(1:end),'LineWidth', 1.2);
% yline(0.8,'-','environment');
xlabel('time [s]');
ylabel('position [rad]');
legend('xs','xs','xm');

subplot(3,1,2);
hold on;
grid on;
plot(out.dot_xs.Time(1:end), out.dot_xs.data(1:end),'LineWidth', 1.2);
hold on, plot(out.dot_xm.Time(1:end), out.dot_xm.data(1:end),'LineWidth', 1.2);
xlabel('time [s]');
ylabel('velocity [rad/s]');
legend('d\_xs','d\_xm');

subplot(3,1,3);
hold on;
grid on;
plot(out.fh.Time(1:end), out.fh.data(1:end),'LineWidth', 1.2);
hold on, plot(out.fe.Time(1:end),out.fe.data(1:end),'LineWidth', 1.2);
xlabel('time [s]');
ylabel('forces [Nm]');
legend('fm', 'fs');