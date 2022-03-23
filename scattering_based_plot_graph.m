figure();
subplot(3,1,1);
hold on;
grid on;
plot(out.xd.Time(1:end), out.xd.data(1:end),'LineWidth', 1.2);
hold on, plot(out.xs.Time(1:end), out.xs.data(1:end),'LineWidth', 1.2);
hold on, plot(out.xm.Time(1:end), out.xm.data(1:end),'LineWidth', 1.2);
yline(0.8,'-','environment');
xlabel('time [s]');
ylabel('position [rad]');
legend('xd','xs','xm');

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

%%
% figure();
% subplot(2,1,1);
% hold on;
% grid on;
% hold on, plot(out.fl.Time(1:end), out.fl.data(1:end),'LineWidth', 1.2);
% hold on, plot(out.fr.Time(1:end), out.fr.data(1:end),'LineWidth', 1.2);
% xlabel('time [s]');
% ylabel('forces [Nm]');
% legend('fl','fr');
% 
% subplot(2,1,2);
% hold on;
% grid on;
% plot(out.dot_xl.Time(1:end), out.dot_xl.data(1:end),'LineWidth', 1.2);
% hold on, plot(out.dot_xr.Time(1:end), out.dot_xr.data(1:end),'LineWidth', 1.2);
% xlabel('time [s]');
% ylabel('velocity [rad/s]');
% legend('d\_xl','d\_xr');
