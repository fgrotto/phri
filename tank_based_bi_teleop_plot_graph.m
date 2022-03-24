figure();
subplot(4,1,1);
hold on;
grid on;
plot(out.xd.Time(1:end), out.xd.data(1:end),'LineWidth', 1.2);
hold on, plot(out.xs.Time(1:end), out.xs.data(1:end),'LineWidth', 1.2);
hold on, plot(out.xm.Time(1:end), out.xm.data(1:end),'LineWidth', 1.2);
yline(0.8,'-','environment');
xlabel('time [s]');
ylabel('position [rad]');
legend('xd','xs','xm');

subplot(4,1,2);
hold on;
grid on;
plot(out.dot_xs.Time(1:end), out.dot_xs.data(1:end),'LineWidth', 1.2);
hold on, plot(out.dot_xm.Time(1:end), out.dot_xm.data(1:end),'LineWidth', 1.2);
xlabel('time [s]');
ylabel('velocity [rad/s]');
legend('d\_xs','d\_xm');

subplot(4,1,3);
hold on;
grid on;
plot(out.fh.Time(1:end), out.fh.data(1:end),'LineWidth', 1.2);
hold on, plot(out.fe.Time(1:end),out.fe.data(1:end),'LineWidth', 1.2);
xlabel('time [s]');
ylabel('forces [Nm]');
legend('fm', 'fs');

subplot(4,1,4);
hold on;
grid on;
plot(out.H_master.Time(1:end), out.H_master.data(1:end),'LineWidth', 1.2);
hold on, plot(out.H_slave.Time(1:end),out.H_slave.data(1:end),'LineWidth', 1.2);
xlabel('time [s]');
ylabel('Energy [J]');
legend('H master', 'H slave');


%%
figure();
subplot(2,1,1);
hold on;
grid on;
hold on, plot(out.tau_tl_master.Time(1:end), out.tau_tl_master.data(1:end),'LineWidth', 1.2);
hold on, plot(out.tau_r_master.Time(1:end), out.tau_r_master.data(1:end),'LineWidth', 1.2);
xlabel('time [s]');
ylabel('forces [Nm]');
legend('tau tl master','tau r master');

subplot(2,1,2);
hold on;
grid on;
plot(out.tau_tl_slave.Time(1:end), out.tau_tl_slave.data(1:end),'LineWidth', 1.2);
hold on, plot(out.tau_r_slave.Time(1:end), out.tau_r_slave.data(1:end),'LineWidth', 1.2);
xlabel('time [s]');
ylabel('forces [Nm]');
legend('tau tl slave','tau r slave');
