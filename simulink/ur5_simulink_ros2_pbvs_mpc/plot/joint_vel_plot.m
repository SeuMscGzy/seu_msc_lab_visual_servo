h_fig = figure('Name', 'joint_vel_plot');
plot(out.u.Data(:,7),out.u.Data(:,1),'LineWidth',2.2,'LineStyle','-.');
hold on;
plot(out.u.Data(:,7),out.u.Data(:,2),'LineWidth',2.2,'LineStyle','-.');
hold on;
plot(out.u.Data(:,7),out.u.Data(:,3),'LineWidth',2.2,'LineStyle','-.');
hold on;
plot(out.u.Data(:,7),out.u.Data(:,4),'LineWidth',2.2,'LineStyle','-.');
hold on;
plot(out.u.Data(:,7),out.u.Data(:,5),'LineWidth',2.2,'LineStyle','-.');
hold on;
plot(out.u.Data(:,7),out.u.Data(:,6),'LineWidth',2.2,'LineStyle','-.');
hold on;
set(gca,'XLim',[0 out.u.Data(end,7)],'FontSize',16);
set(gca,'YLim',[-0.6, 0.6],'FontSize',16);
set(gca,'YTick',-0.6:0.2:0.6);
legend({'$joint1_v$','$joint2_v$','$joint3_v$','$joint4_v$','$joint5_v$','$joint6_v$'},'FontSize',16,'Interpreter','latex','NumColumns',2);
xlabel('Time (s)','FontSize',20,'Interpreter','tex');
ylabel('Velocity (rad/s)','FontSize',20,'Interpreter','tex');
hold on;
saveas(h_fig, h_fig.Name, 'fig')
saveas(h_fig, h_fig.Name, 'svg')
