h_fig = figure('Name', 'ee_pose_plot');
plot(out.ee_pose.Data(:,8),out.ee_pose.Data(:,1),'LineWidth',2.2,'LineStyle','-.');
hold on;
plot(out.ee_pose.Data(:,8),out.ee_pose.Data(:,2),'LineWidth',2.2,'LineStyle','-.');
hold on;
plot(out.ee_pose.Data(:,8),out.ee_pose.Data(:,3),'LineWidth',2.2,'LineStyle','-.');
hold on;
set(gca,'XLim',[0 out.ee_pose.Data(end,8)],'FontSize',16);
set(gca,'YLim',[-1, 1],'FontSize',16);
set(gca,'YTick',-1:0.25:1);
legend({'$x$','$y$','$z$'},'FontSize',16,'Interpreter','latex','NumColumns',4);
xlabel('Time (s)','FontSize',20,'Interpreter','tex');
ylabel('Position (pixel)','FontSize',20,'Interpreter','tex');
hold on;
saveas(h_fig, h_fig.Name, 'fig')
saveas(h_fig, h_fig.Name, 'svg')
