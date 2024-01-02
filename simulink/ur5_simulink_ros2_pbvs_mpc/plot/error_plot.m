h_fig = figure('Name', 'error_plot');
plot(out.error.Data(:,7),out.error.Data(:,1),'LineWidth',2.2,'LineStyle','-.');
hold on;
plot(out.error.Data(:,7),out.error.Data(:,2),'LineWidth',2.2,'LineStyle','-.');
hold on;
plot(out.error.Data(:,7),out.error.Data(:,3),'LineWidth',2.2,'LineStyle','-.');
hold on;
set(gca,'XLim',[0 out.error.Data(end,7)],'FontSize',16);
set(gca,'YLim',[-1, 1],'FontSize',16);
set(gca,'YTick',-1:0.25:1);
legend({'$x$','$y$','$z$'},'FontSize',16,'Interpreter','latex','NumColumns',4);
xlabel('Time (s)','FontSize',20,'Interpreter','tex');
ylabel('Error (m)','FontSize',20,'Interpreter','tex');
hold on;
saveas(h_fig, h_fig.Name, 'fig')
saveas(h_fig, h_fig.Name, 'svg')
