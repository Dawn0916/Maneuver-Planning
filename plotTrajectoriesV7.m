figure(1);

    box on;
    plot([0 2270], [0 0], '-k', 'LineWidth', 1.5);
    hold on;
    plot([0 2270], [w_lane w_lane], '--k','LineWidth', 1.5);
    hold on;
    plot([0 2270], [2*w_lane 2*w_lane], '-k', 'LineWidth', 1.5);
    hold on;
    rectangle('Position', [Xv1(1)-l/2 Xv1(2)-b/2 l b], 'FaceColor', 'r');
    hold on;
    rectangle('Position', [Xv2(1)-l/2 Xv2(2)-b/2 l b], 'FaceColor', 'b');
    hold on;
    rectangle('Position', [Xv3(1)-l/2 Xv3(2)-b/2 l b], 'FaceColor', 'k');
    hold on;
    rectangle('Position', [Xv4(1)-l/2 Xv4(2)-b/2 l b], 'FaceColor', 'c');
    hold on;
    rectangle('Position', [Xv5(1)-l/2 Xv5(2)-b/2 l b], 'FaceColor', 'g');
    hold on;
    rectangle('Position', [Xv6(1)-l/2 Xv6(2)-b/2 l b], 'FaceColor', 'm');
    hold on;
    rectangle('Position', [Xv7(1)-l/2 Xv7(2)-b/2 l b], 'FaceColor', 'y');
    hold on;
    axis equal;
    xlim([Xv1(end,1)-2200 Xv1(end,1)+10]);
    ylim([-9 19.5])
    %         set(gcf, 'Position', [1000 1000 1250 1250]);
    set(gcf, 'Color', 'w');
    set(gca, 'FontSize', 12);
    set(gca,'ytick',[0:10:10]);
    set(gca,'xtick',[200:10:1000]);
    xlabel('x in [m]', 'FontSize', 12);
    ylabel('y in [m]', 'FontSize', 12);
    title('Vehicles Configurations for the Simulation at Time Step k = 50', 'FontSize', 12)



function plotTrajectoriesV7(~, ~, ~, ~, ~, ~, ~, ~, ~)
    
    close 1;
    global Xv1;
    global Xv2;
    global Xv3;
    global Xv4;
    global Xv5;
    global Xv6;
    global Xv7;

    global l;
    global b;
%     global x_safe;
    global w_lane;
%     global cen_up_lane;
%     global cen_low_lane;
    
        
    i = 1;
    sizex = size(Xv7); 
    g = sizex(1);   %to get the number of states of the system
                    %from the number of rows
     figure(1);               
        hold on;
        plot([0 2270], [0 0], '-k', 'LineWidth', 1.5);
        hold on;
        plot([0 2270], [w_lane w_lane], '--k','LineWidth', 1.5);
        hold on;
        plot([0 2270], [2*w_lane 2*w_lane], '-k', 'LineWidth', 1.5);
        hold on;
        box on;
        while i < g
            if i < g
            rectangle('Position', [Xv1(i,1)-l/2 Xv1(i,2)-b/2 l b], 'EdgeColor', 'r', 'LineWidth', 1.25);
            hold on;
            rectangle('Position', [Xv2(i,1)-l/2 Xv2(i,2)-b/2 l b], 'EdgeColor', 'b', 'LineWidth', 1.25);
            hold on;
            rectangle('Position', [Xv3(i,1)-l/2 Xv3(i,2)-b/2 l b], 'EdgeColor', 'k', 'LineWidth', 1.25);
            hold on;
            rectangle('Position', [Xv4(i,1)-l/2 Xv4(i,2)-b/2 l b], 'EdgeColor', 'c', 'LineWidth', 1.25);
            hold on;
            rectangle('Position', [Xv5(i,1)-l/2 Xv5(i,2)-b/2 l b], 'EdgeColor', 'g', 'LineWidth', 1.25);
            hold on;
            rectangle('Position', [Xv6(i,1)-l/2 Xv6(i,2)-b/2 l b], 'EdgeColor', 'm', 'LineWidth', 1.25);
            hold on;
            rectangle('Position', [Xv7(i,1)-l/2 Xv7(i,2)-b/2 l b], 'EdgeColor', 'y', 'LineWidth', 1.25);
            hold on;
            i=i+1;
            end
        end
        hold on;
        rectangle('Position', [Xv1(end,1)-l/2 Xv1(end,2)-b/2 l b], 'FaceColor', 'r');
        hold on;
        rectangle('Position', [Xv2(end,1)-l/2 Xv2(end,2)-b/2 l b], 'FaceColor', 'b');
        hold on;
        rectangle('Position', [Xv3(end,1)-l/2 Xv3(end,2)-b/2 l b], 'FaceColor', 'k');
        hold on;
        rectangle('Position', [Xv4(end,1)-l/2 Xv4(end,2)-b/2 l b], 'FaceColor', 'c');
        hold on;
        rectangle('Position', [Xv5(end,1)-l/2 Xv5(end,2)-b/2 l b], 'FaceColor', 'g');
        hold on;
        rectangle('Position', [Xv6(end,1)-l/2 Xv6(end,2)-b/2 l b], 'FaceColor', 'm');
        hold on;
        rectangle('Position', [Xv7(end,1)-l/2 Xv7(end,2)-b/2 l b], 'FaceColor', 'y');
        axis equal;

        Xv=[Xv1(end,1) Xv2(end,1) Xv3(end,1) Xv4(end,1) Xv5(end,1) Xv6(end,1) Xv7(end,1)];
        xlim([max(Xv)-2200 max(Xv)+10]);
        ylim([-9 19.5]);
        set(gcf, 'Color', 'w');
        set(gca, 'FontSize', 12);
        set(gca,'ytick',[0:10:10]);
        set(gca,'xtick',[200:10:1000]);
        xlabel('x in [m]', 'FontSize', 12);
        ylabel('y in [m]', 'FontSize', 12);

end



