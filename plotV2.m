function plotV2()
    
%     close 1;
    global Xv1;% position of the first vehicle ( center of a rectangle )
    global Xv2;% position of the second vehicle
    global l; % length of vehicle
    global b;%width of vehicle
%     global x_safe;
    global w_lane;
%     global cen_up_lane;
%     global cen_low_lane;
      
figure(3)
% set(gcf,'Position',[50 500 600 100])  %scale for short figure
set(gcf,'Units','normalized','OuterPosition',[0.02 0.1 0.25 0.18]);
set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)
fontsize_labels=12;
% line_width=1.5;
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
y_count_end=1000;
% plot road
plot([-50 y_count_end],[0 0],'k-', 'LineWidth', 1.5)
hold on
plot([-50 y_count_end],[w_lane w_lane],'k--', 'LineWidth', 1.0)
hold on
plot([-50 y_count_end],[2*w_lane 2*w_lane],'k--', 'LineWidth', 1.0)
hold on
plot([-50 y_count_end],[3*w_lane 3*w_lane],'k-', 'LineWidth', 1.5)
hold on

% vehicles
xlabel('$x$(m)','interpreter','latex','FontSize',fontsize_labels);
ylabel('$y$(m)','interpreter','latex','FontSize',fontsize_labels);
axis([0,y_count_end,-1,3*w_lane+1]);
pause(1);

for n=1:length(Xv1)
    axis([Xv1(n,1)-50,Xv1(n,1)+200,-0.5,3*w_lane+0.5]);    %Slip axis for short figure
    Max_i=min(n,7);
    for i=1:1:Max_i 
    % Fill in the vehicle1 with red
    x1_position=[Xv1(n+1-i,1)-l/2,Xv1(n+1-i,1)-l/2,Xv1(n+1-i,1)+l/2,Xv1(n+1-i,1)+l/2]; % The x_position of Car1
    y1_position=[Xv1(n+1-i,2)-b/2,Xv1(n+1-i,2)+b/2,Xv1(n+1-i,2)+b/2,Xv1(n+1-i,2)-b/2]; %The y_position of Car1
    color1=[1 i/7 i/7];% Set the color for Car1 
    vehicle1=patch(x1_position,y1_position,color1);  %Colour the Car1
    
    % Frame of vehicle 1.
    line([Xv1(n+1-i,1)-l/2,Xv1(n+1-i,1)-l/2],[Xv1(n+1-i,2)+b/2,Xv1(n+1-i,2)-b/2],'Color','r');
    line([Xv1(n+1-i,1)-l/2,Xv1(n+1-i,1)+l/2],[Xv1(n+1-i,2)-b/2,Xv1(n+1-i,2)-b/2],'Color','r');
    line([Xv1(n+1-i,1)+l/2,Xv1(n+1-i,1)+l/2],[Xv1(n+1-i,2)-b/2,Xv1(n+1-i,2)+b/2],'Color','r');
    line([Xv1(n+1-i,1)+l/2,Xv1(n+1-i,1)-l/2],[Xv1(n+1-i,2)+b/2,Xv1(n+1-i,2)+b/2],'Color','r');
    end  
%   hold on
    for i=1:1:Max_i 
    % Fill in the vehicle2 with blue
    x1_position=[Xv2(n+1-i,1)-l/2,Xv2(n+1-i,1)-l/2,Xv2(n+1-i,1)+l/2,Xv2(n+1-i,1)+l/2]; % The x_position of Car1
    y1_position=[Xv2(n+1-i,2)-b/2,Xv2(n+1-i,2)+b/2,Xv2(n+1-i,2)+b/2,Xv2(n+1-i,2)-b/2]; %The y_position of Car1
    color2=[i/7 i/7 1];% Set the color for Car1 
    vehicle2=patch(x1_position,y1_position,color2);  %Colour the Car1
    % Frame of vehicle 2.
    line([Xv2(n+1-i,1)-l/2,Xv2(n+1-i,1)-l/2],[Xv2(n+1-i,2)+b/2,Xv2(n+1-i,2)-b/2],'Color','b');
    line([Xv2(n+1-i,1)-l/2,Xv2(n+1-i,1)+l/2],[Xv2(n+1-i,2)-b/2,Xv2(n+1-i,2)-b/2],'Color','b');
    line([Xv2(n+1-i,1)+l/2,Xv2(n+1-i,1)+l/2],[Xv2(n+1-i,2)-b/2,Xv2(n+1-i,2)+b/2],'Color','b');
    line([Xv2(n+1-i,1)+l/2,Xv2(n+1-i,1)-l/2],[Xv2(n+1-i,2)+b/2,Xv2(n+1-i,2)+b/2],'Color','b');
    end
    pause(0.05);
    frame=getframe(gcf);
    imind=frame2im(frame);
    [imind,cm] = rgb2ind(imind,256);
    if n==1
%          imwrite(imind,cm,'test.gif','gif', 'Loopcount',inf,'DelayTime',1e-4);
    else
         imwrite(imind,cm,'test.gif','gif','WriteMode','append','DelayTime',1e-4);
    end
end

figure(4)
% set(gcf,'Position',[50 500 600 100])  %scale for short figure
% set(gcf,'Units','normalized','OuterPosition',[0.02 0.5 0.95 0.25]);
set(gcf,'Units','normalized','OuterPosition',[0.02 0.1 0.25 0.18]);
set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)
fontsize_labels=12;
% line_width=1.5;
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
% fontsize_labels=12;
y_count_end=1000;
% plot road
plot([-200 y_count_end],[0 0],'k-', 'LineWidth', 1.5)
hold on
plot([-200 y_count_end],[w_lane w_lane],'k--', 'LineWidth', 1.0)
hold on
plot([-200 y_count_end],[2*w_lane 2*w_lane],'k--', 'LineWidth', 1.0)
hold on
plot([-200 y_count_end],[3*w_lane 3*w_lane],'k-', 'LineWidth', 1.5)
hold on

% vehicles
xlabel('$x$(m)','interpreter','latex','FontSize',fontsize_labels);
ylabel('$y$(m)','interpreter','latex','FontSize',fontsize_labels);
axis([0,y_count_end,-1,3*w_lane+1]);
pause(1);

for n=1:length(Xv1)
    axis([Xv1(n,1)-200,Xv1(n,1)+50,-0.5,3*w_lane+0.5]);    %Slip axis for short figure
    Max_i=min(n,7);
    for i=1:1:Max_i 
    % Fill in the vehicle1 with red
    x1_position=[Xv1(n+1-i,1)-l/2,Xv1(n+1-i,1)-l/2,Xv1(n+1-i,1)+l/2,Xv1(n+1-i,1)+l/2]; % The x_position of Car1
    y1_position=[Xv1(n+1-i,2)-b/2,Xv1(n+1-i,2)+b/2,Xv1(n+1-i,2)+b/2,Xv1(n+1-i,2)-b/2]; %The y_position of Car1
    color1=[1 i/7 i/7];% Set the color for Car1 
    vehicle1=patch(x1_position,y1_position,color1);  %Colour the Car1
    % Frame of vehicle 1.
    line([Xv1(n+1-i,1)-l/2,Xv1(n+1-i,1)-l/2],[Xv1(n+1-i,2)+b/2,Xv1(n+1-i,2)-b/2],'Color','r');
    line([Xv1(n+1-i,1)-l/2,Xv1(n+1-i,1)+l/2],[Xv1(n+1-i,2)-b/2,Xv1(n+1-i,2)-b/2],'Color','r');
    line([Xv1(n+1-i,1)+l/2,Xv1(n+1-i,1)+l/2],[Xv1(n+1-i,2)-b/2,Xv1(n+1-i,2)+b/2],'Color','r');
    line([Xv1(n+1-i,1)+l/2,Xv1(n+1-i,1)-l/2],[Xv1(n+1-i,2)+b/2,Xv1(n+1-i,2)+b/2],'Color','r');
    
    end  
    hold on
     for i=1:1:Max_i 

    % Fill in the vehicle2 with blue
    x1_position=[Xv2(n+1-i,1)-l/2,Xv2(n+1-i,1)-l/2,Xv2(n+1-i,1)+l/2,Xv2(n+1-i,1)+l/2]; % The x_position of Car1
    y1_position=[Xv2(n+1-i,2)-b/2,Xv2(n+1-i,2)+b/2,Xv2(n+1-i,2)+b/2,Xv2(n+1-i,2)-b/2]; %The y_position of Car1
    color2=[i/7 i/7 1];% Set the color for Car1 
    vehicle2=patch(x1_position,y1_position,color2);  %Colour the Car1
    % Frame of vehicle 2.
    line([Xv2(n+1-i,1)-l/2,Xv2(n+1-i,1)-l/2],[Xv2(n+1-i,2)+b/2,Xv2(n+1-i,2)-b/2],'Color','b');
    line([Xv2(n+1-i,1)-l/2,Xv2(n+1-i,1)+l/2],[Xv2(n+1-i,2)-b/2,Xv2(n+1-i,2)-b/2],'Color','b');
    line([Xv2(n+1-i,1)+l/2,Xv2(n+1-i,1)+l/2],[Xv2(n+1-i,2)-b/2,Xv2(n+1-i,2)+b/2],'Color','b');
    line([Xv2(n+1-i,1)+l/2,Xv2(n+1-i,1)-l/2],[Xv2(n+1-i,2)+b/2,Xv2(n+1-i,2)+b/2],'Color','b');
    
    end
    pause(0.05);
    frame=getframe(gcf);
    imind=frame2im(frame);
    [imind,cm] = rgb2ind(imind,256);
    if n==1
         imwrite(imind,cm,'test.gif','gif', 'Loopcount',inf,'DelayTime',1e-4);
    else
         imwrite(imind,cm,'test.gif','gif','WriteMode','append','DelayTime',1e-4);
    end
end
end