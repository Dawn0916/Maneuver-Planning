%%% Vehicle4 and Vehicle6 change Lane

% simulation of the highway scenario
% function
addpath('./nmpcroutine');
clear all;
close all;

global T; % sampling time of MPC
global l; % length of vehicle
global b; % width of vehicle
global Xv1; % position of the first vehicle ( center of a rectangle )
global Xv2; % position of the second vehicle
global y_ref; %Goal lane
global Vv1; % velocity of the first vehicle
global Vv2; % velocity of the second vehicle

% lane width
global w_lane;

% lane centers;
global cen_up_lane;
global cen_low_lane;

% Safe distance
global x_safe;
global x_follow;

% tracking of the time skips
global iterations;

mpciterations = 1; % number of MPC iterations
N             = 25; % prediction horizon
T             = 0.2;
tmeasure      = 0.0;
x1measure = [+35.000 +0.000 +10 +2.625];
x2measure = [+20.050 +0.000 +100 +7.875];
% y_ref0=[2.625 7.875];
y_ref=[];

u01 = zeros(2,N);
u02 = zeros(2,N);

tol_opt       = 1e-6;
opt_option    = 0;
iprint        = 5;

%     type          = 'differential equation';
type          = 'difference equation';
atol_ode_real = 1e-8;
rtol_ode_real = 1e-8;
atol_ode_sim  = 1e-2;
rtol_ode_sim  = 1e-2;

l = 4.7;
b = 1.83;

Xv1=x1measure(end,3:4); %position of the first vehicle
Xv2=x2measure(end,3:4); %position of the second vehicle
% y_ref=y_ref0(end,1);
Vv1=x1measure(end,1:2); %velocity of the first vehicle
Vv2=x2measure(end,1:2); %velocity of the second vehicle

x_safe = 40; % safety distance
x_follow=10;
% set lane width
w_lane = 5.25;

% calculate lane centers (for plot of target vehicle)
cen_up_lane = w_lane + w_lane/2;
cen_low_lane = w_lane/2;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% analysis of the two vehicles simultaneously
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

t01 = tmeasure;
t02 = tmeasure;


tic
for iter=1:125
    
    iter;
    iterations(end+1,:) = iter;
    
    [t01, x1measure, u01] = nmpc(@runningcosts1, @terminalcosts1, @constraints1, ...
        @terminalconstraints1, @linearconstraints1, @system, ...
        mpciterations, N, T, t01(end,:), x1measure(end,:), u01, ...
        tol_opt, opt_option, ...
        type, atol_ode_real, rtol_ode_real, atol_ode_sim, rtol_ode_sim, ...
        iprint, @printHeader, @printClosedloopData); %for the first vehicle
            
    [t02, x2measure, u02] = nmpc(@runningcosts2, @terminalcosts2, @constraints2, ...
        @terminalconstraints2, @linearconstraints2, @system, ...
        mpciterations, N, T, t02(end,:), x2measure(end,:), u02, ...
        tol_opt, opt_option, ...
        type, atol_ode_real, rtol_ode_real, atol_ode_sim, rtol_ode_sim, ...
        iprint, @printHeader, @printClosedloopData); %for the second vehicle
    
    Xv1(end+1,:) = x1measure(end,3:4);
    Vv1(end+1,:) = x1measure(end,1:2);
    Xv2(end+1,:) = x2measure(end,3:4);
    Vv2(end+1,:) = x2measure(end,1:2);
%     y_ref(end+1,:)=y_ref0(end,1);
  
%     plotTrajectoriesV2();
    
    u01 = [u01(:,2:size(u01,2)) u01(:,size(u01,2))];
    u02 = [u02(:,2:size(u02,2)) u02(:,size(u02,2))]; 
%     Xv1(end,1) - Xv2(end,1);
        
end
toc
rmpath('./nmpcroutine');

plotV2

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definition of the NMPC functions for the first vehicles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function cost = runningcosts1(t, x, u, kT)
    global b;
    global w_lane;
    global Xv1;
    global Vv1;
    global Xv2;
    global Vv2;
    global x_safe;
%     global y_ref;
    if x(3)-(Xv2(end,1)+Vv2(end,1)*kT)<0  %ego vehicle is behind the other vehicle
        if -(x(3)-(Xv2(end,1)+Vv2(end,1)*kT))> (2*x_safe) %can stay in first lane
            cost = 1*u(1)^2 + 0.1*u(2)^2 + 100*(x(1) - 35)^2;
%             y_ref=[;2.625];
        else
            if -(x(3)-(Xv2(end,1)+Vv2(end,1)*kT))>1.5* x_safe %change to second lane
            cost = 1*u(1)^2 + 0.1*u(2)^2 + 100*(x(1) - 35)^2 + 10*(x(4)-7.875)^2;
%             y_ref=[;7.875];
            else %change to third lane
            cost = 1*u(1)^2 + 0.1*u(2)^2 + 100*(x(1) - 35)^2 + 10*(x(4)-13.125)^2;
%             y_ref=[;13.125];
            end
        end
    else %ego vehicle is in front of the other vehicle
        if x(3)-(Xv2(end,1)+Vv2(end,1)*kT)< 1.5* x_safe %stay in third lane
            cost = 1*u(1)^2 + 0.1*u(2)^2 + 100*(x(1) - 35)^2 + 10*(x(4)-13.125)^2;
%             y_ref=[;13.125];
        else
            if x(3)-(Xv2(end,1)+Vv2(end,1)*kT)< 2*x_safe %change to second lane
                cost = 1*u(1)^2 + 0.1*u(2)^2 + 100*(x(1) - 35)^2 + 10*(x(4)-7.875)^2;
%                  y_ref=[;7.875];
            else %change to first lane 
                cost = 1*u(1)^2 + 0.1*u(2)^2 + 100*(x(1) - 35)^2 + 10*(x(4)-2.625)^2;
%                  y_ref=[;2.625];
            end
        end
    end
%     y_ref
end

function cost = terminalcosts1(t, x, kT)
    global b;
    global w_lane;
    global Xv1;
    global Vv1;
    global Xv2;
    global Vv2;
    global x_safe;
%       cost = 100*(x(1) - 35)^2;
     if x(3)-(Xv2(end,1)+Vv2(end,1)*kT)<0  %ego vehicle is behind the other vehicle
        if -(x(3)-(Xv2(end,1)+Vv2(end,1)*kT))> (2*x_safe) %can stay in first lane
            cost = 100*(x(1) - 35)^2;
        else
            if -(x(3)-(Xv2(end,1)+Vv2(end,1)*kT))>1.5* x_safe %change to second lane
            cost =100*(x(1) - 35)^2 + 10*(x(4)-7.875)^2;
            else %change to third lane
            cost =100*(x(1) - 35)^2 + 10*(x(4)-13.125)^2;
            end
        end
    else %ego vehicle is in front of the other vehicle
        if x(3)-(Xv2(end,1)+Vv2(end,1)*kT)< 1.5* x_safe %stay in third lane
            cost = 100*(x(1) - 35)^2 + 10*(x(4)-13.125)^2;
        else
            if x(3)-(Xv2(end,1)+Vv2(end,1)*kT)< 2*x_safe %change to second lane
                cost = 100*(x(1) - 35)^2 + 10*(x(4)-7.875)^2;
            else %change to first lane
                cost = 100*(x(1) - 35)^2 + 10*(x(4)-2.625)^2;
            end
        end
    end
end

function [c,ceq] = constraints1(t, x, u, kT)

    global b;
    global w_lane;
    global Xv1;
    global Vv1;
    global Xv2;
    global Vv2;

    global x_safe;
    v_x_min = 50/3.6;
    v_x_max = 70;
    c   = [];
    c(1) = x(4)-3*w_lane+b/2;
    c(2) = -x(4)+b/2;
    c(3) = -x(1)+v_x_min;
    c(4) = x(1)-v_x_max;
%     c(5) = -((((Xv2(end,1)+Vv2(end,1)*kT)-x(3))^2)/(5^2)+(((Xv2(end,2)+Vv2(end,2)*kT)-x(4))^2)/((w_lane/2)^2))+1;
    
    if (x(4)>w_lane)&&(x(4)<2*w_lane)
        c(5) = -((Xv2(end,1)+Vv2(end,1)*kT)-x(3))^2+x_safe^2;
    else
        c(5) = -x(3);
    end
    c(6) = -x(3);
    ceq = [];
end

function [c,ceq] = terminalconstraints1(t, x, kT)

    global b;
    global w_lane;
    global Xv1;
    global Vv1;
    global Xv2;
    global Vv2;
 
    global x_safe;
        
    v_x_min = 50/3.6;
    v_x_max = 70;
    c   = [];
    c(1) = x(4)-3*w_lane+b/2;
    c(2) = -x(4)+b/2;
    c(3) = -x(1)+v_x_min;
    c(4) = x(1)-v_x_max;
    c(5) = -((((Xv2(end,1)+Vv2(end,1)*kT)-x(3))^2)/(5^2)+(((Xv2(end,2)+Vv2(end,2)*kT)-x(4))^2)/((w_lane/2)^2))+1;
    if (x(4)>w_lane)&&(x(4)<2*w_lane)
        c(5) = -((Xv2(end,1)+Vv2(end,1)*kT)-x(3))^2+x_safe^2;
    else
        c(5) = -x(3);
    end
    c(6) = -x(3);
    ceq = [];
    
end

function [A, b, Aeq, beq, lb, ub] = linearconstraints1(t, x, u)
    a_min = -9; %minimum deceleration 
    a_max = 6;  %maximum acceleration
    a_ymin= -1*0.5; %minimum steering rate
    a_ymax= 1*0.5;  %maximum steering rate
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    lb  = [a_min a_ymin];
    ub  = [a_max a_ymax];
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definition of the NMPC functions for the second vehicles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function cost = runningcosts2(t, x, u, kT)
    global Xv1;
    global Vv1;
    global Xv2;
    global Vv2;

    global x_safe;
    global x_follow;
%     cost = 1*u(1)^2 + 0.1*u(2)^2 + 100*(x(1) - 20)^2 + 200*(-x(3)+(Xv1(end,1)+Vv1(end,1)*kT)-x_follow)^2;
    cost = 1*u(1)^2 + 0.1*u(2)^2 + 100*(x(1) - 20)^2 +10*(x(4)-7.875)^2;
end

function cost = terminalcosts2(t, x, kT)
    global Xv1;
    global Vv1;
    global Xv2;
    global Vv2;


    global x_safe;
    global x_follow;
     cost = 100*(x(1) - 20)^2 +10*(x(4)-7.875)^2;
end

function [c,ceq] = constraints2(t, x, u, kT)

    global b;
    global w_lane;
    global Xv1;
    global Vv1;
    global Xv2;
    global Vv2;

    global x_safe;
        
    v_x_min = 60/3.6;
    v_x_max = 70;
    c   = [];
    c(1) = x(4)-3*w_lane+b/2;
    c(2) = -x(4)+b/2;
    c(3) = -x(1)+v_x_min;
    c(4) = x(1)-v_x_max;
    c(5) = -((((Xv1(end,1)+Vv1(end,1)*kT)-x(3))^2)/(5^2)+(((Xv1(end,2)+Vv1(end,2)*kT)-x(4))^2)/((w_lane/2)^2))+1;
%     if ((Xv1(end,2)>w_lane)&&((Xv1(end,2)<2*w_lane)
%         c(5) = -((Xv1(end,1)+Vv1(end,1)*kT)-x(3))^2+x_safe^2;
%     else
%         c(5) = -x(3);
%     end
    c(6) = -x(3);
    ceq = [];
    
end

function [c,ceq] = terminalconstraints2(t, x, kT)

    global b;
    global w_lane;
    global Xv1;
    global Vv1;
    global Xv2;
    global Vv2;

    
    global x_safe;
        
    v_x_min = 60/3.6;
    v_x_max = 70;
    c   = [];
    c(1) = x(4)-3*w_lane+b/2;
    c(2) = -x(4)+b/2;
    c(3) = -x(1)+v_x_min;
    c(4) = x(1)-v_x_max;
%     c(5) = -((((Xv1(end,1)+Vv1(end,1)*kT)-x(3))^2)/(5^2)+(((Xv1(end,2)+Vv1(end,2)*kT)-x(4))^2)/((w_lane/2)^2))+1;
%     if ((Xv1(end,2)>w_lane)&&((Xv1(end,2)<2*w_lane)
%         c(5) = -((Xv1(end,1)+Vv1(end,1)*kT)-x(3))^2+x_safe^2;
%     else
%         c(5) = -x(3);
%     end
    c(6) = -x(3);
    ceq = [];
    
end

function [A, b, Aeq, beq, lb, ub] = linearconstraints2(t, x, u)
    a_min = -9; %minimum deceleration 
    a_max = 6;  %maximum acceleration
    a_ymin= -1*0.5; %minimum steering rate
    a_ymax= 1*0.5;  %maximum steering rate
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    lb  = [a_min a_ymin];
    ub  = [a_max a_ymax];
end




function plotTrajectoriesV2()
    
%     close 1;
    global Xv1;
    global Xv2;
    global l;
    global b;
%     global x_safe;
    global w_lane;
%     global cen_up_lane;
%     global cen_low_lane;
    set(gcf,'Units','normalized','OuterPosition',[0.02 0.1 0.95 0.25]);
    set(0,'DefaultAxesFontName', 'Times New Roman')
    set(0,'DefaultAxesFontSize', 12)
    fontsize_labels=12;
% line_width=1.5;
    set(gcf,'PaperPositionMode','auto')
    set(gcf, 'Color', 'w');
        
    i = 1;
    sizex = size(Xv2); 
    g = sizex(1);   %to get the number of states of the system
                    %from the number of rows
     figure(1);               
%         hold on;
        plot([0 1000], [0 0], '-k', 'LineWidth', 1.5);
        hold on;
        plot([0 1000], [w_lane w_lane], '--k','LineWidth', 1.5);
        hold on;
        plot([0 1000], [2*w_lane 2*w_lane], '--k', 'LineWidth', 1.5);
        hold on;
        plot([0 1000], [3*w_lane 3*w_lane], '-k', 'LineWidth', 1.5);
        hold on;
%     hold on;
        box on;
        while i < g
            if i < g
            rectangle('Position', [Xv1(i,1)-l/2 Xv1(i,2)-b/2 l b], 'EdgeColor', 'r', 'LineWidth', 1.25);
            hold on;
            rectangle('Position', [Xv2(i,1)-l/2 Xv2(i,2)-b/2 l b], 'EdgeColor', 'b', 'LineWidth', 1.25);
            hold on;
            i=i+1;
            end
        end
%         hold on;
        rectangle('Position', [Xv1(end,1)-l/2 Xv1(end,2)-b/2 l b], 'FaceColor', 'r');
        hold on;
        rectangle('Position', [Xv2(end,1)-l/2 Xv2(end,2)-b/2 l b], 'FaceColor', 'b');
%         hold on;
        axis equal;

        Xv=[Xv1(end,1) Xv2(end,1)];
%         x_max=max(Xv);
%         x_min=min(Xv);
%         xlim([x_min-20 x_max+50]);
        xlim([0 1000]);
        ylim([-0.25 3*w_lane+0.25]);
        set(gcf,'Position',[50 600 3000 200])
        set(gcf, 'Color', 'w');
        set(gca, 'FontSize', 12);
        set(gca,'ytick',[0:10:10]);
        set(gca,'xtick',[0:100:1000]);
        xlabel('$x$(m)','interpreter','latex','FontSize',fontsize_labels);
        ylabel('$y$(m)','interpreter','latex','FontSize',fontsize_labels);
%         save 2
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definition of the NMPC functions common for the two vehicles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dx = system(t, x, u, T)

    dx    = zeros(4,1);
    
    % double integrator model of the lead vehicle (used for prediction)
    dx(1) = x(1)+u(1)*T;                                                 %xd
    dx(2) = x(2)+u(2)*T;                                                 %yd
    dx(3) = x(3)+x(1)*T+T^2*u(1)/2;                                     %x                                               
    dx(4) = x(4)+x(2)*T+T^2*u(2)/2;                                     %y
    
    %Needed for TimeDiscrete
    dx=dx.';
    
end

function printHeader()
%     fprintf('   k  |    u(1)     u(2)      V(1)     V(2)    X(1)    X(2)    Time\n');
%     fprintf('--------------------------------------------------\n');
end

function printClosedloopData(mpciter, u, x, t_Elapsed)
    
    global iterations;
    
    fprintf(' %3d  | %+11.6f %+11.6f %+6.3f %+6.3f %+6.3f  %+6.3f  %+6.3f', iterations(end,:), ...
            u(1), u(2), x(1), x(2), x(3), x(4) ,t_Elapsed);
end