function [ttc,tiv,p1,p2,p]=ttc_tiv()
  global Xv1;
  global Vv1;
  global Xv2;
  global Vv2;
  Delta_v=2;
  
  %% Speed Keeping
  %% ttc
  ttc=abs((Xv2(end,1)-Xv1(end,1))/(Vv2(end,1)-Vv1(end,1)));
  
  %% tiv
  if (Xv2(end,1)-Xv1(end,1)>0
      tiv=abs((Xv2(end,1)-Xv1(end,1))/(Vv1(end,1))); %Car2 is in front of  Car1
  else
      tiv=abs((Xv2(end,1)-Xv1(end,1))/(Vv2(end,1))); %Car1 is in front of  Car2
  end
  
  %% ttc~p
  if ttc<1
      p1=1;
  else
      if ttc<=10
          p1=(-1/9)*ttc+(10/9);
      else
          p1=0;
      end
  end
  
  %% tiv~p
  if tiv<0.5
      p2=1;
  else
      if tiv<=1
          p2=-tiv+1.5;
      else
          if tiv<=2
              p2=(-0.5)*tiv+1;
          else
              p2=0;
          end
      end
  end
  p=max(p1,p2);
  
  
  %% Accelerating
  
  
  
  %% Decelerating
  
  
  