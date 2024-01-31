%test ttc
%i is in front of j
x_i_j=10;  
v_i=1:1:90;
v_j=20;
y1=x_i_j./(v_i-v_j);
y2=x_i_j./v_i;
figure(1)
plot(v_i,y1);
figure(2)
plot(v_i,y2);

%j is in front of i
y3=-x_i_j./(v_i-v_j);
y4=x_i_j./v_i;
figure(3)
plot(v_i,y3);
figure(4)
plot(v_i,y4);