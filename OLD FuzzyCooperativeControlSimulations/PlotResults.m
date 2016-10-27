%Robot Trajectory plotting Script


close all;
% Collect Data
R1xr = Robot1.signals.values(:,1);%Robot1.signals(1,1).values(:,1);
R1yr = Robot1.signals.values(:,2);%Robot1.signals(1,2).values(:,1);
R1x = Robot1.signals.values(:,3);%Robot1.signals(1,3).values(:,1);
R1y = Robot1.signals.values(:,4);%Robot1.signals(1,4).values(:,1);
R1t = Robot1.time(:,1);
R1_V = Robot1_VW.signals.values(:,1);
R1_W = Robot1_VW.signals.values(:,2);
R1_Yerr = Robot1_Yerr.signals.values(:,1);



R2xr = Robot2.signals.values(:,1);%Robot1.signals(1,1).values(:,1);
R2yr = Robot2.signals.values(:,2);%Robot1.signals(1,2).values(:,1);
R2x = Robot2.signals.values(:,3);%Robot1.signals(1,3).values(:,1);
R2y = Robot2.signals.values(:,4);%Robot1.signals(1,4).values(:,1);
R2t = Robot2.time(:,1);
R2_V = Robot2_VW.signals.values(:,1);
R2_W = Robot2_VW.signals.values(:,2);
R2_Yerr = Robot2_Yerr.signals.values(:,1);


R3xr = Robot3.signals.values(:,1);%Robot1.signals(1,1).values(:,1);
R3yr = Robot3.signals.values(:,2);%Robot1.signals(1,2).values(:,1);
R3x = Robot3.signals.values(:,3);%Robot1.signals(1,3).values(:,1);
R3y = Robot3.signals.values(:,4);%Robot1.signals(1,4).values(:,1);
R3t = Robot3.time(:,1);
R3_V = Robot3_VW.signals.values(:,1);
R3_W = Robot3_VW.signals.values(:,2);
R3_Yerr = Robot3_Yerr.signals.values(:,1);



c = 2:2000:21000;
c = [c, 10000];

figure(1);
hold on;
plot(R1x,R1y,'b');
plot(R1xr,R1yr,'r-.','LineWidth',1);
plot(R2x,R2y,'r');
plot(R2xr,R2yr,'b-.','LineWidth',1);
plot(R3x,R3y,'g');
plot(R3xr,R3yr,'m-.','LineWidth',1);
plot(R1x(c),R1y(c),'o', 'markerSize',10);
plot(R2x(c),R2y(c),'o', 'markerSize',10);
plot(R3x(c),R3y(c),'o', 'markerSize',10);
%plot(R3xr(c),R3yr(c),'o', 'markerSize',10);
% subplot(2,1,1); hold on; plot(R1x,R1y,'b');
% plot(R1x(c),R1y(c),'o', 'markerSize',10);
% subplot(2,1,2); hold on; plot(R2x,R2y,'r');
% plot(R2x(c),R2y(c),'o', 'markerSize',10);
%axis([-1 26 0 12]);
xlabel('X(m)');
ylabel('Y(m)');
legend('Robot 1','Robot 1(Ref)','Robot 2','Robot 2(Ref)','Robot 3','Robot 3(Ref)');
hold off;

figure(2)
subplot(3,1,1); hold on; 
plot(R1xr,R1yr,'r--');
plot(R1x,R1y,'b');
xlabel('X(m)');
ylabel('Y(m)');
%legend('Ref Trajactory','Actual Trajactory');
title('Robot 1');
subplot(3,1,2); hold on; 
plot(R2xr,R2yr,'r--');
plot(R2x,R2y,'b');
xlabel('X(m)');
ylabel('Y(m)');
%legend('Ref Trajactory','Actual Trajactory');
title('Robot 2');
subplot(3,1,3); hold on; 
plot(R3xr,R3yr,'r--');
plot(R3x,R3y,'b');
xlabel('X(m)');
ylabel('Y(m)');
%legend('Ref Trajactory','Actual Trajactory');
title('Robot 3');


figure(3);
hold on;
plot(R1t,R1_V,'b');
plot(R2t,R2_V,'r');
plot(R3t,R3_V,'m');
legend('V1','V2','V3');
title('Fuzzy Controller Outputs V(m/sec) Linear Velocities');
xlabel('Time(Secs)');
ylabel('V(m/sec)');

figure(4);
hold on;
plot(R1t,R1_W,'b');
plot(R2t,R2_W,'r');
plot(R3t,R3_W,'m');
legend('W1','W2','W3');
title('Fuzzy Controller Outputs W(rad/sec) Angular Velocities');
xlabel('Time(Secs)');
ylabel('W(rad/sec)');

figure(5);
hold on;
plot(R1t,R1_Yerr,'b');
plot(R2t,R2_Yerr,'r');
plot(R3t,R3_Yerr,'m');
legend('Yerr 1','Yerr 2','Yerr 3');
title('Path Following Errors Yerr');
xlabel('Time(Secs)');
ylabel('Yerr)');
% figure(2);
% hold on;
% plot(R1t,R1y,'b');
% plot(R2t,R2y,'r');
% plot(R1t(c),R1y(c),'o', 'markerSize',10);
% plot(R2t(c),R2y(c),'o', 'markerSize',10);
% xlabel('t(sec)');
% ylabel('Y(m)');
% legend('Robot 1','Robot 2');
% hold off;

