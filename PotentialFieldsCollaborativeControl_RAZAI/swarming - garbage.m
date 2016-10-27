
% robots_trajectory=cell2mat(robot_Vec);
% 
% figure(1);
% plot(robots_trajectory(:,1),robots_trajectory(:,2),'k-<'),hold on, grid on,
% plot(robots_trajectory(:,3),robots_trajectory(:,4),'r-<'),hold on, grid on,
% plot(robots_trajectory(:,5),robots_trajectory(:,6),'b-<'),hold on, grid on,
% plot(robots_trajectory(:,7),robots_trajectory(:,8),'c-<'),hold on, grid on,



% %Plotting Initial position
% %     for i=1:number_of_robots
% %         dummy=robot(i);
% %         xy_pos=cell2mat(dummy);
% %         x_pos=xy_pos(1);
% %         y_pos=xy_pos(2);
% %         figure(1),plot(x_pos,y_pos,'*'), grid on,hold on, axis tight;
% %     end
% %
% robots_trajectory=cell2mat(robot_Vec);
% 
% figure(1);
% plot(robots_trajectory(:,1),robots_trajectory(:,2),'k-<'),hold on, grid on,
% plot(robots_trajectory(:,3),robots_trajectory(:,4),'r-<'),hold on, grid on,
% plot(robots_trajectory(:,5),robots_trajectory(:,6),'b-<'),hold on, grid on,
% plot(robots_trajectory(:,7),robots_trajectory(:,8),'c-<'),hold on, grid on,
%  plot(robots_trajectory(:,9),robots_trajectory(:,10),'y-<'),hold on, grid on,
% 
% %drawing a circle of radius=alpha
% count=1;
% for theta=0:0.1:2*pi
% x_circle(count)=(alpha+0.5)*cos(theta);
% y_circle(count)=(alpha+0.5)*sin(theta);
% count=count+1;
% end
% 
% x_circle=x_circle+hyp_robot(1);
% y_circle=y_circle+hyp_robot(2);
% 
% plot(x_circle,y_circle,'r-'),hold on,
% 
% 
% 
% % % % % %plotting a line between the final points
% % % % % for i=1:2:(number_of_robots*2)
% % % % %     
% % % % %     x1=robots_trajectory(end,i);
% % % % %     y1=robots_trajectory(end,i+1);
% % % % %     
% % % % %     if i<(number_of_robots*2-2)
% % % % %         x2=robots_trajectory(end,i+2);
% % % % %         y2=robots_trajectory(end,i+3);
% % % % %     else
% % % % %         x2=robots_trajectory(end,mod(i+2,number_of_robots*2));
% % % % %         y2=robots_trajectory(end,mod(i+3,number_of_robots*2));
% % % % %     end
% % % % %     
% % % % %     if (x1<=x2)
% % % % %         
% % % % %         x_points=x1:0.1:x2;
% % % % %     elseif (x1>x2)
% % % % %         x_points=x2:0.1:x1;
% % % % %         %     elseif x1==x2
% % % % %         %         x_points=x1*ones(1,100);
% % % % %     end
% % % % %     
% % % % %     if (abs(x1-x2)<=1e-3)
% % % % %         disp('I am here')
% % % % %         if y1<y2
% % % % %             y_points=y1:0.1:y2;
% % % % %         elseif y1>y2
% % % % %             y_points=y2:0.1:y1;
% % % % %         end
% % % % %         
% % % % %         x_points=x1*ones(1,length(y_points));
% % % % %     else
% % % % %         
% % % % %         slope=(y2-y1)/(x2-x1);
% % % % %         y_points=slope*(x_points-x1)+y1;
% % % % %     end
% % % % %      
% % % % %     
% % % % %     figure(1)
% % % % %     plot(x_points,y_points,'b--'),hold on,
% % % % % end