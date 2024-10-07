close all;
clear all;

disp('Dynamic Window Approach sample program start!!')

%% 机器人的初期状态[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
% x=[0 0 pi/2 0 0]'; % 5x1矩阵 列矩阵  位置 0，0 航向 pi/2 ,速度、角速度均为0
x = [0 0 pi/10 0 0]'; 

% 下标宏定义 状态[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
POSE_X      = 1;  %坐标 X
POSE_Y      = 2;  %坐标 Y
YAW_ANGLE   = 3;  %机器人航向角
V_SPD       = 4;  %机器人速度
W_ANGLE_SPD = 5;  %机器人角速度 

goal = [10,10];   % 目标点位置 [x(m),y(m)]

% 障碍物位置列表 [x(m) y(m)]

obstacle=[0 2;
          2 4;
          2 5;      
          4 2;
%           4 4;
          5 4;
%            5 5;
          5 6;
          5 9
          8 8
          8 9
          7 9];
% obstacle=[0 2;
%           4 2;
%           4 4;
%           5 4;
%           5 5;
%           5 6;
%           5 9
%           8 8
%           8 9
%           7 9
%           6 5
%           6 3
%           6 8
%           6 7
%           7 4
%           9 8
%           9 11
%           9 6];

for i =-1
    for j = -1:12
        obstacle = [obstacle; [i,j]];
    end
end     
for i =12
    for j = -1:12
        obstacle = [obstacle; [i,j]];
    end
end 
for j =-2
    for i = -1:12
        obstacle = [obstacle; [i,j]];
    end
end 
for j=13
    for i= -1:12
        obstacle = [obstacle; [i,j]];
    end
end 

obstacleR = 0.5;% 冲突判定用的障碍物半径
global dt; 
dt = 0.1;% 离散时间/积分步长[s]

% 机器人运动学模型参数
% 最高速度m/s],最高旋转速度[rad/s],加速度[m/ss],旋转加速度[rad/ss],
% 速度分辨率[m/s],转速分辨率[rad/s]]
Kinematic = [1.0,toRadian(20.0),0.2,toRadian(50.0),0.01,toRadian(1)];
%定义Kinematic的下标含义
MD_MAX_V    = 1;%   最高速度m/s]
MD_MAX_W    = 2;%   最高旋转速度[rad/s]
MD_ACC      = 3;%   加速度[m/ss]
MD_VW       = 4;%   旋转加速度[rad/ss]
MD_V_RESOLUTION  = 5;%  速度分辨率[m/s]
MD_W_RESOLUTION  = 6;%  转速分辨率[rad/s]]

% 评价函数参数 [heading,dist,velocity,predictDT]
% 航向得分的比重、距离得分的比重、速度得分的比重、向前模拟轨迹的时间
evalParam = [0.08, 0.1 ,0.1, 3.0];
% evalParam = [2, 0.2 ,0.2, 3.0];
area      = [-3 14 -3 14];% 模拟区域范围 [xmin xmax ymin ymax]

% 模拟实验的结果
result.x=[];   %累积存储走过的轨迹点的状态值 
tic; % 估算程序运行时间开始

% movcount=0;
%% Main loop   循环运行 5000次 指导达到目的地 或者 5000次运行结束
% for i = 1:5000  
%     u = [0;0];
%     x = f(x,u);% 机器人移动到下一个时刻的状态量 根据当前速度和角速度推导 下一刻的位置和角度
% 
%     % 历史轨迹的保存
%     result.x = [result.x; x'];  %最新结果 以列的形式 添加到result.x
% 
%     % 是否到达目的地
%     if norm(x(POSE_X:POSE_Y)-goal')<0.25   % norm函数来求得坐标上的两个点之间的距离
%         disp('==========Arrive Goal!!==========');break;
%     end
% 
%     %====Animation====
%     hold off;               % 关闭图形保持功能。 新图出现时，取消原图的显示。
%     ArrowLength = 0.5;      % 箭头长度
% 
%     % 机器人
%     % quiver(x,y,u,v) 在 x 和 y 中每个对应元素对组所指定的坐标处将向量绘制为箭头
%     % quiver(x(POSE_X), x(POSE_Y), ArrowLength*cos(x(YAW_ANGLE)), ArrowLength*sin(x(YAW_ANGLE)), 'ok'); 
%     % 绘制机器人当前位置的航向箭头
%     hold on;                                                     
%     %启动图形保持功能，当前坐标轴和图形都将保持，从此绘制的图形都将添加在这个图形的基础上，并自动调整坐标轴的范围
% 
%     plot(result.x(:,POSE_X),result.x(:,POSE_Y),'-b');hold on;    % 绘制走过的所有位置 所有历史数据的 X、Y坐标
%     plot(goal(1),goal(2),'*r');hold on;                          % 绘制目标位置
% 
%     %plot(obstacle(:,1),obstacle(:,2),'*k');hold on;              % 绘制所有障碍物位置
%     DrawObstacle_plot(obstacle,obstacleR);
% end

DrawObstacle_plot(obstacle,obstacleR);
toc  %输出程序运行时间  形式：时间已过 ** 秒。

%% degree to radian
function radian = toRadian(degree)
radian = degree/180*pi;

end

%% radian to degree
function degree = toDegree(radian)
degree = radian/pi*180;

end

%% Motion Model 根据当前状态推算下一个控制周期（dt）的状态
% u = [vt; wt];当前时刻的速度、角速度 x = 状态[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
function x = f(x, u)
global dt;
F = [1 0 0 0 0
     0 1 0 0 0
     0 0 1 0 0
     0 0 0 0 0
     0 0 0 0 0];
 
B = [dt*cos(x(3)) 0
    dt*sin(x(3)) 0
    0 dt
    1 0
    0 1];
 
x= F*x+B*u;  
end

%% 绘制所有障碍物位置
% 输入参数：obstacle 所有障碍物的坐标   obstacleR 障碍物的半径
function [] = DrawObstacle_plot(obstacle,obstacleR)
r = obstacleR; 
theta = 0:pi/20:2*pi;
for id=1:length(obstacle(:,1))
    x = r * cos(theta) + obstacle(id,1); 
    y = r  *sin(theta) + obstacle(id,2);
    plot(x,y,'-m');hold on; 
end

end
