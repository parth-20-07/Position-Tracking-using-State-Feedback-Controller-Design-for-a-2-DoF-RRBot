rosshutdown;
clear; close; clc;
% ROS Setup
rosinit;
j1_effort = rospublisher("/rrbot/joint1_effort_controller/command");
j2_effort = rospublisher("/rrbot/joint2_effort_controller/command");
JointStates = rossubscriber("/rrbot/joint_states");
tau1 = rosmessage(j1_effort);
tau2 = rosmessage(j2_effort);
tau1.Data = 0;
tau2.Data = 0;
send(j1_effort,tau1);
send(j2_effort,tau2);
client = rossvcclient('/gazebo/set_model_configuration');
req = rosmessage(client);
req.ModelName = 'rrbot';
req.UrdfParamName = 'robot_description';
req.JointNames = {'joint1','joint2'};
req.JointPositions = [deg2rad(30), deg2rad(45)];
resp = call(client,req,'Timeout',3);
tic;
time = 0;
i=1;
K=[
   23.9371    6.4042    5.2636    0.1559;
    6.0097    1.8868    4.7955    0.2022;
]
while(time < 10)
    time = toc;
    t(i) = time;
    % read the joint states
    jointData = receive(JointStates);
    X(i,1) = jointData.Position(1,1);
    X(i,2) = jointData.Velocity(1,1);
    X(i,3) = jointData.Position(2,1);
    X(i,4) = jointData.Velocity(1,1);
    x = X(i,:)';
    Z = -K*x;
    tau1.Data = Z(1,1);
    tau2.Data = Z(2,1);
    U(i,1)=Z(1,1);
    U(i,2)=Z(2,1);
    send(j1_effort,tau1);
    send(j2_effort,tau2);
    % sample the time, joint state values, and calculated torques here to
    % be plotted at the end
    i=i+1;
end
tau1.Data = 0;
tau2.Data = 0;
send(j1_effort,tau1);
send(j2_effort,tau2);
% disconnect from roscore
rosshutdown;
% plot the trajectories
plot(t,X(:,1))
xlabel('t')
ylabel('${\theta_1}$', 'Interpreter','latex')
title('Gazebo Simulation')
saveas(gcf,'theta_1.jpg')
plot(t,X(:,2))
xlabel('t')
ylabel('$\dot{\theta_1}$', 'Interpreter','latex')
title('Gazebo Simulation')
saveas(gcf,'theta_dot_1.jpg')
plot(t,U(:,1))
xlabel('t')
ylabel('${\tau_1}$', 'Interpreter','latex')
title('Gazebo Simulation')
saveas(gcf,'Tau_1.jpg')
plot(t,X(:,3))
xlabel('t')
ylabel('${\theta_2}$', 'Interpreter','latex')
title('Gazebo Simulation')
saveas(gcf,'theta_2.jpg')
plot(t,X(:,4))
xlabel('t')
ylabel('$\dot{\theta_2}$', 'Interpreter','latex')
title('Gazebo Simulation')
saveas(gcf,'theta_dot_2.jpg')
plot(t,U(:,2))
xlabel('t')
ylabel('${\tau_2}$', 'Interpreter','latex')
title('Gazebo Simulation')
saveas(gcf,'tau_2.jpg')