rosshutdown
ipaddress = "http://192.168.2.91:11311"; %Change to 'your_vm_ip'
rosinit(ipaddress);
odomSub = rossubscriber('/odom');
[velPub,velMsg] = rospublisher('/cmd_vel','geometry_msgs/Twist');

%% Set time and path

sampleTime = 0.05; % Sample time [s]
t = 0:sampleTime:100; % Time array

%% Path from RRT or RRT* path planner

desired_pose = pthObj.States; %Chosse variable of your Path from the planner
path = desired_pose(:,1:2); %define Waypoints
desired_pose = desired_pose';
  
%% Set Conroller Pure Pursit

controller = controllerPurePursuit('DesiredLinearVelocity',0.15,'MaxAngularVelocity',0.7);
controller.Waypoints = path;
goal = [path(end,1) path(end,2)]';
  
%% Pose from ROS

odompose = odomSub.LatestMessage;
odomQuat = [odompose.Pose.Pose.Orientation.W,
odompose.Pose.Pose.Orientation.X, ...
odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
odomRotation = quat2eul(odomQuat);
pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y,
odomRotation(1)];
poses = zeros(3,numel(t)); % Pose matrix
initPose = pose;

% Set iteration rate

r = rateControl(1/sampleTime);

%% Motion Control

for idx = 1:numel(t)
position = poses(:,idx)';
odompose = odomSub.LatestMessage;
odomQuat = [odompose.Pose.Pose.Orientation.W,
odompose.Pose.Pose.Orientation.X, ...
odompose.Pose.Pose.Orientation.Y,
odompose.Pose.Pose.Orientation.Z];
odomRotation = quat2eul(odomQuat);
pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y,
odomRotation(1)];
currPose = pose(1:2);

% End if pathfollowing is vehicle has reached goal position within tolerance of 0.05m

dist = norm(goal'-currPose);
if (dist < .05)
disp("Goal position reached")
break;
end

% Run the Pure Pursuit controller and convert output to wheel speeds

[V,w] = controller(poses(:,idx));
velMsg.Linear.X = V;
velMsg.Angular.Z = w;
send(velPub,velMsg)
            
% Perform forward discrete integration step

odompose = odomSub.LatestMessage;
odomQuat = [odompose.Pose.Pose.Orientation.W,
odompose.Pose.Pose.Orientation.X, ...
odompose.Pose.Pose.Orientation.Y,
odompose.Pose.Pose.Orientation.Z];
odomRotation = quat2eul(odomQuat);
pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y,
odomRotation(1)];
poses(:,idx+1) = pose';
            
%plot robot onto known map

plot(pose(1), pose(2), '-ko','MarkerSize', 2)
hold on
% waiting to iterate at the proper rate

waitfor(r);
end
%% Stop TurtleBot

velMsg.Linear.X = 0; % Linear motion Velocity
velMsg.Angular.Z = 0; % Angular motion Velocity
send(velPub,velMsg);
