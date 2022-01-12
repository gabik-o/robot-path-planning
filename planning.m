%%Map and set poses
load officemap.mat;

% Set the start and goal poses
%start = ExampleHelperAMCLGazeboTruePose; %import start pose automatically
%when ROS connected

start = [0, 0, 0];
goal = [5, 4, pi];

%% Planner setup

bounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

%Choose one of StateSpace

ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 0.15;

%Set properties w.r.t StateSpace below
%Set stateValidator

stateValidator = validatorOccupancyMap(ss);
stateValidator.Map = map_bigger_obstacles;
stateValidator.ValidationDistance = 0.1;

%Set properties w.r.t stateValidator below
%Choose planner

planner = plannerRRTStar(ss, stateValidator);

%planner = plannerRRT(ss, stateValidator);
%Set planer properites in blank

planner.MaxConnectionDistance = 0.15 ;
planner.MaxIterations = 10000 ;
planner.GoalReachedFcn = @exampleHelperCheckIfGoal;
rng(0,'twister')
[pthObj, solnInfo] = plan(planner, start, goal);

%interpolate(pthObj,600)% set interpolate number
%% Plot

show(map)
hold on

% Show the start and goal positions of the robot

plot(start(1), start(2), 'ro')
plot(goal(1), goal(2), 'mo')

% Show the start and goal headings

r = 0.5;
plot([start(1), start(1) + r*cos(start(3))], [start(2), start(2) +
r*sin(start(3))], 'g-' )
plot([goal(1), goal(1) + r*cos(goal(3))], [goal(2), goal(2) +
r*sin(goal(3))], 'm-' )

% Search tree

plot(solnInfo.TreeData(:,1), solnInfo.TreeData(:,2), '.b-');

% Interpolate and plot path

plot(pthObj.States(:,1), pthObj.States(:,2), 'r-', 'LineWidth', 2)
numberofelemnts = numel(pthObj.States)
