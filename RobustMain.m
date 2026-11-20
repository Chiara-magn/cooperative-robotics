%% ROBUST UVMS TASK-PRIORITY EXERCISE
% This script assembles the main blocks of the control architecture:
%
%   task definitions -> ActionManager/iCAT -> kinematic simulator -> Unity
%
% At every iteration, the Kinematic Control Layer computes the system velocity ydot = [q_dot; v_nu].
% UvmsSim integrates this reference directly; in a real robot, a Dynamic Control Layer would track the same reference.
%
% Run this script with the current MATLAB folder set to the template root so that the relative paths below resolve consistently.

%% Make exercise dependencies visible to MATLAB
addpath('./tools');
addpath('./icat');
addpath('./robust_robot');
addpath('./tpik');
clc; clear; close all;

%% Configure and initialize the simulation
% dt is both the integration step and the nominal controller sample time.
dt       = 0.005;
endTime  = 50;

% UvmsModel owns state and kinematics; UvmsSim updates that shared handle.
robotModel = UvmsModel();
sim = UvmsSim(dt, robotModel, endTime);

% Unity is used only as viewer/sensor endpoint; it is not the simulator.
unity = UnityInterface("127.0.0.1");

%% Define one action as a strict-priority stack of tasks
% The first cell is the highest-priority task. Add safety tasks before the action objective when extending this exercise (for example, joint limits).
task_position = Task_position();
task_orientation = Task_horizontal();
task_altitude = Task_min_altitude();
task_zero_altitude_control = Task_landing_altitude();

% ex 3 stacks
% move_tool_tasks = {task_tool};
% safety_altitude = {task_altitude, task_position};

% ex 4 stacks
safe_navigation = {task_altitude, task_orientation, task_position};
landing = {task_zero_altitude_control, task_orientation, task_position};

% Actions are registered and selected by name; the task order still defines strict priority within each action.
actionManager = ActionManager(robotModel);
% actionManager.addAction("safety_altitude_control", safety_altitude); % ex 3
% actionManager.addAction("move_tool", move_tool_tasks);

actionManager.addAction("safe_navigation", safe_navigation);
actionManager.addAction("landing", landing);

% Unifying list, written by hand. It must contain every task of every action.
% Priority order: safety first, then attitude, then position.
unifying = {task_altitude, task_zero_altitude_control, task_orientation, task_position};
actionManager.setUnifyingTaskList(unifying);

actionManager.setCurrentAction("safe_navigation", 0);
% actionManager.setCurrentAction("safety_altitude_control"); % ex 3

% The logger monitors every task in the unifying list.
task_set = unifying;
% task_set = safety_altitude; % ex 3

%% Set desired poses
% Positions are in metres and orientations are [roll pitch yaw] in radians. Both poses are expressed in the inertial/world frame.

% goal ex_2
% w_arm_goal_position = [12.2025, 37.3748, -39.8860]';
% w_arm_goal_orientation = [0, pi, pi/2];
% w_vehicle_goal_position = [10.5, 37.5, -38]';
% w_vehicle_goal_orientation = [0, 0, 0];

% goal ex_3_1
% w_arm_goal_position = [10.5, 37.5, -38]';
% w_arm_goal_orientation = [0, pi, pi/2];
% w_vehicle_goal_position = [12.2025, 37.3748, -39.8860]';
% w_vehicle_goal_orientation = [0, 0, 0];

% goal ex_3_2
% w_arm_goal_position = [10.5 37.5 -38]';
% w_arm_goal_orientation = [0, pi, pi/2];
% w_vehicle_goal_position = [10.5, 37.5, -38]';
% w_vehicle_goal_orientation = [0, 0, 0];

% goal ex_4 (the arm goal is not used in this exercise, but setGoal requires it)
w_arm_goal_position = [10.5, 37.5, -38]';
w_arm_goal_orientation = [0, pi, pi/2];
w_vehicle_goal_position = [50 -12.5 -33]';
w_vehicle_goal_orientation = [0, 0, 0];

% ex 1 initial position
% robotModel.eta = [10.5, 35.5, -36, 0, 0, pi/2]';

% ex 2 initial position
% robotModel.eta = [10.5, 35.5, -36, -pi/3, pi/3, pi/2]';

% ex 3 initial position
% robotModel.eta = [10.5 35.5 -36 0 0 pi/2]';

% ex 4 initial position
robotModel.eta = [48.5 11.5 -33 0 0 0]';

% Store the goals as homogeneous transforms in the shared model.
robotModel.setGoal(w_arm_goal_position, w_arm_goal_orientation, w_vehicle_goal_position, w_vehicle_goal_orientation);

% Preallocate one column for the initial state plus all integration steps.
logger = SimulationLogger(ceil(endTime/dt)+1, robotModel, task_set);

%% Run the closed-loop kinematic simulation
landingStarted = false;

tic
for step = 1:sim.maxSteps
    % Read the newest sensor sample. If none is available, the interface keeps the previous altitude value.
    robotModel.altitude = unity.receiveAltitude(robotModel);

    % Switch to landing once the vehicle is horizontally close to the goal position.
    distGoal = norm(robotModel.eta(1:2) - w_vehicle_goal_position(1:2));
    if ~landingStarted && distGoal < 0.3
        actionManager.setCurrentAction("landing", sim.time);
        landingStarted = true;
        fprintf('Landing started at t = %.2f s\n', sim.time);
    end

    % Update every task and solve the current strict-priority hierarchy.
    [v_nu, q_dot] = actionManager.computeICAT(robotModel, sim.time);

    % Integrate arm and vehicle velocities, then refresh all transforms.
    sim.step(v_nu, q_dot);

    % Send the state after integration so the viewer displays the new sample.
    unity.send(robotModel);

    % Store state, task references and activations at the current sample.
    logger.update(sim.time, sim.loopCounter);

    % misalignment calculation for test and verification
    R = robotModel.wTv(1:3,1:3);
    zv = R(:,3);
    misalignment = norm([0;0;1] - zv);

    % Print a compact status update approximately once per simulated second.
    if mod(sim.loopCounter, round(1 / sim.dt)) == 0
        fprintf('t = %.2f s\n', sim.time);
        fprintf('alt = %.2f m\n', robotModel.altitude);
        % fprintf('misalignment = %.4f\n', misalignment);
    end

    % Busy-wait until one sample period has elapsed when real-time playback is desired.
    % Remove this call to execute the simulation as fast as possible.
    SlowdownToRealtime(dt);
end

%% Review results and release communication resources
logger.plotAll();

% Explicit deletion closes UDP sockets before the script terminates.
delete(unity);