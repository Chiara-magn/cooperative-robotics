classdef ActionManager < handle
    %ACTIONMANAGER Store actions and solve the active task-priority stack.
    %
    % An action is represented by a cell array of Task objects. The order in the cell array defines strict priority: element 1 has the highest priority, and each following task uses only the motion left available by the tasks above it.
    %
    % The solver uses the UVMS system velocity convention
    %
    %   ydot = [q_dot; v_nu]
    %
    % where q_dot contains the arm joint velocities and v_nu contains the vehicle velocities expressed in the vehicle frame. Their dimensions are obtained from the robot model when the manager is constructed.

    properties (Access = private)
        actions          % Dictionary: action name -> prioritized task stack.
        currentActionName (1,1) string = "" % Name of the active action.
        dofArm           % Number of manipulator degrees of freedom.
        dofVehicle       % Number of vehicle velocity degrees of freedom.
        dof              % Total dimension of ydot = [q_dot; v_nu].
        satMin           % [dof x 1] original lower system-velocity limits.
        satMax           % [dof x 1] original upper system-velocity limits.
        unifyingTasks = {}    % unified task list to store tasks of all actions.
        previousActionName (1,1) string = ""   % Azione precedente.
        tActionStart = 0                % Istante in cui è stata selezionata l'azione corrente.
        transitionDuration = 2          % Durata della transizione [s].
    end

    properties (Constant, Access = private)
        % The residual damping objective is generated internally rather than represented by a Task, so its regularization parameters live here.
        residualLambda = 1e-4
        residualThreshold = 0.01
        residualGain = 10
    end

    methods
        function obj = ActionManager(robot)
            %ACTIONMANAGER Construct an empty action library for a UVMS model.
            %
            % Input:
            %   robot - UvmsModel whose arm and vehicle dimensions and velocity limits define the system velocity vector handled by this manager.
            %
            % Output:
            %   obj - Initialized ActionManager with no active action.
            obj.actions = configureDictionary("string", "cell");
            obj.dofArm = numel(robot.q);
            obj.dofVehicle = numel(robot.v_nu);
            obj.dof = obj.dofArm + obj.dofVehicle;

            armMax = robot.q_dot_max(:);
            vehicleMax = robot.v_nu_max(:);
            if numel(armMax) ~= obj.dofArm || numel(vehicleMax) ~= obj.dofVehicle || any(~isfinite([armMax; vehicleMax])) || any([armMax; vehicleMax] < 0)
                error('ActionManager:InvalidVelocityLimits', 'q_dot_max and v_nu_max must contain one finite, non-negative limit for each arm and vehicle velocity.');
            end
            obj.satMax = [armMax; vehicleMax];
            obj.satMin = -obj.satMax;
        end

        function addAction(obj, actionName, taskStack)
            %ADDACTION Register a prioritized task stack under a unique name.
            %
            % Inputs:
            %   actionName - Non-empty string scalar used to identify the action in subsequent calls.
            %   taskStack - 1-by-N cell array of Task objects, ordered from highest to lowest priority.
            %
            % Output:
            %   None. This method updates obj.actions in place.
            actionName = string(actionName);

            if ~isscalar(actionName) || ismissing(actionName) || strlength(strtrim(actionName)) == 0
                error('ActionManager:InvalidActionName', 'Action name must be a non-empty string scalar.');
            end
            actionName = strtrim(actionName);

            if ~iscell(taskStack) || isempty(taskStack) || ~all(cellfun(@(task) isa(task, 'Task'), taskStack))
                error('ActionManager:InvalidTaskStack', 'Task stack must be a non-empty cell array of Task objects.');
            end

            if isKey(obj.actions, actionName)
                error('ActionManager:DuplicateAction', 'An action named "%s" is already registered.', actionName);
            end

            % Dictionary values have cell type; the outer cell stores the complete task stack as the single value associated with the key.
            obj.actions(actionName) = {taskStack};
        end

        function [v_nu, q_dot] = computeICAT(obj, robot, t)
            %COMPUTEICAT Compute arm and vehicle velocity references.
            %
            % Inputs:
            %   robot - UvmsModel containing the current state, transforms, goals and sensor data used by the active tasks.
            %   t - Current simulation time [s], used to compute the transition between the previous and the current action.
            %
            % Outputs:
            %   v_nu - [dofVehicle x 1] vehicle velocity expressed in the vehicle frame.
            %   q_dot - [dofArm x 1] manipulator joint velocity [rad/s].
            %
            % The unifying task list is solved in priority order. Each task first updates its reference rate xdotbar, Jacobian J and activation matrix A; A is then scaled by a transition weight: 1 for tasks in both the previous and current action, 0->1 for tasks entering, 1->0 for tasks leaving. The iCAT recursion processes the tasks while preserving higher-priority motion.
            if obj.currentActionName == ""
                error('ActionManager:NoActiveAction', 'Select an action before calling computeICAT.');
            end
        
            % Work on the unifying list, so tasks leaving the action are still computed while they fade out.
            tasks = obj.unifyingTasks;
        
            % Membership of each unifying task in the current and previous action.
            inCur = obj.isInAction(tasks, obj.currentActionName);
            if obj.previousActionName == ""
                inPrev = inCur;     % First action: no transition.
            else
                inPrev = obj.isInAction(tasks, obj.previousActionName);
            end
        
            % Transition progress: 0 at the action switch, 1 after transitionDuration, with a smooth profile.
            s = min(max((t - obj.tActionStart) / obj.transitionDuration, 0), 1);
            s = 0.5 * (1 - cos(pi * s));
        
            % Refresh all configuration-dependent task quantities and apply the transition weight to the activation.
            for i = 1:length(tasks)
                tasks{i}.updateReference(robot);
                tasks{i}.updateJacobian(robot);
                tasks{i}.updateActivation(robot);
        
                if inCur(i) && inPrev(i)
                    w = 1;          % In both actions: always active.
                elseif inCur(i)
                    w = s;          % Entering: 0 -> 1.
                elseif inPrev(i)
                    w = 1 - s;      % Leaving: 1 -> 0.
                else
                    w = 0;          % Not used by either action.
                end
                tasks{i}.A = tasks{i}.A * w;
            end
        
            % Start with zero commanded velocity, the full system motion space and the original velocity bounds. Each priority consumes part of the available velocity range.
            ydotbar = zeros(obj.dof,1);
            Qp = eye(obj.dof);
            satMinRemaining = obj.satMin;
            satMaxRemaining = obj.satMax;
            K = eye(obj.dof); % Neutral control-space weighting required by the current iCAT interface.
            for i = 1:length(tasks)
                [Qp, ydotbar, ~, ~, ~, satMinRemaining, satMaxRemaining] = iCAT_task(tasks{i}.A, tasks{i}.J, Qp, ydotbar, tasks{i}.xdotbar, tasks{i}.lambda, tasks{i}.threshold, tasks{i}.gain, satMinRemaining, satMaxRemaining, obj.satMin, obj.satMax, K);
            end
        
            % Use any residual degrees of freedom to drive the system velocity toward zero. As the last-priority task, this cannot alter motion already assigned to higher-priority objectives and must respect the velocity range left by the preceding tasks.
            [~, ydotbar] = iCAT_task(eye(obj.dof), eye(obj.dof), Qp, ydotbar, zeros(obj.dof,1), obj.residualLambda, obj.residualThreshold, obj.residualGain, satMinRemaining, satMaxRemaining, obj.satMin, obj.satMax, K);
        
            % Split ydotbar using the same ordering used to build every task Jacobian: arm joints first, then the vehicle twist.
            q_dot = ydotbar(1:obj.dofArm);
            v_nu = ydotbar(obj.dofArm+1:obj.dof);
        end

        function setCurrentAction(obj, actionName, t)
            %SETCURRENTACTION Select an action by name.
            %
            % Input:
            %   actionName - String scalar previously passed to addAction.
            %
            % Output:
            %   None. This method updates obj.currentActionName in place.
            
            actionName = string(actionName);
            if nargin < 3, t = 0; end
            if ~isscalar(actionName) || ismissing(actionName) || strlength(strtrim(actionName)) == 0
                error('ActionManager:InvalidActionName', 'Action name must be a non-empty string scalar.');
            end
            actionName = strtrim(actionName);

            if ~isKey(obj.actions, actionName)
                error('ActionManager:UnknownAction', 'No action named "%s" has been registered.', actionName);
            end

            if isempty(obj.unifyingTasks)
                error('ActionManager:NoUnifyingList', 'Set the unifying task list before selecting an action.');
            end
            obj.validateUnifyingList();
        
            if actionName ~= obj.currentActionName
                obj.previousActionName = obj.currentActionName;
                obj.currentActionName = actionName;
                obj.tActionStart = t;
            end
        end

        % creates an unified list of all the tasks
        function setUnifyingTaskList(obj, taskList)
            if ~iscell(taskList) || isempty(taskList) || ~all(cellfun(@(t) isa(t,'Task'), taskList))
                error('ActionManager:InvalidTaskList', 'Unifying list must be a non-empty cell array of Task objects.');
            end
            obj.unifyingTasks = taskList;
        end

        %VALIDATEUNIFYINGLIST Error if any task of a registered action is missing from the unifying list.
        function validateUnifyingList(obj)
            names = keys(obj.actions);
            for a = 1:numel(names)
                stack = obj.actions{names(a)};
                for k = 1:numel(stack)
                    if ~any(cellfun(@(u) u == stack{k}, obj.unifyingTasks))
                        error('ActionManager:MissingTask', ...
                            'Task %d of action "%s" is not in the unifying list.', k, names(a));
                    end
                end
            end
        end
    
        
        function tf = isInAction(obj, tasks, name)
            %ISINACTION Flag which tasks of a list belong to the named action.
            tf = false(1, numel(tasks));
            if name == "", return, end
            stack = obj.actions{name};
            for i = 1:numel(tasks)
                tf(i) = any(cellfun(@(u) u == tasks{i}, stack));
            end
        end

    end
end
