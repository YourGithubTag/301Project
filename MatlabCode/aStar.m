function [retmap,retvisited,retsteps] = aStar( mapfile,startlocation,targetlocation)
% convert map file into a 2D matlab array
retmap = map_convert(mapfile);

% initally no steps
retsteps = [];

% get the mnumber of rows and columns
[m, n] = size(retmap);
% initially, all nodes are unevaluated so they are 1
retvisited = ones(m,n);

% create a cell array for nodes that are on the track
track = cell(1,1);
count = 1;

for i = 1:m
    for j = 1:n
        % If the node is on the track (0 for track, 1 for walls, )
        if(retmap(i,j) == 0)
            % Add node to array
            track{count,1} = [i,j];
            count = count + 1;
        end
    end
end

% A set of nodes that are already evaluated
closedSet =  cell(1,1);
countCloseSet = 1;

% A set of discovered nodes that are not yet evaluated
openSet = cell(1,1);
% Add the start point to the set (since it is already known)
openSet{1} = startlocation;
countOpenSet = 2;

% A cell array storing the neighbour node that the current node came from,
% for example, if node B came from node A then the value at cameFrom(node B) is node A
cameFrom = cell(1,1);

% gScore : For each node, the distance from the start node to that node
gScore = Inf(m,n);
% The distance from start node to start node is 0
gScore(startlocation(1), startlocation(2)) = 0;

% F(n) = G(n) + H(n)
% fScore : For each node, the total distance from the start node to the end
% node by passing through that node.
fScore = Inf(m,n);
fScore(startlocation(1), startlocation(2)) = heuristic_cost_estimate(startlocation, targetlocation);

% While there are unnevaluated nodes then loop
while(~cellfun('isempty', openSet))
    % The current node is the one with the lowest fScore
    current = lowest_fScore(targetlocation, openSet,fScore);
    % If target is reached then the final path is found (exit loop)
    if(current == targetlocation)
        retvisited(targetlocation(1), targetlocation(2)) = 0;
        closedSet{countCloseSet + 1} = current;
        break;
    end
    % Remove the current node from the set of unevaluated nodes
    for k = 1:numel(openSet);
        if(openSet{k} == current)
            openSet(:,k) = [];
            countOpenSet = countOpenSet - 1;
            break;
        end
    end
    % Add the current node to the set of evaluated nodes
    closedSet{countCloseSet} = current;
    % The current node has been evaluated so its value is now 0
    retvisited(current(1), current(2)) = 0;
    countCloseSet = countCloseSet + 1;
    % Get the neighbour nodes of the current node
    neighbours = getNeighbour(track,current);
    % Get the number of neighbours
    [row, ~] = size(neighbours);
    % loop through each neighbour to find the best node to move to
    for i = 1:row
        % If the neighbour is already evaluated then skip this iteration
        if(isFound(neighbours(i,:),closedSet) == 1)
            continue;
        end
        % tentitative_gScore : The distance from the start node to a neighbour node
        % In this case, the distance between each node is always 1
        
        tentitative_gScore = gScore(current(1), current(2)) + 1;
        
        % If the neighbour node is not in the set of discovered nodes then
        if(isFound(neighbours(i,:),openSet) == 0)
            % Add the node to the set since it has been discovered
            openSet{countOpenSet} = neighbours(i,:);
            countOpenSet = countOpenSet + 1;
        elseif tentitative_gScore >= gScore(neighbours(i,1),neighbours(i,2))
            continue;
        end
        
        % Store the current node as the node that this neighbour came from
        cameFrom{neighbours(i,1),neighbours(i,2)} = current;
        % Store the distance from the start node to this neighbour node
        gScore(neighbours(i,1),neighbours(i,2)) = tentitative_gScore;
        % Approximates the total distance from the start node to the end
        % node by passing through this neighbour
        fScore(neighbours(i,1),neighbours(i,2)) = gScore(neighbours(i,1),neighbours(i,2)) + heuristic_cost_estimate(neighbours(i,:), targetlocation);
    end
end
% If the target was reach after finish searching
if(current == targetlocation)
    % Gets the steps of the best path found
    retsteps = getSteps(cameFrom, current, startlocation);
end
% gets the nodes which have been searched
% search = getSearchedPath(closedSet);
end

% This function return true if a node is found in a set, false otherwise
function [logic] = isFound(node, Set)
logic = 0;
for i = 1:numel(Set)
    if(node == Set{i})
        logic = 1;
        break;
    else
        logic = 0;
    end
end
end

% This function returns an array of neighbour nodes of the current node
function [neighbours] = getNeighbour(track,current)
[m,~] = size(track);
neighbours = [];
% Loop through the track array (so that all walls are ignored)
for i = 1:m
    % Add the node to the array of neighbour if it is a neighbour of the
    % current node (checking 4 directions: top, bottom, left, right)
    if(track{i,1}(1) + 1 == current(1) && track{i,1}(2) == current(2)...
            || track{i,1}(1) - 1 == current(1) && track{i,1}(2) == current(2)...
            || track{i,1}(1) == current(1) && track{i,1}(2) + 1 == current(2)...
            || track{i,1}(1) == current(1) && track{i,1}(2) - 1 == current(2))
        neighbours = [neighbours;track{i,1}];
    end
end
end

% This function returns the node closest to the end node with the lowest fScore
function [ind] = lowest_fScore(targetlocation, openSet, fScore)
[~, m] = size(openSet);
% Initally the first node in the set is the lowest
lowest = fScore(openSet{1}(1),openSet{1}(2));
ind = openSet{1};
% Loop through all the other nodes in the open set
for i = 2:m
    % If lower a fScore node is found then that node becomes the new lowest
    if(fScore(openSet{i}(1),openSet{i}(2)) < lowest)
        lowest = fScore(openSet{i}(1),openSet{i}(2));
        ind = openSet{i};
        % Otherwise, if the two fScores are equal then...
    elseif(fScore(openSet{i}(1),openSet{i}(2)) == lowest)
        % The node that is closer to the end node becomes the new lowest
        if(heuristic_cost_estimate(openSet{i}, targetlocation) <= heuristic_cost_estimate(ind, targetlocation))
            lowest = fScore(openSet{i}(1),openSet{i}(2));
            ind = openSet{i};
        end
        % nothing happens otherwise
    end
end
end

% This function returns the steps taken from the start node to get to the end node
function [s] = getSteps(cameFrom, current, startlocation)
% Trace back from the current node (normally, the end node)
s = current;
% while the start node is not reached, Loop
while(~isequal(current, startlocation))
    % move to the node that this current node came from
    current = cell2mat(cameFrom(current(1),current(2)));
    % Add the current node to the step array
    s = [s;current];
end
% Flip the array of steps taken (because we trace backwards from the end)
s = flipud(s);
end

% This function return the manhattan distance between 2 nodes
function [cost] = heuristic_cost_estimate(start, goal)
cost = (abs(start(1) - goal(1)) + abs(start(2) - goal(2)));
end
