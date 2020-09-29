% Function descriptions
% To view a map, given a map file
startloc = [2,2];
targetloc = [14,18];

viewmap('map_1.txt',0); % shows walls
viewmap('map_1.txt',1); % shows track

%or else

[m]=map_convert('map_1.txt');
plotmap(m);
%Here   'm' is the map file returned as a matrix


% Your simulations will use this structure
[x,y,z] = dfs('map_8.txt',[startloc],[targetloc]);
[n,o,p] = BFS('map_8.txt',[startloc],[targetloc]);
[m,v,s]= aStar('map_8.txt',[startloc],[targetloc]);

timeDFS = @() dfs('map_8.txt',[startloc],[targetloc]);
timeBFS = @() BFS('map_8.txt',[startloc],[targetloc]);
timeaStar = @() aStar('map_8.txt',[startloc],[targetloc]);

disp('DFS' )
timeforDFS= timeit(timeDFS)
disp('BFS' )
timeforBFS=timeit(timeBFS)
disp('aStar')
timeforASTAR=timeit(timeaStar)



%Here   'm' is the map file returned as a matrix
%       'v' is a matrix that shows which cells have been visited, '0' means
%       visited, '1' means not visited
%       's' is the vector of steps to reach the target,
%       [startloc] is a vector i.e. [2,2]
%       [targetloc] is also a vector ie [4,18]

%To view the path determined above use 

plotmap(x,z);
plotmap(n,p);
plotmap(m,s);
