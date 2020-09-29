startloc = [2,2];
targetloc = [14,18];

% Your simulations will use this structure
[x,y,z] = dfs('map_8.txt',[startloc],[targetloc]);

% timedfs = @() dfs('map_8.txt',[startloc],[targetloc]);
% disp('dfs')
% timeforBFS=timeit(timedfs)

% plotmap(x,z);