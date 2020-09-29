startloc = [2,2];
targetloc = [14,18];

% Your simulations will use this structure
[x,y,z] = BFS('map_8.txt',[startloc],[targetloc]);

% timeBFS = @() BFS('map_8.txt',[startloc],[targetloc]);
% disp('BFS')
% timeforBFS=timeit(timeBFS)


% plotmap(x,z);