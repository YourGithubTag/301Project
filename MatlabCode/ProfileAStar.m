startloc = [2,2];
targetloc = [14,18];

% Your simulations will use this structure
[x,y,z] = aStar('map_8.txt',[startloc],[targetloc]);

% timeaStar = @() aStar('map_8.txt',[startloc],[targetloc]);
% disp('aStar')
% timeforASTAR=timeit(timeaStar)

% plotmap(x,z);