% This is a shell that you will have to follow strictly. 
% You will use the plotmap() and viewmap() to display the outcome of your algorithm.

% Load sample_data_map_8, three variables will be created in your workspace. These were created as a 
% result of [m,v,s]=dfs('map_8.txt',[14,1],[1,18]);
% The solution can be viewed using 
% plotmap(m,s) 

% write your own function for the DFS algorithm.
function [retmap,retvisited,retsteps] = BFS( mapfile,startlocation,targetlocation)
[M] = map_convert(mapfile);
step = 1;

[x,y] = size(M);
h_1 = startlocation(1);
v_1 = startlocation(2);
h_2 = targetlocation(1);
v_2 = targetlocation(2);
retvisited = ones(x,y);
retmap = map_convert(mapfile);
current_x = h_1;
current_y = v_1;
retsteps(step,:) = [current_x,current_y];
retvisited(current_x,current_y) = 0;

while(current_x~=h_2 | current_y~=v_2)
    if((M(current_x+1,current_y) == 0&(retvisited(current_x+1,current_y)~=0))&(M(current_x-1,current_y) == 0&(retvisited(current_x-1,current_y)~=0)))
        if(current_x<h_2)
            current_x = current_x + 1;  
            step = step+1;
            retsteps(step,:) = [current_x,current_y];
            retvisited(current_x,current_y) = 0;
            len = length(retsteps);
        else
            current_x = current_x - 1;  
            step = step+1;
            retsteps(step,:) = [current_x,current_y];
            retvisited(current_x,current_y) = 0;
            len = length(retsteps);
        end
    elseif((M(current_x,current_y+1) == 0&(retvisited(current_x,current_y+1)~=0))&(M(current_x,current_y-1) == 0&(retvisited(current_x,current_y-1)~=0)))
        if(current_y<v_2)
            current_y = current_y + 1;  
            step = step+1;
            retsteps(step,:) = [current_x,current_y];
            retvisited(current_x,current_y) = 0;
            len = length(retsteps);
        else
            current_y = current_y - 1;  
            step = step+1;
            retsteps(step,:) = [current_x,current_y];
            retvisited(current_x,current_y) = 0;
            len = length(retsteps);
        end       
    elseif(M(current_x+1,current_y) == 0&&(retvisited(current_x+1,current_y)~=0))
        current_x = current_x + 1;  
        step = step+1;
        retsteps(step,:) = [current_x,current_y];
        retvisited(current_x,current_y) = 0;
        len = length(retsteps);
    elseif(M(current_x,current_y + 1) == 0&&(retvisited(current_x,current_y + 1)~=0))
        current_y = current_y + 1;  
        step = step+1;
        retsteps(step,:) = [current_x,current_y];
        retvisited(current_x,current_y) = 0;
        len = length(retsteps);
    elseif(M(current_x-1,current_y) == 0&&(retvisited(current_x-1,current_y)~=0))
        current_x = current_x - 1;  
        step = step+1;
        retsteps(step,:) = [current_x,current_y];
        retvisited(current_x,current_y) = 0;
        len = length(retsteps);
    elseif(M(current_x,current_y - 1) == 0&&(retvisited(current_x,current_y-1)~=0))
        current_y = current_y - 1;  
        step = step+1;
        retsteps(step,:) = [current_x,current_y];
        retvisited(current_x,current_y) = 0;
        len = length(retsteps);
    else
        current_x = retsteps(len,1);
        current_y = retsteps(len,2);
        step = step + 1;
        retsteps(step,:) = [current_x,current_y];
        len = len - 1;
    end 
    
end

end
