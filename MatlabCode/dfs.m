% This is a shell that you will have to follow strictly. 
% You will use the plotmap() and viewmap() to display the outcome of your algorithm.

% Load sample_data_map_8, three variables will be created in your workspace. These were created as a 
% result of [m,v,s]=dfs('map_8.txt',[14,1],[1,18]);
% The solution can be viewed using 
% plotmap(m,s) 

% write your own function for the DFS algorithm.
function [retmap,retvisited,retsteps] = dfs(mapfile,startlocation,targetlocation)
    retmap = map_convert(mapfile);
    retvisited = ones(15,19);
    retsteps = [];
    retsteps = [retsteps; startlocation];
    retvisited(startlocation(1),startlocation(2)) = 0;
    while(startlocation(1) ~= targetlocation(1) || startlocation(2) ~= targetlocation(2))
        [down, right, up, left, retvisited] = nextstep(retmap, retvisited, startlocation);
        [dp, rp, upp,lp] = priority(startlocation, retvisited, down, right, up, left);
        if(dp == 1)
            
            if(retvisited(startlocation(1),startlocation(2)) == 0 && retvisited(startlocation(1)+1, startlocation(2)) == 0)
                retvisited(startlocation(1),startlocation(2)) = -1;
            end
            startlocation(1) = startlocation(1)+1;
            if(retvisited(startlocation(1),startlocation(2)) ~= -1)
                retvisited(startlocation(1),startlocation(2)) = 0;
            end
            
            retsteps = [retsteps; startlocation];
        elseif(rp == 1)
            if(retvisited(startlocation(1),startlocation(2)) == 0 && retvisited(startlocation(1), startlocation(2)+1) == 0)
                retvisited(startlocation(1),startlocation(2)) = -1;
            end
            startlocation(2) = startlocation(2)+1;
            if(retvisited(startlocation(1),startlocation(2)) ~= -1)
                retvisited(startlocation(1),startlocation(2)) = 0;
            end
            
            
            retsteps = [retsteps; startlocation];
        elseif(upp == 1)
            if(retvisited(startlocation(1),startlocation(2)) == 0 && retvisited(startlocation(1)-1, startlocation(2)) == 0)
                retvisited(startlocation(1),startlocation(2)) = -1;
            end
            startlocation(1) = startlocation(1)-1;
            if(retvisited(startlocation(1),startlocation(2)) ~= -1)
                retvisited(startlocation(1),startlocation(2)) = 0;
            end
            
            
            retsteps = [retsteps; startlocation];
        elseif(lp == 1)
            if(retvisited(startlocation(1),startlocation(2)) == 0 && retvisited(startlocation(1), startlocation(2)-1) == 0)
                retvisited(startlocation(1),startlocation(2)) = -1;
            end
            startlocation(2) = startlocation(2)-1;
            if(retvisited(startlocation(1),startlocation(2)) ~= -1)
                retvisited(startlocation(1),startlocation(2)) = 0;
            end
            
            
            retsteps = [retsteps; startlocation];
        end

    end
    
end


function [valid,retvisited] = check(retmap, row, col, currentlocation, retvisited)
%???ban14?16
%     if(retvisited(currentlocation(1),currentlocation(2)) == 0 && retvisited(row, col) == 0)
%         retvisited(row, col) = -1;
%     end
    if(retmap(row, col) == 1)%is wall
        valid = 0;
%     elseif(retvisited(currentlocation(1),currentlocation(2)) == 0 && retvisited(row, col) == 0)
%         valid = 1;
%         retvisited(currentlocation(1),currentlocation(2)) = -1;
    elseif(retvisited(row, col) == -1)
        valid = 0;
    else
        valid = 1;
    end
end

function [down, right, up, left, retvisited] = nextstep(retmap, retvisited, currentlocation)
    [down, retvisited] = check(retmap, currentlocation(1)+1, currentlocation(2), currentlocation, retvisited);
    [up, retvisited] = check(retmap, currentlocation(1)-1, currentlocation(2),currentlocation, retvisited);
    [right, retvisited] = check(retmap, currentlocation(1), currentlocation(2)+1,currentlocation, retvisited);
    [left,retvisited] = check(retmap, currentlocation(1), currentlocation(2)-1,currentlocation, retvisited);
end

function [dp, rp, upp,lp] = priority(startlocation, retvisited, d, r, u, l)
    arr = [];
    if(d ~= 0)
        arr(1) = retvisited(startlocation(1)+1, startlocation(2));
    else
        arr(1) = -2;
    end
    if(r ~= 0)
        arr(2) = retvisited(startlocation(1), startlocation(2)+1);
    else
        arr(2) = -2;
    end
    if(u ~= 0)
        arr(3) = retvisited(startlocation(1)-1, startlocation(2));
    else
        arr(3) = -2;
    end
    if(l ~= 0)
        arr(4) = retvisited(startlocation(1), startlocation(2)-1);
    else
        arr(4) = -2;
    end
    [val, idx] = max(arr);
    if(idx == 1)
        dp = 1;
    else
        dp = 0;
    end
    
    if(idx == 2)
        rp = 1;
    else
        rp = 0;
    end
    
    if(idx == 3)
        upp = 1;
    else
        upp = 0;
    end
    
    if(idx == 4)
        lp = 1;
    else
        lp = 0;
    end
end

function placestep(position,i)
% This function will plot a insert yellow rectangle and also print a number in this rectangle. Use with plotmap/viewmap. 
position = [16-position(1) position(2)];
position=[position(2)+0.1 position(1)+0.1];
rectangle('Position',[position,0.8,0.8],'FaceColor','y');
c=sprintf('%d',i);
text(position(1)+0.2,position(2)+0.2,c,'FontSize',10);
end
