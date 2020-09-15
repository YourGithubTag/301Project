function D = dijkstra(G, pairs)
    % This function takes an adjacency matrix called G
    % and a p-by-2 matrix called pairs.
    % The pairs matrix will contain pairs of indices.
    % This function will determine the shortest distance from
    % the first index in the pair to the second index for
    % every pair in matrix pairs.
    %
    % The function will only return a p-by-1 matrix of shortest
    % distances. I could use it to also return the shortest path,
    % but I don't need that data, so only shortest distance is
    % calculated.

    p = size(pairs, 1);  % Number of pairs
    v = size(G, 1); % Number of vertices
    D = [];

    for i = 1:v
        for j = 1:v
            if G(i,j) <= esp
                G(i,j) = inf
            end
        end
    end

    for i = 1:p 
        dist = inf(1, v);
        seen = ones(1, v);
        not_seen = v;

        dist(1, pairs(i,1)) = 0;

        while not_seen > 0
            [distance index] = min(dist .* seen);

            if distance == inf
                break;
            end

            if index == pairs(i,2)
                break;
            end

            seen(index) = inf;
            not_seen = not_seen - 1;

            for n = 1:v
                if seen(n) == 1
                    alt = distance + G(index, n);
                    if alt < dist(n)
                        dist(n) = alt;
                    end
                end
            end
        end
        D = [D ; dist(pairs(i,2))];
    end