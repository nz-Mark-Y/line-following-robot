%% This is a shell that you will have to follow strictly. 
% You will use the plotmap() and viewmap() to display the outcome of your algorithm.

% Load sample_data_map_8, three variables will be created in your workspace. These were created as a 
% result of [m,v,s]=dfs('map_8.txt',[14,1],[1,18]);
% The solution can be viewed using 
% plotmap(m,s) 

%startlocation and targetlocation are both arrays (y-coordinate,x-coordinate) ie. rows and cols, top left of the map is (0,0)
function [retmap,retvisited,retsteps] = Astar( mapfile,startlocation,targetlocation)
	
	map = map_convert(mapfile); %map is the mapfile returned as a matrix
	
    [maprows,mapcols]=size(map); %dimensions of the map
	
    retmap = map;
    retvisited = ones(maprows,mapcols);%retvisited is a matrix that shows which cells have been visited, '0' means visited, '1' means not visited
    retsteps = []; %retsteps is the vector of steps to reach the target
	
	%The array of currently discovered nodes that have not been evaluated yet.
    %Initially, only the start node is stored in this array.
    openSet = cell(1);    
	
	%Start node is added to openSet
    openSet = [{startlocation}];
    
    %The set of nodes that have already been evaluated
    closedSet = cell(1);
    closedSetIndex = 0;
    
    %The set of nodes on the map that are tracks and can be travelled on(ie. not walls)
    tracksSet = cell(1);
    tracksSetIndex = 0;
    
	%All track nodes on the map are added to the tracksSet array
    for i = 1:maprows
        for j = 1:mapcols
            if map(i,j) == 0
                tracksSetIndex = tracksSetIndex + 1;
                tracksSet{tracksSetIndex,1} = [i,j];				
				
            end
        end
    end
	
    %contains the node from which the current node can be reached most easily
    cameFromNode = cell(1);
    
    %For each node, the cost of getting from the start node to that node.
    gScore = zeros(maprows,mapcols);
	
	%For each node, the total cost of getting from the start node to the goal by passing by that node. f(n)= g(n) + heuristic(n)
    fScore = zeros(maprows,mapcols);
	
	%The gScore of the start location
    gScore(startlocation(1),startlocation(2)) = 0; % The cost of going from the start node to the start node is zero
	
    fScore(startlocation(1),startlocation(2)) = heuristic_score_estimate(startlocation,targetlocation); %for the start node, the fScore is fully heuristic
        
    [~,s] = size(openSet);
    %While there are still nodes that are needed to be searched, keep looping
    while(s~=0)        
        %newcurrent is the node in openSet with lowest fScore, a form of priority queueing
        newcurrent = openSet{1}; 
		minimumfScore = fScore(openSet{1}(1),openSet{1}(2));
		% loop through openSet and find the node with the lowest fScore
		for i = 1:s			
			if(fScore(openSet{i}(1),openSet{i}(2)) < minimumfScore)
				newcurrent = openSet{i};
				minimumfScore = fScore(openSet{i}(1),openSet{i}(2));				
			% If the two fScores are the same then the one with the lower heuristic score is selected, as there is more certainty with that value
			elseif(fScore(openSet{i}(1),openSet{i}(2)) == minimumfScore)
				if(heuristic_score_estimate(openSet{i}, targetlocation) <= heuristic_score_estimate(newcurrent, targetlocation))
					newcurrent = openSet{i};
					minimumfScore = fScore(openSet{i}(1),openSet{i}(2));					
				end
			end
		end
		%current is the node in openSet with lowest fScore, a form of priority queueing
		current = newcurrent;
		
        %If the targetlocation is found, exit the loop
        if(current == targetlocation)            
            retvisited(current(1), current(2)) = 0;
			retsteps = reconstruct_path(current,cameFromNode,startlocation);%get the vector of steps needed to go from start to target			
            break;
        end

        % The new current node is deleted from openSet 
        for i = 1:s
            if current == openSet{i}
                openSet(i,:) = [];
                break;
            end
        end

        %The new current node is added to closedSet 
		retvisited(current(1),current(2)) = 0; %the location of the current node is set as being 'visited'
        closedSetIndex = closedSetIndex + 1;
        closedSet{closedSetIndex} = current;
		
        %get the current node's neighbour nodes
        neighbours = getNeighbours(current,tracksSet);
        neighbourSize = size(neighbours);
		neighbourNum = neighbourSize(1);
        
        for i = 1:neighbourNum
            
            %If the neighbour node has already been evaluated then ignore it
			alreadyExists = any(cellfun(@(x) isequal(x, neighbours(i,:)), closedSet));
            
			if (alreadyExists)
                
            else            
				%Updating the current gScore 
				newgScore = gScore(current(1),current(2)) + 1;
				
				%If the neighbour node is not in openSet then it is added in
				alreadyExists = any(cellfun(@(x) isequal(x, neighbours(i,:)), openSet));				
				if (~alreadyExists)
					openSet = [openSet;{neighbours(i,:)}];										
				end				
				%This path is the best until now so we store it
				cameFromNode{neighbours(i,1),neighbours(i,2)} = current;				
				gScore(neighbours(i,1),neighbours(i,2)) = newgScore;
				fScore(neighbours(i,1),neighbours(i,2)) = newgScore + heuristic_score_estimate(neighbours(i,:),targetlocation);
			end
		end
		[~,s] = size(openSet);
    end    
end

function steps = reconstruct_path(current,cameFromNode,startlocation)
	steps = [current];
	looping = 1;
	while (looping == 1)
		if(isequal(current,startlocation))%once we've traced back to the start, break the loop
			break
		end
		current = cell2mat(cameFromNode(current(1),current(2)));
		steps = [steps; current];
	end	
	steps = flipud(steps);%reverse the array since we want the steps from the start to the target location
end	

%The heuristic_cost_estimate() is a function that estimates the cost of the cheapest path from node n to the goal node
%Heuristic means that the function is not perfect but performs well enough for the requirements
function score = heuristic_score_estimate(n,goal)
    distance = sqrt((goal(1)-n(1))^2 + (goal(2)-n(2))^2); %this formula finds the straight line distance between the two nodes
	score = round(distance);
end

%This function gets the neighbours of the current input node
function neighbours = getNeighbours(current,tracksSet)
    
	s = size(tracksSet);
	s = s(1);
    neighbours = [];
    
    for i = 1:s %loop through each node in tracksSet and check if it is a neighbour of the current node
        if (tracksSet{i,1}(1)  == current(1) && tracksSet{i,1}(2)+1 == current(2)) 
            neighbours = [neighbours;tracksSet{i,1}];
        elseif (tracksSet{i,1}(1) == current(1) && tracksSet{i,1}(2)-1 == current(2))
			neighbours = [neighbours;tracksSet{i,1}];
		elseif (tracksSet{i,1}(1) == current(1)+1 && tracksSet{i,1}(2) == current(2))
			neighbours = [neighbours;tracksSet{i,1}];
		elseif (tracksSet{i,1}(1) == current(1)-1 && tracksSet{i,1}(2) == current(2))
			neighbours = [neighbours;tracksSet{i,1}];
		end
    end
end