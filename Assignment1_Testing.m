% % for i = 1:length(milestones_large1)
% %     for j = 1:lenght(milestones_large1)
% % %         for k =1:length(map1)
% % 
% % 
% % 
% % %         end
% %     end
% % end
% 
% sorted_milestones_large = sortrows(milestones_large , 1);
% dist_large = zeros(length(milestones_large) , length(milestones_large));
% adj_large = zeros(length(milestones_large) , length(milestones_large));
% 
% for i = 1: 50 %length(sorted_milestones_large)
% 
% map1 = map(map(:,1)>sorted_milestones_large(i,1)-0.5 & map(:,3)<sorted_milestones_large(i,1)+3 , :);
%     for j = 1:length(sorted_milestones_large)
% 
%         if j==i
%             dist_large(i,j) = 0;
%             adj_large(i,j) = 0;
%         end
%        
% 
%         if (sorted_milestones_large(j,1))> sorted_milestones_large(i,1)+6
%             dist_large(i, [j:end]) = 0;
%             adj_large(i,[j:end]) = 0;
%             continue;
%         else
%             Edge1 = [0 0 0 0];
%             Edge1 = [sorted_milestones_large(i,:) sorted_milestones_large(j,:)];
%             for k =1:length(map1)
% 
%                 [colliding, pt ] = EdgeCollision( Edge1, map1(k,:), 0 );
% 
%                 if colliding == 1
%                     dist_large(i,j) = 0;
%                     adj_large(i,j) = 0;
% 
%                 else
%                     dist_large(i,j) = sqrt((sorted_milestones_large(i,1) - sorted_milestones_large(j,1))^2 +( sorted_milestones_large(i,2) - sorted_milestones_large(j,2))^2);
%                     adj_large(i,j) = 1;
%                     
%                 end
%             end
%         
%         end
%     end
%     
% end
% 
% 



row = 50;
col = 50;
map = maze(row,col);
start = [0.5, 1.0];
finish = [col+0.5, row];
milestones = [start; finish];  % each row is a point [x y] in feasible space
edges = [];  % each row is should be an edge of the form [x1 y1 x2 y2]

h = figure(2);clf; hold on;
plot(start(1), start(2),'go')
plot(finish(1), finish(2),'rx')
show_maze(map,row,col,h); % Draws the maze
drawnow;

fprintf("Attempting large %d X %d maze... \n", row, col);
tic;        
% ------insert your optimized algorithm here------

nS = 2000;  % number of samples to try for milestone creation  % each row is a point [x y] in feasible space

% ------insert your PRM generation code here-------


%% Uniform point generation 

X = zeros(row, col);
X(:,1) = zeros(row,1);
 
for i = 1:col
X(:,i) = (i)*ones(row,1);

end

Y = 1;
 for i = 2: col
     Y = [Y ; i];
 end

milestones_large =[];
 for i = 1:length(X)
     hold on;
     scatter(X(:,i) , Y, "*" , "black")
     milestones_large = [ milestones_large ; X(:,i) Y] ;
 end




%%% Adjacency and distance matrix
n = length(milestones_large) ;
adj_test = zeros(n , n);

for i = 1:n


    adj_test(i,i) = 0;

    if i~=n

        if 0 == CheckCollision(milestones_large(i,:), milestones_large(i+1 , :) , map)
            adj_test(i,i+1) = 1;
        end
    end

    if i~=1
        if  0 == CheckCollision(milestones_large(i,:) , milestones_large(i-1 , :), map)
            adj_test(i,i-1) = 1;
        end
    end

    if i< n-length(Y)

        if 0 == CheckCollision(milestones_large(i,:), milestones_large(i+length(Y) , :) , map)
            adj_test(i,length(Y)+i) = 1;
        end
    end

end


adj_test = adj_test+adj_test';


%%% Djikstra algo

start = 1; %start at node 1 of milestones 
finish = length(milestones_large); %finish at the last node of milestones
distance_test = adj_test;

n = length(milestones_large);
visited_nodes(1:n) = 0;
dist(1:n) = Inf;
pred(1:n) = 0;
spath_large = [];
dist(start) = 1;
unvisited = 1:n;

while ~isempty(unvisited)
    [~,current] = min(dist(unvisited));
    current = unvisited(current);

    if current == finish
        break;
    end

    unvisited = setdiff(unvisited, current);

    %update distance to all neighbors 
    neighbors = find(adj_test(current,:));

    for i=1:length(neighbors)
        neighbor = neighbors(i);

        if dist(neighbor)>dist(current) + distance_test(current,neighbor)
            dist(neighbor) = dist(current) + distance_test(current,neighbor);
            pred(neighbor) = current;

        end
    end
end

if pred(finish)~=0
   p = finish;
   i=1;
   while p ~= start
       spath_large(i) = p;
       p = pred(p);
       i = i+1;
   end
end

spath_large = fliplr(spath_large(1:i-1));

toc;

start = [0.5, 1.0];
finish = [col+0.5, row];

hold on;
if (~isempty(spath_large))
    for i=1:length(spath_large)-1
        plot(milestones_large(spath_large(i:i+1),1),milestones_large(spath_large(i:i+1),2), 'go-', 'LineWidth',3);
    end
plot([start(1) milestones_large(spath_large(1),1)] , [start(2) milestones_large(spath_large(1),2)], 'go-', 'LineWidth',3)
plot([finish(1) milestones_large(spath_large(end),1)] , [finish(2) milestones_large(spath_large(end),2)], 'go-', 'LineWidth',3)

end









