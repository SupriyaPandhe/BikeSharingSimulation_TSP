
clc; clear;
iii = 1;


%% Simulation for different station spacing and coverage area
for  station_spacing = 1:2:9

    %assumption: as the coverage size increases, the station spacing
    %increases because the demand decreases as coverage area increases (as population density decreses).
    size = station_spacing *10; 
    map_size = [size,size];
    station_map = zeros(map_size);

    %Set station locations with bikes and empty slots
    station_map = populate_station(station_map, station_spacing,station_spacing);
    bikes_per_station = 10;
    bikes_at_station = station_map .* bikes_per_station;
    empty_slots_per_bike = 2;
    empty_slots_at_station = station_map .* bikes_per_station *empty_slots_per_bike;
    total_number_bikes = sum(sum(bikes_at_station));
    bike_per_grid = total_number_bikes / (size*size); 

    %Scatter autonomous bikes accross the same sized mapsame about equivilent in both systems.
    %if not enough bikes for every grid
    
    if (bike_per_grid <1 )
        bike_spacing = 1;
        avaliable_bike_map = zeros(map_size);
        avaliable_bike_map = populate_station(avaliable_bike_map, bike_spacing,bike_spacing);
    else
        avaliable_bike_map = ones(size, size);
        avaliable_bike_map = avaliable_bike_map .* round(bike_per_grid);
    end
    
    total_time_cost (1,iii)  = (size/10) * (size/10); %Coverage area
    total_time_cost (2,iii)  = 0;% Autonomous model trip time
    total_time_cost (3,iii) = 0; % traditional model trip time
    total_time_cost (4,iii) = 0; % Walking time (on a city grid) - Manhattan distance 
    total_time_cost (5,iii) = 0; % Walking time (on a euclidien distance) aka taking short cuts

    %run a simulation for specified number of trips
  
    number_trips = 15;   % no of trips
    for trip_count = 1:number_trips;
        %generate random start and end location on the map
       
        start_coordinate = randi([1,map_size(1)],1,2);
        end_coordinate = randi([1,map_size(1)],1,2);

        %Calculate the time for the trip
        [autonomous_time_cost, avaliable_bike_map ]= autonomous_model_cost(map_size, avaliable_bike_map,start_coordinate, end_coordinate);
        [traditional_time_cost, bikes_at_station, empty_slots_at_station ] = traditional_model_cost(map_size, ...
            bikes_at_station, empty_slots_at_station,start_coordinate, end_coordinate);
        only_walking_time_cost = walking_cost(start_coordinate, end_coordinate);
        only_walking_euclidien_time_cost = walking_cost_euclidien (start_coordinate, end_coordinate);

        %calcalate the total costs for all the trips
        total_time_cost (2,iii) = total_time_cost(2,iii) + autonomous_time_cost;
        total_time_cost (3,iii) = total_time_cost (3,iii) + traditional_time_cost;    
        total_time_cost (4,iii) = total_time_cost (4,iii) + only_walking_time_cost;    
        total_time_cost (5,iii) = total_time_cost (5,iii) + only_walking_euclidien_time_cost;    

    end
    iii= iii+1;
end

%average the trip time
total_time_cost(2,:)= total_time_cost(2,:)/number_trips; %autonomous
total_time_cost(3,:)= total_time_cost(3,:)/number_trips; %traditional 
total_time_cost(4,:)= total_time_cost(4,:)/number_trips; %walking manhattan 
total_time_cost(5,:)= total_time_cost(5,:)/number_trips; %walking euclidien

% 
% %plot the results
% figure(1);
% plot (total_time_cost(1,:), total_time_cost(2,:),'b'); hold on
% plot (total_time_cost(1,:), total_time_cost(3,:),'r'); hold on
% plot (total_time_cost(1,:), total_time_cost(4,:),'k'); hold on
% plot (total_time_cost(1,:), total_time_cost(5,:),'-.g')
% legend('Autonomous Model','Traditional Station Model', 'Walking Streets', 'Walking Euclidien');
% title('Average Trip Time vs Systam Coverage Area')
% xlabel('Bike System Coverage Area (km^2)');
% ylabel('Average trip time (minutes)');


%fit exponential 
f2 = fit(total_time_cost(1,:)', total_time_cost(2,:)','exp1'); %autonomous 
f3 = fit(total_time_cost(1,:)', total_time_cost(3,:)','exp1'); %Traditional 
f4 = fit(total_time_cost(1,:)', total_time_cost(4,:)','exp1'); %walking directly manhattan
f5 = fit(total_time_cost(1,:)', total_time_cost(5,:)','exp1'); %walking euclidien

%fitted exponential curve with scatter points
figure(2)
scatter(total_time_cost(1,:)', total_time_cost(2,:)','o','filled','b'); hold on;
plot(f2,'b'); hold on;
scatter(total_time_cost(1,:)', total_time_cost(3,:)','o','filled','r'); hold on;
plot(f3,'r'); hold on;
scatter(total_time_cost(1,:)', total_time_cost(4,:)','o','filled','k'); hold on;
plot(f4,'k'); hold on;
scatter(total_time_cost(1,:)', total_time_cost(5,:)','o','filled','g'); hold on;
plot(f5,'g'); 

legend('Autonomous Model','Autonomous Model - Curve fitting',...
    'Traditional Station Model','Traditional Station Model-Curve fitting',...
    'Walking Streets', 'Walking Streets-Curve Fitting',...
    'Walking Euclidien','Walking Euclidien-Curve Fitting','Location', 'NorthEastOutside','FontSize',15);
title('Average Trip Time vs Systam Coverage Area','FontSize',20)
xlabel('Bike System Coverage Area (km^2)','FontSize',18);
ylabel('Average trip time (minutes)','FontSize',18);
grid on


%% %%functions%% %%

%% Pouplate stations in a grid

function [ map ] = populate_station(map, x_spacing, y_spacing ) 

%Populate station function places a "1" to represent a station at sepcified spacing
%The first and last columns are left blank to represent
%a minimum of 1 block talking to a station.

x_size = size(map,1);
y_size = size (map,2);

for iii = 2:x_spacing+1:x_size-1
    for jjj = 2:y_spacing+1:y_size-1
        map(iii,jjj) = 1;
    end
end

end

%% Creates matrix of size of the the map 
% assigns the distance to the end coordinate as the heuristic - manhattan 

function [ heuristic] = generate_manhattan_huristic( map_size, end_coordinate ) 

heuristic = zeros(map_size);

for iii = 1 : map_size(1)
    for jjj = 1: map_size(2)
        heuristic (iii,jjj) = pdist2([iii,jjj],end_coordinate,'cityblock');
    end
end

end

%% Creates matrix of size of the the map 
%assigns the distance to the end coordinate as the heuristic - Euclidian

function [ heuristic] = generate_euclidian_huristic( map_size, end_coordinate ) 


heuristic = zeros(map_size);

for iii = 1 : map_size(1)
    for jjj = 1: map_size(2)
        heuristic (iii,jjj) = pdist2([iii,jjj],end_coordinate);
    end
end

end


%% Use a A star styled search for the closest non zero location
%search for a non zero value which searchs in the direction indicated by the heuristic


function [object_cordinate] = find_closest_object(grid_of_objects, possible_movements, present_location, heuristic) 


%store if the present location is checked to prevent rechecking
checked = zeros(size (grid_of_objects));
checked(present_location) = 1;

movement_cost = 0;

%flag for end of search
found = false; 
%flag is for if no object is found
resign = false; 

%heuristic cost used in a star
heuristic_cost = heuristic(present_location(1),present_location(2) ) + movement_cost;

%stack of locations left to search
searched_stack = [heuristic_cost, movement_cost, present_location];

%stack of objects  found= [movement cost, y location, x location]
object_found = [];


%If a bike is found at present location
if (0 < grid_of_objects(present_location(1), present_location(2)))
    found = true;
    object_found= [0, present_location];
end


count = 0;
while  not(found) && not(resign)

        %check if stack is empty ie. no more left to check
        if (~size(searched_stack,1))
            resign = true;

        else
            %sort so the least total cost is on top
            searched_stack = sortrows(searched_stack);

            %pop the top of search stack
            next_travel = searched_stack (1, :);
            searched_stack (1, :) = [];

            %get new cost and coordinates
            cost = next_travel (2);
            y = next_travel (3);
            x = next_travel (4);
            %since we moved 1, add to cost
            cost = cost +1;
            %Make checked non zero to indicated we have traved to location 
            checked(y,x) = count;
            count = count + 1;

            %if an object is found
            if (0 < grid_of_objects(y,x))
                found = true;
                object_found = [cost+ heuristic(y,x), y, x];    
                %search all possible movements
            else
                for iii= 1: size (possible_movements,1)
                    y2 = y + possible_movements(iii,1);
                    x2 = x + possible_movements(iii,2);

                    if (y2> 0 && y2 < size(grid_of_objects,1)+1 && x2>0 && x2 < size(grid_of_objects,2)+1)
                        if (checked(y2, x2) == 0 && grid_of_objects(y2,x2)> -1)
                            total_cost = cost + heuristic(y2,x2);
                            searched_stack = [searched_stack ; total_cost, cost, y2, x2];
                            checked(y2,x2) = count;

                            %if bike is found
                            if (grid_of_objects(y2,x2)> 0)
                                 object_found = [cost, y2, x2];
                                 found = true;
                             end
                        end
                    end
                end
            end
        end
    end

    closest_object_found = sortrows(object_found);   

    %in case no object is found, return nothing
    object_find_check = size (closest_object_found);
    if(object_find_check)
        object_cordinate = closest_object_found(1,2:3);
end

end


%% calculate time cost for autonomoous bike model

function [ time_cost, bike_locations ] = autonomous_model_cost(map_size, bike_locations,start_coordinate, end_coordinate) 
%autonomous_model_cost function finds the cost to travel with an autobike
%cost = time to wait for bike to arrive + time to travel to destination

human_bike_speed = 15; %(km/hr)
autonomous_bike_speed = 3; %(km/hour);
grid_size = 100/1000 ; %in km *(100 meters /(1000 m/km))


bike_movements = [-1 0; % go up
0 -1;   % go left
1 0;    % go down
0 1];   % go right

%heuristic used for A* is manhattan because bike travels on city block
heuristic = generate_manhattan_huristic (map_size, end_coordinate);

%find the closest avaliable bike
closest_bike_location = find_closest_object(bike_locations, bike_movements, start_coordinate,heuristic);

%calculate the time for bike to arrive
blocks_traveled_autonomous = pdist2(closest_bike_location, start_coordinate, 'cityblock');
autonomous_time = blocks_traveled_autonomous* grid_size /autonomous_bike_speed; 

%caculcate the time for biking 
blocks_biked = pdist2(start_coordinate, end_coordinate, 'cityblock');
human_bike_time = blocks_biked * grid_size /human_bike_speed;

%convert minutes
time_cost = (autonomous_time + human_bike_time)* 60; 

%remove the closest bike
bike_locations (closest_bike_location(1),closest_bike_location(2))= bike_locations (closest_bike_location(1),closest_bike_location(2)) -1;
%put the bike in its new location at end coordinate
bike_locations (end_coordinate(1),end_coordinate(2))= bike_locations (end_coordinate(1),end_coordinate(2)) + 1;


end

%% traditional bike cost

function [ time_cost, bikes_at_station_location, empty_slots_at_station_location ] = traditional_model_cost(map_size, ...
    bikes_at_station_location, empty_slots_at_station_location,start_coordinate, end_coordinate)


%cost = time to walk to station with empty bike + time to bike to an empty 
%station near the end point + time to talk from station to end location

human_bike_speed = 15;
human_walk_speed = 3; %(km/hour);
grid_size = 100/1000 ; %in km *(100 meters /(1000 m/km))

walking_movements = [-1 0; % go up
0 -1;   % go left
1 0; % go down
0 1;% go right
%1 1;
%-1 1
% -1 -1
%         1 -1 
];

%generate heuristics for trip
heuristic1 = generate_euclidian_huristic(map_size, start_coordinate);

heuristic2 = generate_euclidian_huristic (map_size, end_coordinate);

%find the closest station with avaliable bikes
start_closest_station_location = find_closest_object(bikes_at_station_location, walking_movements, start_coordinate,heuristic1);

%find the closest station with an open slot
end_closest_station_location = find_closest_object(empty_slots_at_station_location, walking_movements, end_coordinate,heuristic2);

% Walking time
%calculate distance from start to closest location by walking
distance_traveled_walking = pdist2(start_closest_station_location, start_coordinate);

%add the distance from end station to end location
distance_traveled_walking = distance_traveled_walking + pdist2(end_closest_station_location, end_coordinate);

walking_time = distance_traveled_walking * grid_size /human_walk_speed; 


% Biking time 
%calculate distance and time from start to closest empty station by bike
blocks_biked = pdist2(start_coordinate, end_coordinate, 'cityblock');
%concert time
human_bike_time = blocks_biked * grid_size /human_bike_speed;

%convert minutes
time_cost = (walking_time + human_bike_time)* 60; 


% Update bike locations 
%move the bike from start station to end station

bikes_at_station_location(start_closest_station_location(1), start_closest_station_location(2) )=bikes_at_station_location(start_closest_station_location(1), start_closest_station_location(2) ) -1;
bikes_at_station_location(end_closest_station_location(1), end_closest_station_location(2) )= bikes_at_station_location(end_closest_station_location(1), end_closest_station_location(2) ) +1;

empty_slots_at_station_location(start_closest_station_location(1), start_closest_station_location(2)) = empty_slots_at_station_location(start_closest_station_location(1), start_closest_station_location(2)) +1;
empty_slots_at_station_location(end_closest_station_location(1), end_closest_station_location(2)) = empty_slots_at_station_location(end_closest_station_location(1), end_closest_station_location(2) )-1;

end

%% walking_cost finds the time to walk from start to end

function [ time_cost ] = walking_cost(start_coordinate, end_coordinate) 


human_walk_speed = 3; %(km/hour);
grid_size = 100/1000 ; %in km *(100 meters /(1000 m/km))

%walking distance
distance_traveled_walking = pdist2(start_coordinate, end_coordinate, 'cityblock');

%convert distance into a time based on average walking speed
walking_time = distance_traveled_walking * grid_size /human_walk_speed; 

%convert minutes
time_cost = walking_time * 60; 


end

%% Find the time to talk from start to end - Euclidien 

function [ time_cost ] = walking_cost_euclidien(start_coordinate, end_coordinate) 

human_walk_speed = 3; %(km/hour);
grid_size = 100/1000 ; %in km *(100 meters /(1000 m/km))

%talking distance
distance_traveled_walking = pdist2(start_coordinate, end_coordinate);

%convert distance into a time based on average walking speed
walking_time = distance_traveled_walking * grid_size /human_walk_speed; 

%convert minutes
time_cost = walking_time * 60; 


end
