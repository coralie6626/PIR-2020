%% frontier test script
clear ;

%% Funtions
PList = [[10,10,50]]; %pos initial 
PList2 = [[10,30,50]]; %drone 2
n_step = 15;
Nscan= 720*180;
next_point = [];
safeDistance = 10;  %Valeur arbitraire
unseen = [];unseen_new = [];

for i = 1:n_step
    disp(strcat('Step ',num2str(i)));

    [front seen unseen] = f_run([PList;PList2],i);
    disp('seen : ');
    disp(length(seen));
    disp('unseen : ');
    disp(length(unseen));
    disp(PList(i,:));
    disp(PList2(i,:));
    
    %% Cluster
    [clusterIndexes,centers] = diviseBy2Clustering(front,30,1);
%     min_dist = pdist([PList(i,:);centers(1,:)],'euclidean');
    min_dist = 1000;
    
    next_center = 1;
    %% closest center
    for center = 1:length(centers(:,1))
       if pdist([PList(i,:);centers(center,:)],'euclidean') < min_dist
           min_dist = pdist([PList(i,:);centers(center,:)],'euclidean');
           next_center = center;
       end
    end
    

%     center_psb_points = centers(next_center,:);
% %    center_psb_points(3) = center_psb_points(3) + 15;
%     psb_points(:,1:3) = sphere(center_psb_points,r); 
%     front_in_cluster = get_front(front,clusterIndexes,next_center);
    %% Utility
    
%     disp('Utility...');
%     maxMesurements = 10; %Valeur fonctionnelle à tester et trouver
%     
%     for j = 1:length(psb_points)
%         utilityTab = viewUtility(front_in_cluster,psb_points(j,1:3),maxMesurements,Nscan, safeDistance);
%         psb_points(j,4) = sum(utilityTab(:,4));
%     end
%     
%     for t =1:length(PList(:,1))
%         for l = 1:length(psb_points)
%            if psb_points(l,1) == PList(t,1) && psb_points(l,2) == PList(t,2) && psb_points(l,3) == PList(t,3)
%                psb_points(l,4) = 0;
%            end
%         end
%     end
%     
    disp('Choosing next pos...');
%     lmax = findMax(psb_points);
%     bestPoints = [];
%     for k=1:length(lmax)
%         L = lmax(k);
%         bestPoints = [bestPoints; psb_points(L,1:3)];    
%     end
%     
%     next_point = PList(1,:);
% 
%     while ismember(next_point,PList,'rows') == 1
%         next_point = bestPoints(randi([1,length(bestPoints(:,1))],1),:);
%     end
    next_point = centers(next_center,:);
    next_point(3) = next_point(3)+15;
    PList=[PList;next_point];
    %% drone 2
    min_dist = 1000;
    
    next_center = 1;
    %% closest center
    for center = 1:length(centers(:,1))
       dist_to_othes= 0;
       if pdist([PList2(i,:);centers(center,:)],'euclidean') < min_dist
           min_dist = pdist([PList2(i,:);centers(center,:)],'euclidean');
           next_center = center;
       end
    end
    next_point = centers(next_center,:);
    next_point(3) = next_point(3)+15;
    PList2=[PList2;next_point];

end

figure('name',"Final")
mustPoints = [];
FPlength = n_step;
siz= [200,200,200];
draw = [PList;seen;unseen;mustPoints];
Length = length(draw(:,1));
scale = 5 * ones(Length,1); %largeur des points détectés
scale(1:FPlength) = 70; %largeur du point drone
scale(length(seen(:,1))+1:Length) = 5; %largeur des points non détectés
color = 0.5 * ones(Length, 3); %couleur des points non détectés
color(1:FPlength,:) = repmat([1,0,0],FPlength,1); %couleur du point drone
color(1+FPlength:length(seen(:,1)),:) = repmat([0,0,1],length(seen(:,1))-FPlength,1); %couleur des points détectés
scatter3(draw(:,1),draw(:,2),draw(:,3),scale,color,'filled','o');
axis([1,siz(1),1,siz(2),1,siz(3)]);
% main function

function [front seen unseen] = f_run(PList,n_step)
    maxAlgos = 18;
    maxEnvs = 4;
    allEvaluations = zeros(maxAlgos, 5, maxEnvs, 3);
    for run = 3

    filtering = 0;
    perfectEnv = double(run==1);
    lastEnv = 0;
    
    resolution = (run-1)*[360, 90]; %nombre de raycast faits [horizontal, vertical]

    for env=6
        %% load env   

        drone_maxDist = 200; %distance maximale ? laquelle points sonts detectés
        drone_hor_view = [0, 360]; %angle de vue du drone ? l'horizontal [de, à] partant de l'axis X (sens mathématique)
        drone_ver_view = [0, -90]; %angle de vue du drone au vertical [de, à] partant de l'axis X (sens mathématique)

        disp(strcat('Preparing Environment',num2str(env),'...'));

        switch env
            case 1 %perfect frontier: 400
                load env1.mat          
            case 2 %perfect frontier: 60 * Pi = 188.5
                load env2.mat
            case 3 %perfect frontier: 100 * Pi = 314
                load env3.mat
            case 4
                load env4.mat  
            case 5
                load env5.mat
            case 6
                load env66.mat;
                 drone_maxDist = 30;
                 mustPoints = [];
    %              FlightPath = [[10,10,60];[10,180,60];[180,10,60];[180,180,60];[125,125,60]];
    %              mustPoints = [];
    %              block = zeros(180*180,3);
    %              k= 1;
    %              for i = 1:180
    %                  for j = 1:180
    %                     z(i,j) = round(i/10*sin(i/10)+j/10*sin(j/10))+35;
    %                     block(k,1)=i;
    %                     block(k,2)=j;
    %                     block(k,3)=z(i,j);
    %                     k=k+1;
    %                  end
    %              end
    %              
    %              air2 = [];
    %              air = [];
    %              unk = [];   
                   FlightPath = PList;
        end
        %% Algo go

        % 0=libre, 1=occupée -1=inconnu
        FPlength = length(FlightPath(:,1));
        world = zeros(200,200,200);
        siz = size(world); %pour racourcir du code plus tard
        seenWorld = -ones(siz);
        world = insert(world, block, 1);
        world = insert(world, air2, 0);

        if perfectEnv == 1
                seenWorld = insert(seenWorld, air, 0);
                seenWorld = insert(seenWorld, block, 1);
                seenWorld = insert(seenWorld, unk, -1);
        else
            for posCounter = 1:FPlength %Pour tous les points o? le robot se rend
                drone_pos = FlightPath(posCounter,:);
                phi_step = (drone_hor_view(2) - drone_hor_view(1))/resolution(1);%Pas de l'angle de rotation en phi et têta
                theta_step = (drone_ver_view(2) - drone_ver_view(1))/resolution(2);
                for phi = drone_hor_view(1):phi_step:drone_hor_view(2)
                    for theta = drone_ver_view(1):theta_step:drone_ver_view(2)
                        rc = raycast(world, drone_pos, phi, theta, drone_maxDist);
                        for r = 1:length(rc(:,1)) %tous points passés par le raycast ne sont plus inconnus et donc enregistr? dans seenWorld
                            seenWorld(rc(r,1), rc(r,2), rc(r,3)) = world(rc(r,1), rc(r,2), rc(r,3));
                        end
                    end
                end
            end
        end

            algo=7; % algo 7 
            disp(strcat('Now working on: Environment',num2str(env),',','Algorythm',num2str(algo)));

            seen = zeros(resolution(1)*resolution(2)+FPlength,3); %tous points occupés et detectés par le drone + position du drone
            seen(1:FPlength,:) = FlightPath; 
            unseen = zeros(siz(1)*siz(2)*siz(3),3); %touts points occupés et non detectés
            front = zeros(round((2/3)*pi*drone_maxDist^3),3); %touts points considérés comme frontière

            e = 1;
            f=1; %compteur de vraie longeur de front
            s=FPlength+1; %compteur de vraie longeur de seen
            u=1; %compteur de vraie longeur de unseen
            tp=1;
            for x = 1:siz(1)
                for y = 1:siz(2)
                    for z = 1:siz(3)
                        if seenWorld(x,y,z) == 1

                            seen(s,:) = [x,y,z];
                            s = s+1;
                        elseif seenWorld(x,y,z) == 0
                            if isFrontier(seenWorld, [x,y,z], algo)
                                front(f,:) = [x,y,z];
                                f = f+1;
                            end
                        elseif world(x,y,z) == 1

                            unseen(u,:) = [x,y,z];
                            u = u+1;
                        end
                    end
                end
            end

            %coupage des array avec leur vraie longeur maintenant connue
            seen(s:end,:) = [];
            unseen(u:end,:) = [];
            front(f:end,:) = [];
            %alledges(e:end,:) = [];

            if perfectEnv == 0 && filtering == 1
                disp('Deleting small groups of fronts');
                frontMap = 0*world;
                for i=1:length(front(:,1)) %i allant de 1 ? la longueur d'une ligne de la matrice des frontières
                    p = front(i,:);
                    frontMap(p(1),p(2),p(3)) = 1;
                end
                filterSmallGroups(frontMap, front, 5);
            end

            if lastEnv < env
                lastEnv = env;

                %graphique pour touts les points occupés détectés et non détectés
                if perfectEnv == 1
                    title = strcat('Overview in PerfectEnvironment Nr.' , num2str(env) , ' Step ', num2str(n_step));
                else
                    title = strcat('Overview in RealEnvironment Nr.' , num2str(env), '_Res[',num2str(resolution(1)),',',num2str(resolution(2)),']', ' Step ', num2str(n_step));
                end
                figure('name',title);
                draw = [seen;unseen;mustPoints];
                Length = length(draw(:,1));
                scale = 5 * ones(Length,1); %largeur des points détectés
                scale(1:FPlength) = 50; %largeur du point drone
                scale(length(seen(:,1))+1:Length) = 5; %largeur des points non détectés
                color = 0.5 * ones(Length, 3); %couleur des points non détectés
                color(1:FPlength,:) = repmat([1,0,0],FPlength,1); %couleur du point drone
                color(1+FPlength:length(seen(:,1)),:) = repmat([0,0,1],length(seen(:,1))-FPlength,1); %couleur des points détectés
    %           color(length(draw(:,1))-length(mustPoints(:,1)):end,:) = repmat([0,0.5,0],length(mustPoints(:,1))+1,1);
                scatter3(draw(:,1),draw(:,2),draw(:,3),scale,color,'filled','o');
                axis([1,siz(1),1,siz(2),1,siz(3)]);
                saveas(gcf,[pwd strcat('\SimulationResult\',title,'.fig')]);
                set(gcf, 'Visible', 'off');
            end


            %graphique montrant points occupés détectés et points frontièrs
            if perfectEnv == 1
                title = strcat('Frontiers in PerfEnv Nr.' , num2str(env) , ' Algo Nr.' , num2str(algo), ' Step ', num2str(n_step));
            else
                title = strcat('Frontiers in RealEnv Nr.' , num2str(env) , ' Algo Nr.' , num2str(algo), '_Res[',num2str(resolution(1)),',',num2str(resolution(2)),'] with filtering', ' Step ', num2str(n_step));
            end
            figure('name',title);
            draw = [seen;front];
            Length = length(draw(:,1));
            scale = 5 * ones(Length,1); %largeur des points détectés et frontières
            scale(1:FPlength) = 50; %largeur du point drone
            color = repmat([1,0.5,0],Length,1); %couleur des points frontièrs
            color(1:FPlength,:) = repmat([1,0,0],FPlength,1); %couleur du point drone
            color(1+FPlength:length(seen(:,1)),:) = repmat([0,0,1],length(seen(:,1))-FPlength,1); %couleur des points détectés
            scatter3(draw(:,1),draw(:,2),draw(:,3),scale,color,'filled','o');
            axis([1,siz(1),1,siz(2),1,siz(3)]);
            saveas(gcf,[pwd strcat('\SimulationResult\',title,'.fig')]);
            set(gcf, 'Visible', 'off');
    
    end
    disp(strcat('Res:',num2str(resolution),'_PerfectEnvironment:',num2str(perfectEnv)));
    disp('--------------------------------------------------------------');
    
    end
   

end

% functions

function points = comparePointClouds(pCloud1, pCloud2, shared)
%shared == 1: returns all shared points
%shared == 0: returns is pCloud1 without pCloud2
    points = pCloud1;
    p = 1;
    for i=1:length(pCloud1(:,1))
        if pointCloudContains(pCloud2,pCloud1(i,:)) == shared
            points(p,:) = pCloud1(i,:);
            p = p+1;
        end
    end
    points(p:end,:) = [];
end

function b = pointCloudContains(pCloud, P)
    b = 0;
    for i = 1:length(pCloud(:,1))
        if min(pCloud(i,:) == P) == 1
            b = 1;
            return;
        end
    end
end

function points = line(P1, P2)
    points = zeros(1+round(norm([P1(1)-P2(1),P1(2)-P2(2),P1(3)-P2(3)])),3);
    Dir = (P2 - P1)/max(abs(P2-P1));
    p = 1;
    for i = 0:max(abs(P2-P1))
        points(i+1,:) = round(P1 + i * Dir);
        p = p+1;
    end
    points(p:end,:) = [];
end

function points = cube(P1, P2)
%rend un nuage de points formant un cube avec les coins ? P1(1x3) et P2(1x3)
%très lent ? partir de 10^5 points
    points = zeros(abs(P1(1,1)-P2(1,1))*abs(P1(1,2)-P2(1,2))*abs(P1(1,3)-P2(1,3)),3);
    j = 1;
    for x = min(P1(1,1),P2(1,1)):max(P1(1,1),P2(1,1))
        for y = min(P1(1,2),P2(1,2)):max(P1(1,2),P2(1,2))
            for z = min(P1(1,3),P2(1,3)):max(P1(1,3),P2(1,3))
                points(j,:) = [x,y,z];
                j = j+1;
            end    
        end
    end
    points(j:length(points),:) = [];
end

function points = circle(pos, radius)
    points = zeros(round(2*pi*radius),3);
            i = 1;
            for x = pos(1)-radius-1:pos(1)+radius+1
                for y = pos(2)-radius-1:pos(2)+radius+1
                    if abs(distance(round([x,y,pos(3)]), pos) - radius) <= 0.5
                        points(i,:) = round([x,y,pos(3)]);
                        i = i+1;
                    end
                end
            end
    points(i:end,:) = [];
end

function points = sphere(pos, radius)
%rend un nuage de points formant une sphere avec les centre ? pos(1x3)
    points = zeros(round((4/3)*pi*radius^3),3);
    j = 1;
    for x = pos(1,1)-radius:pos(1,1)+radius
        for y = pos(1,2)-radius:pos(1,2)+radius
            for z = pos(1,3)-radius:pos(1,3)+radius
                if distance(pos, [x,y,z]) <= radius
                    points(j,:) = [x,y,z];
                    j = j+1;
                end
            end    
        end
    end
    points(j:length(points),:) = [];
end

function ins = insert(map, pointcloud, value)
%change la valeur des élements dans map indiqués par les élements de
%pointcloud(~x3) et rend la map chang? => met le nuage de point soit
%occup?, soit libre, soit inconnu dans la carte
    siz = size(map);
    for i = 1:length(pointcloud) %Pour tous les points du nuage ? insérer
        %Si toutes ses coordonnées sont positives et si le points est dans
        %la carte
        if min(pointcloud(i,:)) > 0 && pointcloud(i, 1) <= siz(1) && pointcloud(i, 2) <= siz(2) && pointcloud(i, 3) <= siz(3)
            %Le point de coordonnées du point i prend la valeur occupée,
            %inconnue ou libre
            map(pointcloud(i, 1), pointcloud(i, 2), pointcloud(i, 3)) = value;
        end
    end
    ins = map;
end

function ray = raycast(map, From, phi, theta, maxDist)
%fait un raycast dans map partant de From(1x3) en direction du point
%Towards(1x3). Tous points libres sont mis dans ray(~x3)
%le raycast se termine ou au bord de la map ou au premier point occup?
%il s'arrete aussi après la distance maxDist du point From
%rend touts points passés jusqu'? l'arret du raycast
    Dir = [cos(pi*phi/180)*cos(pi*theta/180), sin(pi*phi/180)*cos(pi*theta/180), sin(pi*theta/180)];
    ray = zeros(round(1.5*maxDist), 3);
    i = 1;
    Next = From+Dir;
    limit = size(map);
    if round(Next(1)) > limit(1) || round(Next(2)) > limit(2) || round(Next(3)) > limit(3) || round(min(Next)) < 1
        ray(i,:) = round(Next);
        ray(i+1:end,:) = [];
        return;
    end
    while not(distance(Next, From) >= maxDist || map(round(Next(1)),round(Next(2)),round(Next(3))) == 1)
        ray(i,:) = round(Next);
        i = i+1;
        Next = Next + Dir;
        if round(Next(1)) > limit(1) || round(Next(2)) > limit(2) || round(Next(3)) > limit(3) || round(min(Next)) < 1
            ray(i:end,:) = [];
            return;
        end
    end
    if (map(round(Next(1)),round(Next(2)),round(Next(3))) == 1)
        ray(i,:) = round(Next);
        ray(i+1:end,:) = [];
    else
        ray(i:end,:) = [];
    end
end

function dist = distance(P1, P2)
%distance entre deux points P1(1x3) et P2(1x3)
    dist = norm([P1(1)-P2(1), P1(2)-P2(2), P1(3)-P2(3)]);
end

%map : carte actuelles
%P : point dont on cherche les voisins
%value : 1 si on cherche les voisins occupés, 0 voisins vide, -1 inconnus
%v : type de voisinage (V6, V18, V26)
function nbrs = getNeighbours(map, P, value, v)
    nbrs = zeros(26,3);
    n=1;
    c = 3; %neighbour cube size 
    siz = size(map);
    for x = max(P(1)-(c-1)/2,1):min(P(1)+(c-1)/2,siz(1))
        for y = max(P(2)-(c-1)/2,1):min(P(2)+(c-1)/2,siz(2))
            for z = max(P(3)-(c-1)/2,1):min(P(3)+(c-1)/2,siz(3))
                if P(1) ~= x || P(2) ~= y || P(3) ~= z
                    Dist = distance([x,y,z],P);
                    
                    if Dist < 2 %voisinage <= 26
                        if v == 26 && map(x,y,z)==value 
                            nbrs(n,:) = [x,y,z];
                            n=n+1;
                        end
                        
                        if Dist < sqrt(3) %voisinage <= 18
                           if v == 18 && map(x,y,z)==value
                               nbrs(n,:) = [x,y,z];
                               n=n+1;
                           end                        
                           
                             if Dist < sqrt(2) %voisinage 6
                                if v == 6 && map(x,y,z)==value
                                    nbrs(n,:) = [x,y,z];
                                    n=n+1;
                                end
                             end
                        end
                    end
                end
            end
        end
    end
    nbrs(n:end,:) = [];
end

function b = isFrontier(map, P, algo)
%rend 1 si P(1x3) est consider? comme frontière dans map(~x3) et 0 sinon
    
%Choix de l'algo : 7
%     switch (mod((algo-1),9)+1)
%         case 1
%             voi_occ = 6;
%             voi_unk = 6;
%         case 2
%             voi_occ = 6;
%             voi_unk = 18;
%         case 3
%             voi_occ = 6;
%             voi_unk = 26;
%         case 4
%             voi_occ = 18;
%             voi_unk = 6;
%         case 5 
%             voi_occ = 18;
%             voi_unk = 18;
%         case 6
%             voi_occ = 18;
%             voi_unk = 26;
%         case 7
            voi_occ = 26;
            voi_unk = 6;
%        case 8
%             voi_occ = 26;
%             voi_unk = 18;
%         case 9
%             voi_occ = 26;
%             voi_unk = 26;
%     end
    
    occupied = getNeighbours(map, P, 1, voi_occ);
    unknown = getNeighbours(map, P, -1, voi_unk);
    occupied = length(occupied(:,1));
    unknown = length(unknown(:,1));
   
    if algo <= 9
        b = unknown>0 && occupied>0;
    end
%     if algo >= 10
%         siz = size(edges);
%         nbrs = edges(max(P(1)-1,1):min(P(1)+1,siz(1)), max(P(2)-1,1):min(P(2)+1,siz(2)), max(P(3)-1,1):min(P(3)+1,siz(3)));
%         %isEdge = avgEdges(P(1),P(2),P(3)) >= edgeThreshold(3) && avgEdges(P(1),P(2),P(3)) <= edgeThreshold(4);
%         %isEdge = isEdge || (Edges(P(1),P(2),P(3)) >= edgeThreshold(1) && Edges(P(1),P(2),P(3)) <= edgeThreshold(2));
%         b = max(max(max(nbrs))) >= edgeThreshold(1) && max(max(max(nbrs))) <= edgeThreshold(2) && unknown>0 && occupied>0;
%     end
end

%map : carte actuelle
%point : point de départ
%goal : point d'arrivée
%maxSteps : maximum du nb de frontières entre point et goal autoris?
%value :
%voisinage : V6, V18 ou V26
function b = ConnectionTo(map, point, goal, maxSteps, value, voisinage)
%returns -1 if not connected and else the number of steps required
    b = -1;
    if (min(point == goal) == 1)
        b = 0;
        return;
    end
    if value == inf %map is a pointCloud with only relevant points
        if not(pointCloudContains(map, point) && pointCloudContains(map, goal))
            return;
        else
            curPoint = point;
            b = 1;
            while b <= maxSteps
                pointsFound = 0;
                for i=1:length(map(:,1))
                    switch voisinage
                        case 6
                            maxDist = sqrt(2);
                        case 18
                            maxDist = sqrt(3);
                        case 26
                            maxDist = 2;
                    end
                    if distance(goal, map(i,:)) < distance(goal, curPoint) && distance(curPoint, map(i,:)) < maxDist
                        curPoint = map(i,:);
                        pointsFound = pointsFound+1;
                    end
                end
                    if pointsFound > 0
                        if min(curPoint==goal)==1
                            return;
                        else
                            b = b+1;
                        end
                    else
                        b = -1; 
                        return;
                    end
               
            end
        end
    else %map i a map and only points where map(x,y,z)==value are relevant
        siz = size(map);
        if max(point(1),goal(1)) > siz(1) || max(point(2),goal(2)) > siz(2) || max(point(3),goal(3)) > siz(3) || min([point,goal]) < 1
            return;
        else
            curPoint = point;
            b = 0;
            while b <= maxSteps
                
                b=b+1;
                if min(curPoint==goal) == 1
                    return;
                end
                
                nbrs = getNeighbours(map, curPoint, value, voisinage);
                %length(nbrs(:,1)) nombre de voisin
                if length(nbrs(:,1)) > 0
                    closestDist = distance(curPoint, goal);
                    for i=1:length(nbrs(:,1))
                        if distance(nbrs(i,:),goal) < closestDist
                            curPoint = nbrs(i,:);
                        end
                    end
                    if closestDist == distance(curPoint, goal)
                        b = -1;
                        return;
                    else
                        
                    end
                else
                    b=-1;
                    return;
                end
            end
        end
    end
    b = -1;
end

function filtered = filterSmallGroups(map, front, minSize)
    filtered = front;
    f = 1;
    pCloud = front;
    for i = 1:length(pCloud(:,1))
        longestConnection = 0;
        for j = 1:length(pCloud(:,1))
            longestConnection = max(ConnectionTo(map, pCloud(i,:), pCloud(j,:), minSize,1,6),longestConnection);
        end
        if longestConnection >= minSize
            filtered(f,:) = pCloud(i,:);
            f=f+1;
        end
    end
    filtered(f:end,:) = [];
end

%% Utility

function frontierUtility = viewUtility(front, FlightPath, maxMeasurements, Nscan, safeDistance)
    size = length(front);
    frontierUtility = zeros(size,4);                                               %Initalisation à zero du tableau
    maxDistance = sqrt(Nscan/(4*pi*maxMeasurements));
    %disp(maxDistance); (for tests)
    for i = 1:length(FlightPath(:,1))
        pos = FlightPath(i,:);
        for f = 1:size
            coord = front(f,:);                                                           %on récupère les coordonées de la frontière
            r = distance(pos, coord);                                              %r (distance drone/frontière)

            if r < safeDistance                                                             %Calcule de l'utilité
                u = 0;
            elseif r >= safeDistance && r < maxDistance
                u = maxMeasurements;
            else 
                u = Nscan/(4*pi*r^2);
            end
            if(u > frontierUtility(f,4))
                frontierUtility(f,:) = [coord,u];                                               %On remplis le tableau avec les coordonées de la frontière et son utilité
            end
        end
    end
end
function lmax = findMax(utilityTab)            %fonction qui cherche l'utilité maximale dans le tableau et renvoie la ligne qui correspond
    max =0;
    lmax= 0;
    for i = 1:length(utilityTab)
        if utilityTab(i,4) > max
            lmax = i;
            max = utilityTab(i,4);
        elseif utilityTab(i,4) == max
            lmax = [lmax, i];
        end
    end

end

%% Clusterisation

function centroid = computeCentroid(clusterPoints,center)
    %Pour tous les points, si le points est plus proche du centre que le
    %point courant, c'est le nouveau centroid
    centroid = [clusterPoints(1,1,1),clusterPoints(1,1,2),clusterPoints(1,1,3)];
    for j=2:length(clusterPoints(1,:,1))
        point = [clusterPoints(1,j,1),clusterPoints(1,j,2),clusterPoints(1,j,3)];
        if distance(point,center) < distance(centroid,center)
            centroid = point;
        end
    end
end

%Returns the radius of a cluster, which is the distance between the
%centroid and the farest point
%points : the set of point in the cluster
%centroid : the centroid of the cluster
function radius = calculRadius(cluster, centroid)
    %center = [centroid(i,1,1),centroid(i,1,2),centroid(i,1,3)];
    radius = 0;
    for j=1:length(cluster(1,:,1))
        point = [cluster(1,j,1),cluster(1,j,2),cluster(1,j,3)];
        if(distance(point,centroid)>radius)
            radius = distance(point,centroid);
        end
    end
end

function center = calculateCenter(pointCloud)
    x=0;
    y=0;
    z=0;
    for i=1:length(pointCloud(:,1))
        x=x+pointCloud(i,1);
        y=y+pointCloud(i,2);
        z=z+pointCloud(i,3);
    end
    x = x./length(pointCloud(:,1));
    y = y./length(pointCloud(:,1));
    z = z./length(pointCloud(:,1));
    center = [x,y,z];
end

%returns a list of clusters of which the radius are smaller than maxDist/2
%returns the coordinates of the centers of the clusters
%points : the set of points to cluster
%maxDist : the maximum radius allowed for a cluster
function [clusterIndexes,centers] = diviseBy2Clustering(points,maxDist,reinit)
    persistent maxClusterIndex;
    if isempty(maxClusterIndex) || reinit==1
        maxClusterIndex = 0;
    end

    centers = zeros(1,3);
    clusterIndexes = ones(length(points(:,1)),1);
    clusters(1,:,:) = points;

    currentPointIndex = ones(2,1);

    center = calculateCenter(points);
    centroid = computeCentroid(clusters(1,:,:),center);
    if calculRadius(clusters(1,:,:),centroid) > (maxDist/2)
       clusterIndexes = kmeans(points,2);
       clusters = [];
        for e = 1:length(points(:,1))
            if points(e,:)~=zeros(1,3)
                c = clusterIndexes(e);
                clusters(c,currentPointIndex(c),:) = points(e,:);
                currentPointIndex(c) = currentPointIndex(c)+1;
            end
        end

       for t=1:2
        newPoints = [];
        for n = 1:(currentPointIndex(t)-1)
           if clusters(t,n,:)~=zeros(1,1,3)
                newPoints(n,:) = [clusters(t,n,1),clusters(t,n,2),clusters(t,n,3)];
           end
        end
        [divisedIndexes,divisedCenters] = diviseBy2Clustering(newPoints,maxDist,false);

        for j=1:length(newPoints(:,1)) %Pour tous les points du cluster i
           point = [newPoints(j,1),newPoints(j,2),newPoints(j,3)];
           tmp = find(ismember(points,point,'rows'));
           idx = divisedIndexes(j,1);
           centers(idx,:) = divisedCenters(idx,:);
           clusterIndexes(tmp,1) = idx;
        end
       end

    else
        clusterIndexes = maxClusterIndex+clusterIndexes;
        centers(max(clusterIndexes),:) = round(center,0);
        maxClusterIndex = maxClusterIndex+1;
    end
end

function front_in_cluster= get_front(front,indexTab,index) %donner la liste des frontières appartenant à un cluster
    front_in_cluster = [];
    for  i = 1:length(front)
        if indexTab(i) == index
           front_in_cluster = [front_in_cluster;front(i,:)];
        end
    end
end
