% Genera paradas aleatorias en el borde del recorrido
load('usborder.mat','x','y','xx','yy');
rng(3,'twister') 
nStops = 100; % Número de paradas 
stopsLon = zeros(nStops,1); % Coordenada x
stopsLat = stopsLon; % Coordenada y
n = 1;
while (n <= nStops)
    xp = rand*1.5;
    yp = rand;
    if inpolygon(xp,yp,x,y) % Prueba si está dentro del borde 
        stopsLon(n) = xp;
        stopsLat(n) = yp;
        n = n+1;
    end
end

% Calcula las distancias entre puntos 
idxs = nchoosek(1:nStops,2);
dist = hypot(stopsLat(idxs(:,1)) - stopsLat(idxs(:,2)), ...
             stopsLon(idxs(:,1)) - stopsLon(idxs(:,2)));
lendist = length(dist);

% Crea una gráfica donde las paradas son vértices y los viajes son aristas 
G = graph(idxs(:,1),idxs(:,2));
figure
hGraph = plot(G,'XData',stopsLon,'YData',stopsLat,'LineStyle','none','NodeLabel',{});
hold on
    % Dibuja el borde 
plot(x,y,'r-')
hold off

% Crea un problema de optimización que representa los viajes potenciales 
tsp = optimproblem;
trips = optimvar('trips',lendist,1,'Type','integer','LowerBound',0,'UpperBound',1);
tsp.Objective = dist'*trips;

% Crea las restricciones:
%   Cada parada debe tener dos viajes asociados (hay un viaje desde cada
%   parada y uno hasta cada parada)
constr2trips = optimconstr(nStops,1);
for stop = 1:nStops
    whichIdxs = outedges(G,stop); % Identifica viajes asociados con las paradas
    constr2trips(stop) = sum(trips(whichIdxs)) == 2;
end
tsp.Constraints.constr2trips = constr2trips;

opts = optimoptions('intlinprog','Display','off');
tspsol = solve(tsp,'options',opts)

% Crea una gráfica con las aristas de la solución y la superpone en la
% primera, donde estaban los viajes posibles 
tspsol.trips = logical(round(tspsol.trips));
Gsol = graph(idxs(tspsol.trips,1),idxs(tspsol.trips,2),[],numnodes(G));
hold on
highlight(hGraph,Gsol,'LineStyle','-')
title('Solución con Subtrayectos')

% Agregar restricciones a los subtrayectos para reducir el "camino" lo
% máximo posible
tourIdxs = conncomp(Gsol);
numtours = max(tourIdxs); % Número de subtrayectos 
fprintf('Subtrayectos: %d\n',numtours);
    %  Repetir hasta que sólo quede un trayecto
k = 1;
while numtours > 1 
    for ii = 1:numtours
        inSubTour = (tourIdxs == ii); 
        a = all(inSubTour(idxs),2); 
        constrname = "subtourconstr" + num2str(k);
        tsp.Constraints.(constrname) = sum(trips(a)) <= (nnz(inSubTour) - 1);
        k = k + 1;        
    end
    
    % Optimizar nuevamente
    [tspsol,fval,exitflag,output] = solve(tsp,'options',opts);
    tspsol.trips = logical(round(tspsol.trips));
    Gsol = graph(idxs(tspsol.trips,1),idxs(tspsol.trips,2),[],numnodes(G));
    
    % Graficar la nueva solución 
    hGraph.LineStyle = 'none'; % Eliminar el mapa anterior
    highlight(hGraph,Gsol,'LineStyle','-')
    drawnow

    % Número de subtrayectos después de la optimización 
    tourIdxs = conncomp(Gsol);
    numtours = max(tourIdxs); 
    fprintf('Subtrayectos: %d\n',numtours)    
end
title('Solución al eliminar subtrayectos');
hold off

% Calidad de la estructura de salida
disp(output.absolutegap)





