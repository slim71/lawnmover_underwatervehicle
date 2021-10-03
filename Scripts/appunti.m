%% Inizializzazioni richieste esterne a Supervisor

% Modello
init_position = [pointP(1),pointP(2),0.15,0,0,deg2rad(angle)]';

%%
%WP d'emergenza
%margine di sicurezza di 2m
%distanza laterale tra WP d'emergenza di 0.5m
%rising time di 5s, velocità fissa a 0.2m/s -> 1m longitudinali per
%compiere 0.5m laterali ->4m long totali per tornare in transetto da 2m di
%distanza
%rising time e settling time con oscillazioni a 0.1m dal valore di riferimento
%dare due WP avanti come riferimento
%stringere errore di riconoscimento per ritorno in transetto?<=0.5m
%stringere precisione per ultimi 2 WP su transetto?1m

%% come scritto
% MI ARRIVA UNA MISURA
distFront = FLS_front*cos(theta_f);
distDown = FLS_front*sin(theta_f);

nedFLS = fix2localNED(orient(1),orient(2),orient(3))'*[distFront;0;distDown];
%usare prima e terza componente in NED fisso

-in condizioni nominali iniziali
    averageDepth - altitude - (altitude-FLS_down) se ()>=0.5
-in caso di rilevamento ostacoli devo aggiungere
    - (FLS_down - nedFLS(3)) se ()>=0.5
-dopo essere salito su un ostacolo:
    pos(3) - (altitude-FLS_down) se ()>=0.5
    
-normalmente:
    pos(3) - (altitude-FLS_down)
-per portarsi sopra un ostacolo:
    pos(3) - (altitude-FLS_down) - (FLS_down-nedFLS(3)
    
positioning -> 0
diving -> averageDepth-altitude + delta da down
transect -> " + delta da ostacoli
turn -> idem
surfacing -> 0

pointProf = pos(3) - (altitude-FLS_down) - (FLS_down-nedFLS(3);%averageDepth - FLS_down + (distDown - altitude);
pointApp = calcPoint(pos,or,nedFLS(1)); %punto relativo alle misure
%[pointApp(1), pointApp(2), pointProf]

idx = 1;
margin = 0.5;%anticipo sul cambio di profondità; [m]

%continuamente, ad ogni clock, valuto distanza veicolo dal primo hotpoint
%e se sono a distanza necessaria (entro un tot), cambio prof di riferimento
gotObstacles = (size(points)~= 0); %restituisce: [1 1] se ho ostacoli, [0 0] altrimenti
if gotObstacles(1) && gotObstacles(2) %ci sono punti rilevati per il controllo di altitudine
    distObsVeh = distance(pos+nedAxis,points(1));%distanza tra primo "ostacolo" e CG del veicolo
    
    if  abs(pos(3)-profs(1))/maxRatio <= distObsVeh < abs(pos(3)-profs(1))/maxRatio + margin
        pos_ref(3) = profs(1);%senza prima parte sennò dopo non continua a dare quel riferimento?
    end

    if checkRif(points(1)) %sono arrivato in quel punto, entro un limite; valutare su distanza?
        points = points(2:end,:);%metto invece per riga, prima era points(:,2:end);%pop in testa
    end
    
end

%controllo che non sia subito insormontabile
%SEMPRE COL VEICOLO
if (pos(3) - pointProf)/(distFront+nedAxis(1)) > maxRatio
    %parete, ossia vedo che nella distanza che mi separa dal punto non
    %riuscirei a portarmi alla profondità necessaria
    current_state(7) = 1;
else %se non è insormontabile da subito

    %valuto che il delta di profondità sia maggiore di un limite minimo di
    %risoluzione, e che la distanza tra due punti a profondità diverse sia
    %maggiore di una distanza minima

    %SE HO PUNTI IN POINTS
    if points ~= {}
        dist = distance(points(end),pointApp);
        
        if minGap <= abs(profs(end) - pointProf) %<= maxRatio*dist % posso programmare
            %spostamenti da un punto all'altro
            
            if dist < minDist %scoglietto tra due punti, di cui uno era già stato scansionato
                %ignoro allora il riferimento dato dal punto precedentemente
                %scansionato
                points = points(:,1:end-1);%tolgo punto precedente, ultimo in coda
            else %no scoglietto, devo considerare entrambi i punti
                if (points(end)-pointProf)/dist > maxRatio
                    %parete tra i due, ignoro il punto prima per portarmi
                    %subito al riferimento successivo; passerò nel punto
                    %ignorato non esattamente con un delta di prof
                    %desiderato ma con quello che si ottiene da
                    %delta_OB / OB * (distNeeded_B - (distNeeded_A+AB))
                    %AGGIUNGERE DISEGNO?
                    points = points(:,1:end-1);
                end
            end
            
            points = cat(2, points, pointApp);
        end
    
    %SE NON HO PUNTI IN POINTS
    else %points == {}
        if (pos(3) - pointProf) >= minGap %voglio considerarlo
            dist = distance([pos(1), pos(2)], pointApp);%calcolata dal CG
            if dist >= minDist+nedAxis(1) %considero che FLS montato a prua del veicolo
                points = cat(2, points, pointApp);
            end
        end
    end
end





function valDist = distance(p1,p2)
%p1,p2 considerati di due dimensioni
    valDist = sqrt((p1(1)-p2(1))^2 + (p1(2)-p2(2))^2);
end

function point = calcPoint(p,o,d)
    nedDist = fix2LocalNED(o(1),o(2),o(3))'*[d; 0; 0];
    point = [p(1) p(2)] + [nedDist(1) nedDist(2)];
end

%% come aggiustato in stateflow

obstacle %parete/ostacoli
during:
distFront = FLS_front*cos(deg2rad(theta_f));
distDown = FLS_front*sin(deg2rad(theta_f));
nedFLS = fix2LocalNED(orient(1),orient(2),orient(3))'*[distFront;0;distDown];
pointProf = pos(3) - (altitude-FLS_down) - (FLS_down-nedFLS(3));
pointApp = calcPoint(pos,orient,nedFLS(1));
nedAxis = (fix2LocalNED(orient(1),orient(2),orient(3))'*[semix; semiy; semiz])';
%gestione profondità in caso di almeno un ostacolo
if all(size(points))
    current_state(7) = 1;
    distObsVeh = distance(pos+nedAxis,points(1,:));
    if distObsVeh<(abs(pos(3)-profs(1))/maxRatio + safeMargin)
        pos_ref(3) = profs(1);
    end
    if checkRif(pos_ref,orient_ref,vel_ref,pos,orient,vel,errPosPos,errOrPos,errVelPos)
        points = points(2:end,:);
    end
else
    current_state(6) = 0;
end
%controllo nuove misure
if (pos(3) - pointProf)/(distFront+nedAxis(1)) > maxRatio
    current_state(7) = 1;
else
    if all(size(points))%points ~= {}
        distObs = distance(points(end,:),pointApp);
        
        if minGap <= abs(profs(end) - pointProf)
            if distObs < minDist || (profs(end)-pointProf)/distObs > maxRatio
                points = points(1:end-1,:);
            end
            
            points = cat(1, points, pointApp);
        end
        
    else
        if (pos(3) - pointProf) >= minGap
            distObsVeh = distance([pos(1), pos(2)], pointApp);
            if distObsVeh >= minDist+nedAxis(1)
                points = cat(2, points, pointApp);
            end
        end
    end
end