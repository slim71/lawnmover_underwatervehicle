function  M = swing(puntoDes,puntoCorr,dist,angle)
%Generazione di waypoint aggiuntivi supponendo di conoscere il punto finale
% desiderato sul transetto
%
%   -puntoDes: punto desiderato sulla traiettoria rettilinea; 1x3 [m m m]
%   -puntoCorr: punto stimato di posizione corrente; 1x3 [m m m]
%   -dist: distanza tra waypoint; [m]

%     %lunghezza segmento
%     lunghezza= sqrt((puntoCorr(1) - puntoDes(1))^2 +(puntoCorr(2)-puntoDes(2))^2);% + (puntoCorr(3) - puntoDes(3))^2); 
%     
%     p = floor(lunghezza/dist); %numero waypoint

%     distN = puntoCorr(1) - puntoDes(1); %distanza su N tra punto iniziale e finale
%     distE = puntoCorr(2) - puntoDes(2); %distanza su E tra punto iniziale e finale
%     distD = puntoCorr(3) - puntoDes(3); %distanza su D tra punto iniziale e finale
    
    a = deg2rad(angle);
    rot = [cos(a) sin(a) 0; 
           -sin(a) cos(a) 0;
             0 0 1];
         
    calc = rot*(puntoCorr-puntoDes)';
    distN = calc(1);
    distE = calc(2);
    p = floor(abs(distE)/0.5);
    
    deltaE = distE/p; %distanza tra waypoints su East
    deltaN = distN/p; %distanza tra waypoints su North
%     deltaD = distD/p; %distanza tra waypoints su Down
    
    matrix = zeros(p-1,3);
   
    for i = 1:p-1 %possiamo fermarci a p-1 perché waypoint su traiettoria già fornito
        waypoint = puntoCorr - i * [deltaN deltaE 0];
        waypoint(3) = puntoDes(3);%deltaD];
        matrix(i,:) = waypoint;%matrix = cat(1, matrix, waypoint);
    end
    %matrix = cat(2, matrix, puntoDes);
    M = matrix;   
   
end
