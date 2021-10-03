clear all
close all
clc

run data_current.m

view(3)
xlabel('North');
ylabel('East');
zlabel('Down,z');
grid
hold on

[pN, pE, pDepth] = geodetic2ned(initPoint(1), initPoint(2), initPoint(3),areaOfInterestCorner(1), areaOfInterestCorner(2), 0,wgs84Ellipsoid);
[aN, aE, aDepth] = geodetic2ned(surveyAreaCorner(1), surveyAreaCorner(2), 0, areaOfInterestCorner(1), areaOfInterestCorner(2), 0,wgs84Ellipsoid);
p = ned2Traj(alpha)*[pN, pE, 0]';
a = ned2Traj(alpha)*[aN, aE, -3]';
plot3([p(1) a(1)],[p(2) a(2)],[0,0],'-o','Color','green','Displayname','MissionStart');
plot3([a(1) a(1)],[a(2) a(2)],[0, -3],'-o','Color','black','Displayname', 'Submerging');

[t,v,o] = matricesWayPoints(a',11,17,2,4,1,alpha);
[t2,v2,o2] = matricesWayPoints(a',11,17,2,4,-1,alpha);
st = size(t);
sv = size(v);

% centro = t(st(1),:,1)'+ned2Traj(30)'*[0;lineSpaceBetweenTransects/2;0];
% centro = centro';
% plot3(centro(:,1),centro(:,2),centro(:,3),'s','Color','m');


for pa = 1:st(3)%stesso transetto
    for r = 1:st(1)%itero per ogni WP
        vett = t(r,:,pa);%prendo tutte le colonne per avere tutte le componenti
        plot3(vett(:,1),vett(:,2),vett(:,3),'o','Color','b','HandleVisibility','off');
    end
end

for pa = 1:sv(3)
    for r = 1:sv(1)
        vett = v(r,:,pa);
        plot3(vett(:,1),vett(:,2),vett(:,3),'x','Color','c','HandleVisibility','off');
    end
end

for pa = 1:st(3)
    for r = 1:st(1)
        vett = t2(r,:,pa);
        plot3(vett(:,1),vett(:,2)+0.1,vett(:,3),'o','Color','g','HandleVisibility','off');
    end
end

for pa = 1:sv(3)
    for r = 1:sv(1)
        vett = v2(r,:,pa);
        plot3(vett(:,1),vett(:,2),vett(:,3),'x','Color','m','HandleVisibility','off');
    end
end

p0 = t(3,:,3) + [0 1 0];
pf = t(3,:,4);
wpAdd = currentSwingWaypoints(pf,p0,distEmerg);

plot3(p0(:,1),p0(:,2),p0(:,3), '^', 'Color', 'r', 'Markersize', 3,'HandleVisibility','off');
plot3(pf(:,1),pf(:,2),pf(:,3), 'v', 'Color', 'r', 'Markersize', 3,'HandleVisibility','off');

for i = 1:length(wpAdd)
    vett = wpAdd(i,:);
    plot3(vett(:,1),vett(:,2), vett(:,3), ':d','Color','r','MarkerSize',3,'MarkerFaceColor','#D9FFFF','HandleVisibility','off');
end

lastWP = t(st(1),:,st(3));

plot3([lastWP(1) lastWP(1)],[lastWP(2) lastWP(2)],[lastWP(3), 0],'-o','Color','magenta','Displayname','Emerging');
plot3([lastWP(1) p(1)],[lastWP(2) p(2)],[0,0],'-o','Color','#D95319', 'Displayname','MissionEnd');


legend

axis([30 80 30 80 -4 4])