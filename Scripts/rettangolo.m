close all
clear all
clc

northSideLength= 200;
eastSideLength = 150; 

rectangle('Position',[0 0 eastSideLength northSideLength])
axis([0 250 0 250]) %genero l'area di interesse  rettangolare
lat0 = 42.317744;%areaOfInterestCorner(punto O)
lon0 = 10.915136;
h0 = 0;
lat = 42.318165; %surveyAreaCorner(punto A)
lon = 10.915941;
h = 0;
[na, ea, depth] = geodetic2ned(lat, lon, h,lat0,lon0,h0,wgs84Ellipsoid);
hold on 
plot(ea,na,'ro')
xlabel('East')
ylabel(' North')
latp = 42.317984;%initPoint (punto P)
lonp = 10.916219;
hp = 0;
[np, ep, depthp] = geodetic2ned(latp, lonp, hp,lat0,lon0,h0,wgs84Ellipsoid);
hold on
plot(ep,np,'bo')

firstSideLength = 45;               % [m]
alpha = 30;                         % [deg]
secondSideLength = 25;              % [m]

e2 = ea+firstSideLength*sin(alpha*2*pi/360);%vertice verde in alto
n2 = na+firstSideLength*cos(alpha*2*pi/360);
plot(e2,n2,'gd')
e3 = ea + secondSideLength*cos(alpha*2*pi/360); %vertice viola
n3 = na - secondSideLength*sin(alpha*2*pi/360);
plot (e3,n3,'ms')
e4 = e2 + secondSideLength*cos(alpha*2*pi/360);%vertice verde acqua
n4 = n2 - secondSideLength*sin(alpha*2*pi/360);
plot(e4,n4,'cs')
x = [ea e2  e4 e3 ea];
y =[na n2  n4 n3 na];
line (x,y)
