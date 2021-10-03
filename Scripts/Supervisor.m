%% Internal parameters of the Mission Supervisor block
disp('Loading Supervisor parameters...');

%% Defining greater and lesser side, and subsequent evaluations
if(firstSideLength >= secondSideLength)
    greaterSide = firstSideLength;
    lesserSide = secondSideLength;
    angle = alpha; 
    rot = 1; %First turn CCW respect the Down axis
else
    greaterSide = secondSideLength;
    lesserSide = firstSideLength;
    angle = alpha + 90;
    rot = -1; %First turn CW respect the Down axis
end

%% Main settings
%Transects reference orientations
orPari = wrapToPi([0 0 deg2rad(angle+180)]);
orDispari = wrapToPi([0 0 deg2rad(angle)]);

%Point P
[pN, pE, pDepth] = geodetic2ned(initPoint(1), initPoint(2), initPoint(3),areaOfInterestCorner(1), areaOfInterestCorner(2), 0,wgs84Ellipsoid);
pointP = [pN, pE, 0.15];

%Point A: on the surface as reference, underwater as scan starting point
[aN, aE, ~] = geodetic2ned(surveyAreaCorner(1), surveyAreaCorner(2), 0, areaOfInterestCorner(1), areaOfInterestCorner(2), 0,wgs84Ellipsoid);
pointA = [aN, aE, 0.15];
orientA = [0, 0, wrapToPi(deg2rad(angle))];

%Distance between standard waypoint on the same transect; [m]
wpOffset = 2.5;

%Matrices containing position of transect waypoints and
%position/orientation of turn waypoints
[trWP, turnWP, turnRef] = matricesWayPoints([aN, aE, averageDepth - altitude], greaterSide, lesserSide, wpOffset, lineSpaceBetweenTransects,rot,angle,0.2);
nWPTran = size(trWP,1); %Number of waypoints for each transect
nWPTurn = size(turnWP,1); %Number of waypoints for each turn

%Number of transects to be travelled
nTran = size(trWP,3);

%Indices to be used in the above matrices
page = 1; %Transect/turn to be travelled next
wpTR = 1; %Current reference waypoint onto the transect
wpTU = 1; %Current reference waypoint onto the turn circumference

%Reference orientation angle to go from P to A
paNED = pointA-pointP;
y = deg2rad(angle);%Starting orientation of the vehicle
rotz = [[cos(y) sin(y) 0];
            [-sin(y) cos(y) 0];
            [0 0 1]];
paNED = rotz*paNED';
xb = [1;0;0];
val = xb'*paNED/(norm(xb)*norm(paNED));%Cosine of the narrower angle between surge axis and PA vector
if paNED(2)>=0
    orDes = wrapToPi(y+acos(val));
else
    orDes = wrapToPi(y-acos(val));
end

%Reference orientation angle to return in P after a successful mission
puntoFin = trWP(nWPTran,:,size(trWP,3));
paNEDrit = pointP - [puntoFin(1),puntoFin(2),0.15];
if mod(size(trWP,3),2) == 0
    yrit = deg2rad(angle+180);
else
    yrit = deg2rad(angle);
end
rotzrit = [[cos(yrit) sin(yrit) 0];
            [-sin(yrit) cos(yrit) 0];
            [0 0 1]];
paNEDrit = rotz*paNEDrit';
xb = [1;0;0];
valrit = xb'*paNEDrit/(norm(xb)*norm(paNEDrit));
if mod(nTran,2)~=0
    if paNEDrit(2)>=0
        orDesRit = wrapToPi(yrit+acos(valrit));%+pi?
    else
        orDesRit = wrapToPi(yrit-acos(valrit));
    end
else
    if paNEDrit(2)>=0
        orDesRit = wrapToPi(yrit+acos(valrit)+pi);%+pi?
    else
        orDesRit = wrapToPi(yrit-acos(valrit)+pi);
    end
end

%% Parameters used in the state machine
%Current state flag vector
current_state = [0 0 0 0 0 0 0 0];

%positioning
velPos = [cruiseSpeed 0 0 0 0 0];%[0.5 0 0 0 0 0];

%diving
endDiv = 0.4 ; %Last meters of the descent; [m]
velSub = [0 0 cruiseSpeed*2/5 0 0 0]; %[m/s m/s m/s rad/s rad/s rad/s]
velRidSub = [0 0 0 0 0 0]; %[m/s m/s m/s rad/s rad/s rad/s]

%surfacing
velSurf = [-velSub(1:3) velSub(4:6)]; %[m/s m/s m/s rad/s rad/s rad/s]

%transect
velTran = [cruiseSpeed 0 0 0 0 0]; %Initialized as the surge cruise speed;[m/s m/s m/s rad/s rad/s rad/s]
velRid =  [0.2 0 0 0 0 0];%[cruiseSpeed*2/5 0 0 0 0 0]; % Lowered velocity for the last meters of each transect;[m/s m/s m/s rad/s rad/s rad/s]

%Emergence waypoints settings
wpEmerg = [];
vEmergTran = [cruiseSpeed*2/5 cruiseSpeed*2/5 cruiseSpeed*2/5 0 0 0]; %[m/s m/s m/s rad/s rad/s rad/s]
idxEmerg = 1;
distEmerg = 0.5; %Lateral distance between waypoints to return onto the trajectory; [m]
offset = 2; %Maximum allowed distance from the transect before realignment; [m]
offsetLast = 1; %Maximum allowed distance from the transect in its last meters before realignment; [m]

%turn
velTurnEven = [cruiseSpeed*2/5 cruiseSpeed*2/5 0 0 0 -rot*(cruiseSpeed*2/5)/(lineSpaceBetweenTransects/2)]; %[m/s m/s m/s rad/s rad/s rad/s]
velTurnOdd = [cruiseSpeed*2/5 cruiseSpeed*2/5 0 0 0 rot*(cruiseSpeed*2/5)/(lineSpaceBetweenTransects/2)]; %[m/s m/s m/s rad/s rad/s rad/s]

%% Tolerances
errSphere = [1.5 1.5 1.5];

%positioning
errPosPos = [1 1 1]; %[m m m]
errOrPos = [0.785398 0.785398 0.0872665]; %[rad rad rad], [45° 45° 5°]
errVelPos = [0.05 0.05 0.05 0.0175 0.0175 0.0175]; %[m/s m/s m/s rad/s rad/s rad/s], rotations of 1°/s

%durante diving
if lineSpaceBetweenTransects<4 && cruiseSpeed>=1.5
    errPosDiv = [1.5 1.5 1.5]; %[m m m];
else
    errPosDiv = [1 1 1];
end
errOrDiv = [0.3491 0.3491 0.0873]; %[rad rad rad], [20° 20° 5°]
errVelDiv = [0.05 0.05 0.05 0.0175 0.0175 0.0175]; %[m/s m/s m/s rad/s rad/s rad/s], rotations of 1°/s

%transetti
errPosTrans = [1.5 1.5 1.5]; %[m m m]
errPosTransLast = [0.5 0.5 0.5]; %[m m m]
errOrTrans = [0.261799 0.261799 0.174533]; % [rad rad rad], [15° 15° 10°]
errVelTrans =  [0.1 0.1 0.1 0.0175 0.0175 0.0175]; %[m/s m/s m/s rad/s rad/s rad/s], rotations of 1°/s

%virate
errPosTurn = [1 1 1]; %[m m m]
errOrTurn = [0.261799 0.261799 0.174533];% [rad rad rad], [15° 15° 10°]
errVelTurn = [0.1 0.1 0.1 0.0175 0.0175 0.0175]; %[m/s m/s m/s rad/s rad/s rad/s], rotations of 1°/s

%surfacing
errPosSurf = errPosDiv; %[m m m]
errOrSurf = errOrDiv; %[rad rad rad]
errVelSurf = [0.2 0.2 0.2 0.3491 0.3491 0.3491]; %[m/s m/s m/s rad/s rad/s rad/s], rotations of 20°/s

%Emergence situations
errPosWPTrans = [0.5 0.2 0.2]; %[m m m]
errOrWPTrans = [0.261799 0.261799 0.174533];% [rad rad rad], [15° 15° 5°]
errVelWPTrans = [0.15 0.05 0.05 0.0175 0.0175 0.0175]; %[m/s m/s m/s rad/s rad/s rad/s], rotations of 1°/s

%% Obstacle detection parameters
%Based on data received from the Control block

tSalto = 8; %[s]
profGap = 4; %[m]

minGap = 1; %Minimum gap to change reference depth; [m]
minDist = 2; %Minimum length dimension of the obstacle not to be ignored; [m]

%Longitudinal distance needed for the vehicle to achieve a 4m depth gap;
%doubled for safety reasons (to consider current, fluidodynamics,
%errors...)
bound = cruiseSpeed*tSalto;
distNeeded = bound*2;

maxRatio = profGap / distNeeded;

%Vehicle semiaxis measures, received from the Model block
a = 1.10; %[m]
b = 0.15; %[m] 
c = 0.15; %[m]

semix = a;
semiy = 0;
semiz = 0;

%Additional safety margin
safeMargin = 0.5;

%Orientation angle at which the frontal FLS is installed
theta_f = 45;

%% End of parameter settings
disp('	Supervisor loaded.');