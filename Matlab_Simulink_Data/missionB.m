disp('Loading Mission parameters...')

%% The pylon is a cylinder defined through its center and the radius (this will not be modified)

pylonCenter = [ 43.723046;
                10.396635];         % Lat/Lon [decimal degrees]
         
pylonRadius = 10;                   % [m]

areaDepth = 40;                     % [m]

%% Mission Parameters

initPoint = [ 43.722984;
              10.396204;
               0       ];           % Lat/Lon [decimal degrees], depth [m]
           
initInspectionDirection = 30;       % [deg]
                
initDepth  = 35;                    % [m]
finalDepth = 10;                    % [m]

inspectionDirection = 'vertical';   % it can be 'horizontal'

horizontalTranseptsDistance = 1;    % [m] used when inspectionDirection is 'horizontal'
verticalTranseptsDistance = 20;     % [deg] used when inspectionDirection is 'vertical'

speed = 0.3;                        % [m/s]

disp('Done')