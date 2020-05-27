function humidity = GetSlacMapData(pvupload,doplot)
% humidity = GetSlacMapData
%  returns relative humidity (%) for SLAC main campus location
%  data is 2 hours old and updated every 40 mins
% GetSlacMapData(PV)
%  writes humidity to PV
% GetSlacMapData([],true)
%  Plot map of SLAC location humidity data captured for

ran=0.01;
slaclat = 37.41657 ; % SLAC S30 lat/long
slaclon = -122.204912 ;
latlim = slaclat + [-ran ran] ;
lonlim = slaclon + [-ran ran] ;
numberOfAttempts = 5;
attempt = 0;
info = [];
% Map data server
serverURL = 'http://basemap.nationalmap.gov/ArcGIS/services/USGSImageryOnly/MapServer/WMSServer?';
% Humidity data server
server_humid = 'https://disc1.gsfc.nasa.gov/daac-bin/wms_airsnrt?';
while(isempty(info))
    try
        info = wmsinfo(serverURL);
        info_humid = wmsinfo(server_humid);
        orthoLayer = info.Layer(1);
        humidLayer = info_humid.Layer(12);
    catch e 
        
        attempt = attempt + 1;
        if attempt > numberOfAttempts
            throw(e);
        else
            fprintf('Attempting to connect to server:\n"%s"\n', serverURL)
        end        
    end
end
imageLength = 1024;
[A,R] = wmsread(orthoLayer,'Latlim',latlim, ...
                           'Lonlim',lonlim, ...
                           'ImageHeight',imageLength, ...
                           'ImageWidth',imageLength);
Ah = wmsread(humidLayer,'Latlim',latlim, ...
                           'Lonlim',lonlim);                         
if exist('doplot','var') && doplot
  figure
  geoshow(A,R);
end
humidity=mean(Ah(:));
fprintf('Relative Humidity= %g\n',humidity);
if exist('pvupload','var')
  lcaPut(pvupload,humidity);
end
