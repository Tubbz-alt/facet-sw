% Script to take data whilst laser cleaning process is taking place

context = PV.Initialize(PVtype.EPICS) ;
%       context = PV.Initialize(PVtype.EPICS_labca) ;
%       cname = "PROF:IN10:241:" ;
cname = "CAMR:LT10:900:" ;
pvlist=[PV(context,'name',"CCD_img",'pvname',cname+"Image:ArrayData"); % CCD camera image
  PV(context,'name',"CCD_spotsize_x",'pvname',"SIOC:SYS1:ML00:AO348"); % x Laser spot size (um FWHM)
  PV(context,'name',"CCD_spotsize_y",'pvname',"SIOC:SYS1:ML00:AO349"); % y Laser spot size (um FWHM)
  PV(context,'name',"CCD_xpos",'pvname',"SIOC:SYS1:ML00:AO358"); % Output of locally calculated X laser spot position [mm]
  PV(context,'name',"CCD_ypos",'pvname',"SIOC:SYS1:ML00:AO359"); % Output of locally calculated Y laser spot position [mm]
  PV(context,'name',"laser_shutterStatIn",'pvname',"SHUT:LT10:950:IN_MPS.RVAL",'monitor',true); % Laser MPS shutter IN status
  PV(context,'name',"laser_energy",'pvname',"PMTR:LT10:930:QUERYDATA",'conv',1e6); % Laser energy readout (uJ)
  PV(context,'name',"CCD_intensity",'pvname',"SIOC:SYS1:ML00:AO360")]; % intensity of laser spot on CCD
pset(pvlist,'debug',0) ;

pvs = struct(pvlist) ;

rate=5; % approx data taking rate [Hz]
dtime=90; % max time in mins to be taking data
ndat=dtime*rate*60;
xv=nan(1,ndat); yv=xv; ss_x=xv; ss_y=xv; int=xv; shut=xv; lener=xv;
img=cell(1,ndat);
ind=1;
while ind<=ndat
  t0=tic;
  xv(ind)=caget(pvs.CCD_xpos);
  yv(ind)=caget(pvs.CCD_ypos);
  ss_x(ind)=caget(pvs.CCD_spotsize_x);
  ss_y(ind)=caget(pvs.CCD_spotsize_y);
  int(ind)=caget(pvs.CCD_intensity);
  shut(ind)=caget(pvs.laser_shutterStatIn);
  lener(ind)=caget(pvs.laser_energy);
  img{ind}=uint16(caget(pvs.CCD_img));
  ind=ind+1;
  t1=toc(t0);
  if t1<(1/rate)
    pause((1/rate)-t1);
  end
end
