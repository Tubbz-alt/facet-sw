classdef F2_CathodeServicesApp < handle
  %F2_CATHODESERVICES Support functions for F2_CathodeServices.mlapp
  
  events
    PVUpdated % PV object list notifies this event after each set of monitored PVs have finished updating
  end
  properties(Dependent)
    CleaningNumLines % Number of lines to clean in defined cleaning region (vector of length CleaningNumCols if moving in x direction)
    CleaningNumCols % Number of columns per line to clean (vector of length CleaningNumLines if moving in y direction)
    CleaningTimeRemaining % Remaining cleaning time [min]
    MapNumLines % Number of lines to map in defined mapping region (vector of length MapNumCols if moving in x direction)
    MapNumCols % Number of columns per line to map (vector of length MapNumLines if moving in y direction)
    MapTimeRemaining % Remaining QE mapping time [min]
    LaserFluence % Laser fluence [uJ/cm^2] (FWHM)
    MotorVelo % Required motor velocity based on current rep rate, CleaningStepSize and CleaningNumPulsesPerStep [mm/s]
    AcclDist % Distance required to move before full velocity achieved
  end
  properties(SetObservable)
    State CathodeServicesState = CathodeServicesState.Unknown
    buflen uint16 {mustBeGreaterThan(buflen,9)} = 500 % Data buffer length
  end
  properties
    MapCenter(1,2) single = [0,0] % center of QE mapping region (also motor "home" position when QE Map tab selected)
    MapRadius {mustBeLessThan(MapRadius,5e-3),mustBePositive} = 1.5e-3 % Radius on cathode to map [m]
    MapNumPulsesPerStep uint8 {mustBeLessThan(MapNumPulsesPerStep,100),mustBePositive}= 3 % Number of pulses between mapping step positions
    MapStepSize {mustBeLessThan(MapStepSize,300e-6),mustBePositive}= 80e-6 % Step size to use during QE mapping [m]
    MapStartPosition uint8 {mustBeMember(MapStartPosition,[1,2,3,4])} = 1 % Start position (1=bottom, 2=left, 3=top, 4=right)
    MapLineNum uint16 = 1 % Current line number for mapping process
    MapColNum uint16 = 1 % Current column number for mapping process
    CleaningCenter(1,2) single = [0,0] % center of cleaning area (also motor "home" position) [mm]
    CleaningRadius {mustBeLessThan(CleaningRadius,5e-3),mustBePositive} = 1.5e-3 % Radius on cathode to clean [m]
    CleaningNumPulsesPerStep uint8 {mustBeLessThan(CleaningNumPulsesPerStep,100),mustBePositive}= 3 % Number of pulses between cleaning step positions
    CleaningStepSize {mustBeLessThan(CleaningStepSize,300e-6),mustBePositive}= 80e-6 % Step size to use during laser cleaning [m]
    CheckRepVals logical = false % Perform check for past 10 PV values being the same in the watchdog timer?
    debug uint8 {mustBeMember(debug,[0,1,2])} = 2 % 0=live, 1=read only, 2=no connection
    gui; % ID for GUI (GUI launched by constructor)
    imudrate single {mustBePositive} = 30 % max image update rate [Hz]
    bufacq logical = false % Enable buffering of data 
    MotorVeloHome single = 2 % Home motor velocity [mm/s]
    CleaningStartPosition uint8 {mustBeMember(CleaningStartPosition,[1,2,3,4])} = 1 % Start position (1=bottom, 2=left, 3=top, 4=right)
    CleaningLineNum uint16 = 1 % Current line number for cleaning process
    CleaningColNum uint16 = 1 % Current column number for cleaning process
    imrot uint8 {mustBeMember(imrot,[0,1,2,3])} = 0 % Apply image rotation (in multiples of 90 degrees)
    ImageCIntMax single {mustBePositive} = 10 % Abort if any pixel integrates to this times ImageIntensityRange(2)
    dnCMD logical = false % force drawnow command in main watchdog loop
  end
  properties(SetAccess=private)
    VCC_mirrcal(1,4) single = [0,0,1,1] % Calibration mirror position to image position on VCC image [xpos,ypos,xscale,yscale] [mm]
    pvlist PV % Array of PV objects associated with this app
    pvs % Structure of PV arrays with PV names as structure field names
    RepRate uint8 = 30 % Laser firing rep. period [Hz]
    STDOUT=1; % standard output destination
    STDERR=2; % standard error output destination
    ImageSource string = "VCC" % Source for displaying to axis ("VCC" or "REF")
    CCD_stream logical % streaming of CCD images?
    CCD_scale single {mustBePositive} = 9.9e-6 % m / pixel
    ImageIntensityRange(1,2) single {mustBeNonnegative} = [10 150] % Allowable range for image intensity measurements, set using SetLimits method
    LaserSpotSizeRange(1,2) single {mustBeNonnegative} = [100 2000] % Allowable range for calculated laser spot size on CCD [um FWHM], set using SetLimits method
    LaserFluenceRange(1,2) single {mustBeNonnegative} = [0 20] % Allowable range for caculated laser fluence uJ/cm^2 [FWHM], set using SetLimits method
    GunVacuumRange(1,2) single {mustBeNonnegative} = [0.0001 10] % Allowable range for gun vacuum level [nTorr], set using SetLimits method
    LaserEnergyRange(1,2) single {mustBeNonnegative} = [0 150] % Allowable range for laser energy readbacks [uJ], set using SetLimits method
    GunVacuum single % Last read back gun vacuum level [Torr]
    ImageIntensity single % Last calculated image intensity from selected screen
    LaserEnergy single % Last read back laser energy from Joulemeter [uJ]
    LaserSpotSize single % Last calculated laser spot size on CCD [um FWHM]
    LaserPosition_img(1,2) single % Last recorded laser position on screen [mm]
    LaserPosition_mot(1,2) single % Last indicted laser position based on mirror motors [mm]
    LaserPosition_tol single = 0.05 % Tolerance for laser being where it is supposed to be [mm], set using SetLimits method
    ScreenPosition(1,2) single % Indicated screen position [mm]
    LaserMotorStopped logical = true % Is motor in stopped state?
    wtimerKA % Watchdog keepalive timer
    poshistory single = nan(4,500,'single') % Buffered data for image position on camera
    ShutterCloseOveride logical = false % Force shutter to stay closed even when moving in auto pattern
  end
  properties(SetAccess=private,Hidden)
    listeners
    stateListener
    moverListener
    buflenListener
    bufpos uint16 = 1
    pimg % integrated image filled during cleaning cycle
    qint_f % integrated faraday cup charge data
    qint_t % integrated torroid data
    lint % integrated laser energy data
    initvelo(1,2) = [4,4] % initial motor velocities in PVs at application startup (restored on shutdown) [mm/s]
    wd_running logical = false % true when watchdog method running
    wd_time(1,50) = nan(1,50)
    wd_time_noncleaning(1,50) = nan(1,50) ;
    wd_time_ind uint8 = 1
    wd_time_ind_noncleaning uint8 = 1
  end
  properties(Dependent,Hidden)
    wd_freq
    wd_freqerr
  end
  properties(Hidden)
    imupdate logical = false % force image update if true (set ==2 after loading an image into pimg property to display)
    CalibHan matlab.graphics.axis.Axes % handle to diagnostics window
  end
  properties(Constant,Hidden)
    motrbktol=0.01; % motor readback tolerance [mm] : regard and readings within this tolerance band as equivalent
    logfile='logs/F2_CathodeServicesApp'; % root filename to use for log
    camNames=["CAMR:LT10:900" "CTHD:IN10:111"]; % VCC,REF camera names
    version=1.0; % software version
    configprops=["CleaningCenter" "CleaningRadius" "CleaningNumPulsesPerStep" "CleaningStepSize" "CleaningStartPosition" "VCC_mirrcal" "ImageIntensityRange" "LaserSpotSizeRange" ...
      "LaserFluenceRange" "GunVacuumRange" "LaserEnergyRange" "version" "LaserPosition_tol" "MotorVeloHome" "buflen" "imrot" ...
      "MapCenter" "MapRadius" "MapNumPulsesPerStep" "MapStepSize" "MapStartPosition"]; % Properties to save/restore to/from configuration files
  end
  
  methods
    function obj=F2_CathodeServicesApp(debuglevel,guihan)
      % CS = F2_CathodeServices(debuglevel)
      if ~exist('debuglevel','var')
        error('Must provide debug level');
      end
     
      % labca setup and formation of PV list
%       lcaSetSeverityWarnLevel(14) ;
%       lcaSetSeverityWarnLevel(4) ;
      context = PV.Initialize(PVtype.EPICS) ;
      obj.pvlist=[PV(context,'name',"gun_rfstate",'pvname',"KLYS:LI10:21:MOD:HVON_STATE",'monitor',true,'pvlogic','~'); % Gun rf on/off (10-2 modulator state)
        PV(context,'name',"CCD_img",'pvname',"CAMR:LT10:900:Image:ArrayData"); % CCD camera image
        PV(context,'name',"CCD_counter",'pvname',"CAMR:LT10:900:ArrayCounter_RBV",'monitor',true); % Image acquisition counter
        PV(context,'name',"CCD_gain",'pvname',"CAMR:LT10:900:Gain",'monitor',true); % CCD camera image gain factor
        PV(context,'name',"CCD_datatype",'pvname',"CAMR:LT10:900:DataType"); % CCD camera data type
        PV(context,'name',"CCD_nx",'pvname',"CAMR:LT10:900:ArraySizeX_RBV",'monitor',true); % # x-axis data points in CCD image
        PV(context,'name',"CCD_ny",'pvname',"CAMR:LT10:900:ArraySizeY_RBV",'monitor',true); % # y-axis data points in CCD image
        PV(context,'name',"CCD_xpos",'pvname',"CAMR:LT10:900:Stats:Xpos_RBV",'monitor',true,'conv',0.001); % xpos on CCD [mm]
        PV(context,'name',"CCD_ypos",'pvname',"CAMR:LT10:900:Stats:Ypos_RBV",'monitor',true,'conv',0.001); % ypos on CCD [mm]
        PV(context,'name',"CCD_acq",'pvname',"CAMR:LT10:900:Acquire.RVAL",'monitor',true); % acquiring or not (readback)
        PV(context,'name',"CCD_acqset",'pvname',"CAMR:LT10:900:Acquire",'pvdatatype',java.lang.Integer(0).getClass); % acquiring or not (set)
        PV(context,'name',"CCD_acqmode",'pvname',"CAMR:LT10:900:ImageMode_RBV.RVAL",'monitor',true); % image mode (0=single, 2=continuous)
        PV(context,'name',"CCD_acqsetmode",'pvname',"CAMR:LT10:900:ImageMode",'pvdatatype',java.lang.Integer(0).getClass); % PV to use to set acquisition mode
        PV(context,'name','CCD_x1','pvname',"CAMR:LT10:900:ROI:MinX_mm_RBV",'monitor',true); % x1 [mm]
        PV(context,'name','CCD_y1','pvname',"CAMR:LT10:900:ROI:MinY_mm_RBV",'monitor',true); % y1 [mm]
        PV(context,'name','CCD_x2','pvname',"CAMR:LT10:900:ROI:MaxX_mm_RBV",'monitor',true); % x(end) [mm]
        PV(context,'name','CCD_y2','pvname',"CAMR:LT10:900:ROI:MaxY_mm_RBV",'monitor',true); % y(end) [mm]
        PV(context,'name',"CCD_spotsize",'pvname',["CAMR:LT10:900:Stats:SigmaX_mm_RBV","CAMR:LT10:900:Stats:SigmaY_mm_RBV"],'monitor',true,'pvlogic',"MAX",'conv',2*sqrt(2*log(2))); % Laser spot size (um FWHM)
        PV(context,'name',"CCD_intensity",'pvname',"CAMR:LT10:900:Stats:MaxValue_RBV",'monitor',true); % intensity of laser spot on CCD
        PV(context,'name',"laser_shutterCtrl",'pvname',"IOC:SYS1:MP01:MSHUTCTL",'monitor',true); % Laser MPS shutter control
        PV(context,'name',"laser_shutterStatIn",'pvname',"SHUT:LT10:950:IN_MPS.RVAL",'monitor',true); % Laser MPS shutter IN status
        PV(context,'name',"laser_shutterStatOut",'pvname',"SHUT:LT10:950:OUT_MPS.RVAL",'monitor',true); % Laser MPS shutter OUT status
        PV(context,'name',"watchdog_keepalive",'pvname',"IN10_CATHODESUPPORT:laserShutterOp",'pvdatatype',java.lang.Integer(0).getClass); % EPICS CathodeServices watchdog keepalive PV
        PV(context,'name',"watchdog_keepaliveval",'pvname',"IN10_CATHODESUPPORT:laserShutterOp.RVAL"); % EPICS CathodeServices watchdog keepalive PV value, should be 0 on application start otherwise another app running
        PV(context,'name',"watchdog_isalive",'pvname',"IN10_CATHODESUPPORT:HEARTBEAT",'monitor',true); % Counter which increments on each dbget
        PV(context,'name',"fcup_stat",'pvname',"FARC:IN10:241:PNEUMATIC",'monitor',true); % Faraday cup in/out status
        PV(context,'name',"fcup_val",'pvname',"FARC:IN10:241:VAL",'monitor',true); % Faraday cup reading
        PV(context,'name',"torr_val",'pvname',"TORR:IN10:1:VAL",'monitor',true); % Torroid charge reading
        PV(context,'name',"lsr_posx",'pvname',"MIRR:LT10:770:M2_MOTR_H.RBV",'monitor',true,'conv',obj.VCC_mirrcal([3 1])); % X Position readback for laser based on motors [mm]
        PV(context,'name',"lsr_posy",'pvname',"MIRR:LT10:770:M2_MOTR_V.RBV",'monitor',true,'conv',obj.VCC_mirrcal([4 2])); % Y Position readback for laser based on motors [mm]
        PV(context,'name',"lsr_xmov",'pvname',"MIRR:LT10:770:M2_MOTR_H",'monitor',true,'mode','rw'); % X position move command for laser mirror [mm]
        PV(context,'name',"lsr_ymov",'pvname',"MIRR:LT10:770:M2_MOTR_V",'monitor',true,'mode','rw'); % Y position move command for laser mirror [mm]
        PV(context,'name',"lsr_stopx",'pvname',"MIRR:LT10:770:M2_MOTR_H.STOP",'mode','rw'); % stop X motion immediately
        PV(context,'name',"lsr_stopy",'pvname',"MIRR:LT10:770:M2_MOTR_V.STOP",'mode','rw'); % stop y motion immediately
        PV(context,'name',"lsr_xvel",'pvname',"MIRR:LT10:770:M2_MOTR_H.VELO",'mode',"rw",'putwait',true,'monitor',true); % X mirror move velocity [mm/s]
        PV(context,'name',"lsr_yvel",'pvname',"MIRR:LT10:770:M2_MOTR_V.VELO",'mode',"rw",'putwait',true,'monitor',true); % X mirror move velocity [mm/s]
        PV(context,'name',"lsr_xaccl",'pvname',"MIRR:LT10:770:M2_MOTR_H.ACCL",'monitor',true); % X mirror move acceleration [mm/s/s]
        PV(context,'name',"lsr_yaccl",'pvname',"MIRR:LT10:770:M2_MOTR_V.ACCL",'monitor',true); % Y mirror move acceleration [mm/s/s]
        PV(context,'name',"lsr_motion",'pvname',["MIRR:LT10:770:M2_MOTR_H.DMOV","MIRR:LT10:770:M2_MOTR_V.DMOV"],'monitor',true,'pvlogic',"~&"); % Motion status for laser based on motors, true if in motion
        PV(context,'name',"laser_energy",'pvname',"LASR:LT10:930:PWR",'monitor',true); % Laser energy readout (uJ)
        PV(context,'name',"gun_vacuum",'pvname',"VGCC:IN10:113:P",'monitor',true,'conv',1e9); % Vacuum pressire for gun [nTorr]
        PV(context,'name',"laser_telescope",'pvname',"LASR:LT10:100:TELE",'monitor',true);
        PV(context,'name',"laser_reprate",'pvname',"CAMR:LT10:900:ArrayRate_RBV",'monitor',true);
        PV(context,'name',"watchdog_gunvaclimitHI",'pvname',"IN10_CATHODESUPPORT:gunVacHi"); % PV used by EPICS watchdog for high gun vacuum limit
        PV(context,'name',"watchdog_laserlimitHI",'pvname',"IN10_CATHODESUPPORT:laserHi");
        PV(context,'name',"watchdog_gunvaclimitLO",'pvname',"IN10_CATHODESUPPORT:gunVacLo"); % PV used by EPICS watchdog for high gun vacuum limit
        PV(context,'name',"watchdog_laserlimitLO",'pvname',"IN10_CATHODESUPPORT:laserLo")]; % PV used by EPICS watchdog for high laser energy limit
      pset(obj.pvlist,'debug',debuglevel) ;
      obj.pvs = struct(obj.pvlist) ;
      
      % if this isn't the main GUI linked object, done at this stage
      if ~exist('guihan','var')
        return
      end
      
      % Setup GUI field links to PVs
      obj.pvs.gun_rfstate.guihan = guihan.ModOFFLamp ;
      obj.pvs.CCD_xpos.guihan = guihan.EditField_11 ;
      obj.pvs.CCD_ypos.guihan = guihan.EditField_12 ;
      obj.pvs.CCD_spotsize.guihan = [guihan.LaserSpotSizeGauge,guihan.EditField_14] ;
      obj.pvs.CCD_intensity.guihan = [guihan.EditField_4,guihan.ImageIntensityGauge] ;
      obj.pvs.laser_shutterCtrl.guihan = guihan.CLOSESwitch ; SetMode(obj.pvs.laser_shutterCtrl,"rw"); obj.pvs.laser_shutterCtrl.putwait=true;
      obj.pvs.laser_shutterStatIn.guihan = guihan.STATUSLamp ;
      obj.pvs.laser_shutterStatOut.guihan = guihan.STATUSLampOPEN ;
      obj.pvs.fcup_stat.guihan = guihan.Lamp ;
      obj.pvs.fcup_val.guihan = guihan.EditField_5 ;
      obj.pvs.torr_val.guihan = guihan.EditField_6 ;
      obj.pvs.lsr_posx.guihan = guihan.EditField_7 ;
      obj.pvs.lsr_posy.guihan = guihan.EditField_8 ;
      obj.pvs.lsr_motion.guihan = guihan.InmotionLamp ;
      obj.pvs.laser_energy.guihan = [guihan.EditField_3,guihan.LaserEnergyGauge] ;
      obj.pvs.gun_vacuum.guihan = [guihan.EditField_2,guihan.GunVacuumGauge] ;
      obj.pvs.laser_telescope.guihan = guihan.SmallSpotEnabledLamp ;
      obj.pvs.laser_reprate.guihan = guihan.EditField_13 ;
      obj.pvs.torr_val.guihan = [guihan.EditField_6,guihan.Gauge] ;
      obj.pvs.fcup_val.guihan = [guihan.Gauge_2,guihan.EditField_5] ;
      obj.gui = guihan ;
      obj.debug = debuglevel ;
      
      % There can only be a singleton instance of this running on the control system at one time
      % - use watchdog PV to detect running of another instance
      if caget(obj.pvs.watchdog_keepaliveval)==1
        fprintf(obj.STDERR,'ANOTHER INSTANCE OF F2_CATHODESERVICES  ALREADY RUNNING, ABORTING STARTUP!');
        waitfor(errordlg('ANOTHER INSTANCE OF F2_CATHODESERVICES  ALREADY RUNNING, ABORTING STARTUP!','Existing App Detected'));
        exit
      end
      
      % Set default limits in PV objects, and GUI objects
      obj.SetLimits("GunVacuumRange",obj.GunVacuumRange);
      obj.SetLimits("ImageIntensityRange",obj.ImageIntensityRange);
      obj.SetLimits("LaserEnergyRange",obj.LaserEnergyRange);
      obj.SetLimits("LaserSpotSizeRange",obj.LaserSpotSizeRange);
      obj.SetLimits("LaserPosition_tol",obj.LaserPosition_tol);
      obj.SetLimits("LaserFluenceRange",obj.LaserFluenceRange);
      obj.gui.StepSizeEditField.Value = double(obj.CleaningStepSize*1e6) ;
      obj.gui.PulsesateachpositionSpinner.Value = double(obj.CleaningNumPulsesPerStep) ;
      obj.gui.CleaningRadiusmmEditField.Value = double(obj.CleaningRadius*1e3) ;
      obj.gui.ccentEdit_x.Value = double(obj.CleaningCenter(1)) ;
      obj.gui.ccentEdit_y.Value = double(obj.CleaningCenter(2)) ;
      obj.gui.StartPositionKnob.Value = num2str(obj.CleaningStartPosition) ;
      obj.initvelo(1) = caget(obj.pvs.lsr_xvel) ;
      obj.initvelo(2) = caget(obj.pvs.lsr_yvel) ;
      obj.gui.HomeVELOmmsEditField.Value = double(obj.MotorVeloHome) ;
      
      % Set initial state
      caget(obj.pvlist);% fetch all values once
      if strcmp(obj.pvs.laser_telescope.val{1},'IN')
        obj.State=CathodeServicesState.Standby_cleaninglasermode;
      else
        obj.State=CathodeServicesState.Standby_opslasermode;
      end
      try
        obj.watchdog(); % force initial update of GUI fields
      catch ME
        fprintf(obj.STDERR,'WARNING: initial GUI update failed: %s\n',ME.message);
      end
      if obj.pvs.CCD_acqmode.val{1}==2 && obj.pvs.CCD_acq.val{1}>0
        guihan.StreamImageButton.Value=1;
      end
      obj.RepRate = caget(obj.pvs.laser_reprate) ;
      if obj.RepRate<1
        obj.RepRate = 1;
      end
      obj.pvlist.pset('timeout',0.01 + double(1/obj.RepRate));
      fprintf('Launch PV updater...\n');
      run(obj.pvlist,false,0.02,obj,'PVUpdated'); % polling time for PVs (set to faster than any expected laser firing rate)
      
      % Set PVs to autoupdate, PVUpdated event gets notified whenever one
      % of monitored PVs gets updated with a new value, which triggers
      % watchdog method
      obj.listeners = addlistener(obj,'PVUpdated',@(~,~) obj.watchdogUD) ; % causes watchdog method to be called when any PVs updated
      obj.stateListener = listener(obj,'State','PostSet',@(~,~) obj.ProcStateChange);
      obj.moverListener = addlistener(obj.pvs.lsr_motion,'PVStateChange',@(~,~) obj.ProcMirrorStateChange) ;
      obj.ProcMirrorStateChange(); % Initialize motion state readback
      obj.SetMirrCal(obj.VCC_mirrcal) ;
      obj.gui.BufferSizeMenu.Text = sprintf('Buffer Length = %d',obj.buflen) ;
      
      % Start logging
%       diary(sprintf('%s_%s.log',obj.logfile,datestr(now,30)));
      
      % Restore previous configuration settings
      fprintf('Restoring configuration settings...\n');
      try
        obj.LoadConfig;
      catch ME
        warning('No saved configuration file found, using program defaults');
        fprintf(obj.STDERR,'%s\n',ME.message);
      end
      disp('Returning from F2_CathodeServicesApp...');
      
    end
    function ClearBuffer(obj)
      obj.poshistory = nan(4,obj.buflen,'single') ;
    end
    function ProcStateChange(obj)
      fprintf('%s: %s\n',datetime,obj.State);
    end
    function ProcMirrorStateChange(obj)
      val=caget(obj.pvs.lsr_motion);
      if val{1}==1 && val{2}==1
        obj.LaserMotorStopped=true;
      else
        obj.LaserMotorStopped=false;
      end
      if isautopattern(obj.State) && ~obj.LaserMotorStopped && ~obj.ShutterCloseOveride
        OpenShutterAndVerify(obj);
        fprintf(obj.STDOUT,'%s: Mirror in motion: Opening Laser Shutter\n',datetime);
      elseif isautopattern(obj.State)
        CloseShutterAndVerify(obj);
        fprintf(obj.STDOUT,'%s: Mirror stopped moving: Closing Laser Shutter\n',datetime);
      end
    end
    function Proc_QEMap(obj,varargin)
      persistent mstate boundary whan xm ym shutoutpos
      %PROC_QEMAP Process next logical QE Mapping step
      %Proc_QEMap() 
      %Proc_QEMap("SetBoundary",[x0 y0 w h]) Set mapping boundary (Matlab rectangle function pos format with curvature=1)
      %Proc_QEMap("Stop")
      
      % Initialize state
      if isempty(mstate)
        mstate=0;
      end
      
      % Process boundary set command
      if nargin>1
        if varargin{1}=="SetBoundary"
          boundary=varargin{2};
          return
        elseif varargin{1}=="Stop"
          mstate=4;
          return
        end
      end
      
      % If laser motion currently in progress, update line/column number depending on movement direction, no further action required after this
      % Also, open shutter when in mapping zone defined by shutoutpos variable set in mstate 2
      if ~obj.LaserMotorStopped && mstate~=4
        if obj.State == CathodeServicesState.QEMap_linescan
          switch obj.MapStartPosition
            case {1,3} % horizontal moves
              xm_new = caget(obj.pvs.lsr_posx) ; % current mirror command position
              if xm_new >= shutoutpos(1) && xm_new <= shutoutpos(2)
                OpenShutterAndVerify(obj);
              else
                CloseShutterAndVerify(obj);
              end
              nstep = ceil( abs(xm_new-xm) / (obj.MapStepSize*1e3) ) ;
              if nstep>obj.MapNumCols(obj.MapLineNum)
                obj.MapColNum = obj.MapNumCols(obj.MapLineNum) ;
              else
                obj.MapColNum = nstep ;
              end
            case {2,4} % vertical moves
              ym_new = caget(obj.pvs.lsr_posy) ; % current mirror command position
              if ym_new >= shutoutpos(1) && ym_new <= shutoutpos(2)
                OpenShutterAndVerify(obj);
              else
                CloseShutterAndVerify(obj);
              end
              nstep = ceil( abs(ym_new-ym) / (obj.MapStepSize*1e3) ) ;
              if nstep>obj.MapNumLines(obj.MapColNum)
                obj.MapLineNum = obj.MapNumLines(obj.MapColNum) ;
              else
                obj.MapLineNum = nstep ;
              end
          end
        end
        return
      elseif ~obj.LaserMotorStopped
        return
      end
      
      switch mstate % take action based on current state
        case 0 % Start process actions
          % Telescope should be inserted (small spot size on cathode)
          if strcmp(obj.pvs.laser_telescope.val{1},'OUT')
            if isempty(whan) || ~ishandle(whan)
              whan=warndlg('Laser Telescope Optics not inserted, cannot start Mapping Sequence','Telescope not inserted');
            end
            return
          end

          % Need to have previously defined map program area
          if obj.State ~= CathodeServicesState.QEMap_definearea
            if isempty(whan) || ~ishandle(whan)
              whan=warndlg('Mapping area not yet defined, or different program in progress, stop any in-use program or use "Define Map Area" button first','Map Area Undefined');
            end
            return
          end
          
          % Boundary vector should be filled
          if isempty(boundary)
            if isempty(whan) || ~ishandle(whan)
              whan=warndlg('No area boundary defined, push "Define Map Area" button first','No map area boundary');
            end
            return
          end
          
          % If previous mapping attempt aborted, offer choice to re-start or start fresh
          if obj.MapColNum>1 || obj.MapLineNum>1
            resp = questdlg('Previous mapping program incomplete, continue previous or start new?','Restart Previous Program?','Continue Previous','Start New','Continue Previous');
            if strcmp(resp,'Start New')
              obj.MapColNum=1; obj.MapLineNum=1;
            end
          end

          % Make sure image streaming enabled and laser shutter initially closed
          obj.gui.StreamImageButton.Value = 1 ;
          obj.gui.StreamImageButton.ValueChangedFcn(obj.gui,obj.gui.StreamImageButton) ;
          if ~obj.CloseShutterAndVerify()
            return
          end
          if obj.MapColNum>1 || obj.MapLineNum>1
            mstate = 3 ;
          else
            mstate = 1 ;
          end
          obj.State = CathodeServicesState.QEMap_movingtonewline ; % watchdog method will repeatedly call this function until aborted
          fprintf(obj.STDOUT,'%s: QE Mapping: start\n',datetime);
        case 1 % move to start position
          obj.ShutterCloseOveride = true ; % don't open shutter until on auto pattern path
          obj.State = CathodeServicesState.QEMap_movingtonewline ;
          adist = obj.AcclDist ; % distance to accelerate to full velocity [mm]
          switch obj.MapStartPosition
            case 1 % bottom
              crdlen=double(obj.MapNumCols(obj.MapLineNum))*obj.MapStepSize*1e3; % length of circle cord at this vertical position
              dest = [boundary(1)+boundary(3)/2-crdlen/2-adist(1) boundary(2)] ;
            case 2 % left
              crdlen=double(obj.MapNumLines(obj.MapColNum))*obj.MapStepSize*1e3; % length of circle cord at this horizontal position
              dest = [boundary(1) boundary(2)+boundary(4)/2-crdlen/2-adist(2)] ;
            case 3 % top
              crdlen=double(obj.MapNumCols(obj.MapLineNum))*obj.MapStepSize*1e3; % length of circle cord at this vertical position
              dest = [boundary(1)+boundary(3)/2-crdlen/2-adist(1) boundary(2)+boundary(4)] ;
            case 4 % right
              crdlen=double(obj.MapNumLines(obj.MapColNum))*obj.MapStepSize*1e3; % length of circle cord at this horizontal position
              dest = [boundary(1)+boundary(3) boundary(2)+boundary(4)/2-crdlen/2-adist(2)] ;
          end
          xlim=[obj.pvs.CCD_x1.val{1} obj.pvs.CCD_x2.val{1}].*1e-3; ylim=[obj.pvs.CCD_y1.val{1} obj.pvs.CCD_y2.val{1}].*1e-3;
          if dest(1)<xlim(1)
            dest(1)=xlim(1);
          elseif dest(1)>xlim(2)
            dest(1)=xlim(2);
          end
          if dest(2)<ylim(1)
            dest(2)=ylim(1);
          elseif dest(2)>ylim(2)
            dest(2)=ylim(2);
          end
          reqmove = dest - obj.LaserPosition_mot ;
          if any(abs(reqmove)>obj.motrbktol)
            obj.movemirror(reqmove,"fast");
          else % done with this move
            mstate = 2 ;
          end
          obj.ClearPIMG; % clear integrated image
          fprintf(obj.STDOUT,'%s: QE Map: move to start position\n',datetime);
        case 2 % Move along next line or column
          obj.ShutterCloseOveride = true ; % Automatically opens shutter when over map area (starts just outside)
          obj.State = CathodeServicesState.QEMap_linescan ;
          xm = caget(obj.pvs.lsr_posx) ;
          ym = caget(obj.pvs.lsr_posy) ; % current mirror command position
          adist = obj.AcclDist ; % distance to accelerate to full velocity [mm]
          switch obj.MapStartPosition
            case {1,3} % horizontal moves
              crdlen=double(obj.MapNumCols(obj.MapLineNum))*obj.MapStepSize*1e3; % length of circle cord at this vertical position
              if xm<(boundary(1)+boundary(3)/2) % move in +ve x direction
                reqmove = [crdlen+2*adist(1) 0] ;
                shutoutpos = [xm+adist(1) xm+crdlen+adist(1)] ; % x coordinates to open laser shutter
              else % move in -ve x direction
                reqmove = [-crdlen-2*adist(1) 0] ;
                shutoutpos = [xm-crdlen-adist(1) xm-adist(1)] ; % x coordinates to open laser shutter
              end
              fprintf(obj.STDOUT,'%s: QE Map: line # %d (of %d)\n',datetime,obj.MapLineNum,obj.MapNumLines);
              if obj.MapLineNum == obj.MapNumLines
                mstate = 4 ; % done
                obj.MapColNum = 1 ; obj.MapLineNum = 1 ;
              else
                obj.MapLineNum = obj.MapLineNum + 1 ;
                mstate = 3 ; % move to next line when move finished
              end
            case {2,4} % vertical moves
              crdlen=double(obj.MapNumLines(obj.MapColNum))*obj.MapStepSize*1e3; % length of circle cord at this horizontal position
              if ym<(boundary(2)+boundary(4)/2) % move in +ve y direction
                reqmove = [0 crdlen+2*adist(2)] ;
                shutoutpos = [ym+adist(2) ym+crdlen+adist(2)] ; % x coordinates to open laser shutter
              else % move in -ve y direction
                reqmove = [0 -crdlen-2*adist(2)] ;
                shutoutpos = [ym-crdlen-adist(2) ym-adist(2)] ; % x coordinates to open laser shutter
              end
              fprintf(obj.STDOUT,'%s: QE Map: column # %d (of %d)\n',datetime,obj.MapColNum,obj.MapNumCols);
              if obj.MapColNum == obj.MapNumCols
                mstate = 4 ; % done
                obj.MapColNum = 1 ; obj.MapLineNum = 1 ;
              else
                obj.MapColNum = obj.MapColNum + 1 ;
                mstate = 3 ; % move to next line when move finished
              end
          end
          obj.movemirror(reqmove);
        case 3 % Move to beginning of next line or column
          obj.State = CathodeServicesState.QEMap_movingtonewline ;
          obj.ShutterCloseOveride = true ;
          xm = caget(obj.pvs.lsr_posx) ;
          ym = caget(obj.pvs.lsr_posy) ; % current mirror command position
          adist = obj.AcclDist ; % distance to accelerate to full velocity [mm]
          switch obj.MapStartPosition
            case 1 % horizontal moves, starting from bottom
              crdlen=double(obj.MapNumCols(obj.MapLineNum))*obj.MapStepSize*1e3; % length of circle cord at this vertical position
              if xm<(boundary(1)+boundary(3)/2) % next move in +ve x direction
                dest = [boundary(1)+boundary(3)/2-crdlen/2-adist(1) boundary(2)+obj.MapStepSize*1e3*double(obj.MapLineNum-1)+(obj.MapStepSize/2)*1e3] ;
              else % next move in -ve x direction
                dest = [boundary(1)+boundary(3)/2+crdlen/2+adist(1) boundary(2)+obj.MapStepSize*1e3*double(obj.MapLineNum-1)+(obj.MapStepSize/2)*1e3] ;
              end
              fprintf(obj.STDOUT,'%s: QE Map: move to line # %d (of %d)\n',datetime,obj.MapLineNum,obj.MapNumLines);
            case 3
              % horizontal moves, starting from top
              crdlen=double(obj.MapNumCols(obj.MapLineNum))*obj.MapStepSize*1e3; % length of circle cord at this vertical position
              if xm<(boundary(1)+boundary(3)/2) % next move in +ve x direction
                dest = [boundary(1)+boundary(3)/2-crdlen/2-adist(1) boundary(2)+boundary(4)-obj.MapStepSize*1e3*double(obj.MapLineNum-1)-(obj.MapStepSize/2)*1e3] ;
              else % next move in -ve x direction
                dest = [boundary(1)+boundary(3)/2+crdlen/2+adist(1) boundary(2)+boundary(4)-obj.MapStepSize*1e3*double(obj.MapLineNum-1)-(obj.MapStepSize/2)*1e3] ;
              end
              fprintf(obj.STDOUT,'%s: QE Map: move to line # %d (of %d)\n',datetime,obj.MapLineNum,obj.MapNumLines);
            case 2 % vertical moves, starting on left
              crdlen=double(obj.MapNumLines(obj.MapColNum))*obj.MapStepSize*1e3; % length of circle cord at this horizontal position
              if ym<(boundary(2)+boundary(4)/2) % next move in +ve y direction
                dest = [boundary(1)+obj.MapStepSize*1e3*double(obj.MapColNum-1)+(obj.MapStepSize/2)*1e3 boundary(2)+boundary(4)/2-crdlen/2-adist(2)] ;
              else % next move in -ve y direction
                dest = [boundary(1)+obj.MapStepSize*1e3*double(obj.MapColNum-1)+(obj.MapStepSize/2)*1e3 boundary(2)+boundary(4)/2+crdlen/2+adist(2)] ;
              end
              fprintf(obj.STDOUT,'%s: QE Map: move to column # %d (of %d)\n',datetime,obj.MapColNum,obj.MapNumCols);
            case 4 % vertical moves, starting on right
              crdlen=double(obj.MapNumLines(obj.MapColNum))*obj.MapStepSize*1e3; % length of circle cord at this horizontal position
              if ym<(boundary(2)+boundary(4)/2) % next move in +ve y direction
                dest = [boundary(1)+boundary(3)-obj.MapStepSize*1e3*double(obj.MapColNum-1)-(obj.MapStepSize/2)*1e3 boundary(2)+boundary(4)/2-crdlen/2-adist(2)] ;
              else % next move in -ve y direction
                dest = [boundary(1)+boundary(3)-obj.MapStepSize*1e3*double(obj.MapColNum-1)-(obj.MapStepSize/2)*1e3 boundary(2)+boundary(4)/2+crdlen/2+adist(2)] ;
              end
              fprintf(obj.STDOUT,'%s: QE Map: move to column # %d (of %d)\n',datetime,obj.MapColNum,obj.MapNumCols);
          end
          xlim=[obj.pvs.CCD_x1.val{1} obj.pvs.CCD_x2.val{1}].*1e-3; ylim=[obj.pvs.CCD_y1.val{1} obj.pvs.CCD_y2.val{1}].*1e-3;
          if dest(1)<xlim(1)
            dest(1)=xlim(1);
          elseif dest(1)>xlim(2)
            dest(1)=xlim(2);
          end
          if dest(2)<ylim(1)
            dest(2)=ylim(1);
          elseif dest(2)>ylim(2)
            dest(2)=ylim(2);
          end
          reqmove = dest - obj.LaserPosition_mot ;
          if any(abs(reqmove)>obj.motrbktol)
            obj.movemirror(reqmove);
          else % done with this move
            mstate = 2 ;
          end
        case 4 % QE Map complete, move back to center
          obj.ShutterCloseOveride = true ;
          CloseShutterAndVerify(obj) ;
          if any( abs( obj.CleaningCenter - obj.LaserPosition_mot ) > obj.motrbktol )
            obj.movemirror("home");
          else % done with this move
            mstate = 0 ;
            obj.State = CathodeServicesState.QEMap_definearea ;
            obj.ShutterCloseOveride = false ;
          end
          fprintf(obj.STDOUT,'%s: QE Map: finishing\n',datetime);
      end
    end
    function Proc_Cleaning(obj,varargin)
      persistent cstate boundary whan xm ym shutoutpos
      %PROC_CLEANING Process next logical cleaning step
      %Proc_Cleaning() 
      %Proc_Cleaning("SetBoundary",[x0 y0 w h]) Set cleaning pattern boundaries (Matlab rectangle function pos format with curvature=1)
      
      % Initialize state
      if isempty(cstate)
        cstate=0;
      end
      
      % Process boundary set command
      if nargin>1
        if varargin{1}=="SetBoundary"
          boundary=varargin{2};
          return
        elseif varargin{1}=="Stop"
          cstate=4;
        end
      end
      
      % If laser motion currently in progress, update line/column number depending on movement direction, no further action required after this
      % Also, open shutter when in cleaning zone defined by shutoutpos variable set in cstate 2
      if ~obj.LaserMotorStopped && cstate~=4
        if obj.State == CathodeServicesState.Cleaning_linescan
          switch obj.CleaningStartPosition
            case {1,3} % horizontal moves
              xm_new = caget(obj.pvs.lsr_posx) ; % current mirror command position
              if xm_new >= shutoutpos(1) && xm_new <= shutoutpos(2)
                OpenShutterAndVerify(obj);
              else
                CloseShutterAndVerify(obj);
              end
              nstep = ceil( abs(xm_new-xm) / (obj.CleaningStepSize*1e3) ) ;
              if nstep>obj.CleaningNumCols(obj.CleaningLineNum)
                obj.CleaningColNum = obj.CleaningNumCols(obj.CleaningLineNum) ;
              else
                obj.CleaningColNum = nstep ;
              end
            case {2,4} % vertical moves
              ym_new = caget(obj.pvs.lsr_posy) ; % current mirror command position
              if ym_new >= shutoutpos(1) && ym_new <= shutoutpos(2)
                OpenShutterAndVerify(obj);
              else
                CloseShutterAndVerify(obj);
              end
              nstep = ceil( abs(ym_new-ym) / (obj.CleaningStepSize*1e3) ) ;
              if nstep>obj.CleaningNumLines(obj.CleaningColNum)
                obj.CleaningLineNum = obj.CleaningNumLines(obj.CleaningColNum) ;
              else
                obj.CleaningLineNum = nstep ;
              end
          end
        end
        return
      elseif ~obj.LaserMotorStopped
        return
      end
      
      switch cstate % take action based on current state
        case 0 % Start process actions
          % Telescope should be inserted (small spot size on cathode)
          if strcmp(obj.pvs.laser_telescope.val{1},'OUT')
            if isempty(whan) || ~ishandle(whan)
              whan=warndlg('Laser Telescope Optics not inserted, cannot start Cleaning Sequence','Telescope not inserted');
            end
            return
          end

          % Need to have previously defined cleaning program area
          if obj.State ~= CathodeServicesState.Cleaning_definearea
            if isempty(whan) || ~ishandle(whan)
              whan=warndlg('Cleaning area not yet defined, or different program in progress, stop any in-use program or use "Define Cleaning Program Areas" button first','Cleaning Area Undefined');
            end
            return
          end
          
          % Boundary vector should be filled
          if isempty(boundary)
            if isempty(whan) || ~ishandle(whan)
              whan=warndlg('No area boundary defined, push "Define cleaning program areas" button first','No cleaning area boundary');
            end
            return
          end
          
          % If previous cleaning attempt aborted, offer choice to re-start or start fresh
          if obj.CleaningColNum>1 || obj.CleaningLineNum>1
            resp = questdlg('Previous Cleaning Program incomplete, continue previous or start new?','Restart Previous Program?','Continue Previous','Start New','Continue Previous');
            if strcmp(resp,'Start New')
              obj.CleaningColNum=1; obj.CleaningLineNum=1;
            end
          end

          % Make sure image streaming enabled and laser shutter initially closed
          obj.gui.StreamImageButton.Value = 1 ;
          obj.gui.StreamImageButton.ValueChangedFcn(obj.gui,obj.gui.StreamImageButton) ;
          if ~obj.CloseShutterAndVerify()
            return
          end
          if obj.CleaningColNum>1 || obj.CleaningLineNum>1
            cstate = 3 ;
          else
            cstate = 1 ;
          end
          obj.State = CathodeServicesState.Cleaning_movingtonewline ; % watchdog method will repeatedly call this function until aborted
          fprintf(obj.STDOUT,'%s: Cleaning: start\n',datetime);
        case 1 % move to start position
          obj.ShutterCloseOveride = true ; % don't open shutter until on auto pattern path
          obj.State = CathodeServicesState.Cleaning_movingtonewline ;
          adist = obj.AcclDist ; % distance to accelerate to full velocity [mm]
          switch obj.CleaningStartPosition
            case 1 % bottom
              crdlen=double(obj.CleaningNumCols(obj.CleaningLineNum))*obj.CleaningStepSize*1e3; % length of circle cord at this vertical position
              dest = [boundary(1)+boundary(3)/2-crdlen/2-adist(1) boundary(2)] ;
            case 2 % left
              crdlen=double(obj.CleaningNumLines(obj.CleaningColNum))*obj.CleaningStepSize*1e3; % length of circle cord at this horizontal position
              dest = [boundary(1) boundary(2)+boundary(4)/2-crdlen/2-adist(2)] ;
            case 3 % top
              crdlen=double(obj.CleaningNumCols(obj.CleaningLineNum))*obj.CleaningStepSize*1e3; % length of circle cord at this vertical position
              dest = [boundary(1)+boundary(3)/2-crdlen/2-adist(1) boundary(2)+boundary(4)] ;
            case 4 % right
              crdlen=double(obj.CleaningNumLines(obj.CleaningColNum))*obj.CleaningStepSize*1e3; % length of circle cord at this horizontal position
              dest = [boundary(1)+boundary(3) boundary(2)+boundary(4)/2-crdlen/2-adist(2)] ;
          end
          xlim=[obj.pvs.CCD_x1.val{1} obj.pvs.CCD_x2.val{1}].*1e-3; ylim=[obj.pvs.CCD_y1.val{1} obj.pvs.CCD_y2.val{1}].*1e-3;
          if dest(1)<xlim(1)
            dest(1)=xlim(1);
          elseif dest(1)>xlim(2)
            dest(1)=xlim(2);
          end
          if dest(2)<ylim(1)
            dest(2)=ylim(1);
          elseif dest(2)>ylim(2)
            dest(2)=ylim(2);
          end
          posnow = [caget(obj.pvs.lsr_posx) caget(obj.pvs.lsr_posy)];
          reqmove = dest - posnow ;
          if any(abs(reqmove)>obj.motrbktol)
            obj.movemirror(reqmove,"fast");
          else % done with this move
            cstate = 2 ;
          end
          obj.ClearPIMG; % clear integrated image
          fprintf(obj.STDOUT,'%s: Cleaning: move to start position\n',datetime);
        case 2 % Move along next line or column
          obj.ShutterCloseOveride = true ; % Automatically opens shutter when over cleaning area (starts just outside)
          obj.State = CathodeServicesState.Cleaning_linescan ;
          xm = caget(obj.pvs.lsr_posx) ;
          ym = caget(obj.pvs.lsr_posy) ; % current mirror command position
          adist = obj.AcclDist ; % distance to accelerate to full velocity [mm]
          switch obj.CleaningStartPosition
            case {1,3} % horizontal moves
              crdlen=double(obj.CleaningNumCols(obj.CleaningLineNum))*obj.CleaningStepSize*1e3; % length of circle cord at this vertical position
              if xm<(boundary(1)+boundary(3)/2) % move in +ve x direction
                reqmove = [crdlen+2*adist(1) 0] ;
                shutoutpos = [xm+adist(1) xm+crdlen+adist(1)] ; % x coordinates to open laser shutter
              else % move in -ve x direction
                reqmove = [-crdlen-2*adist(1) 0] ;
                shutoutpos = [xm-crdlen-adist(1) xm-adist(1)] ; % x coordinates to open laser shutter
              end
              fprintf(obj.STDOUT,'%s: Cleaning: line # %d (of %d)\n',datetime,obj.CleaningLineNum,obj.CleaningNumLines);
              if obj.CleaningLineNum == obj.CleaningNumLines
                cstate = 4 ; % done
                obj.CleaningColNum = 1 ; obj.CleaningLineNum = 1 ;
                if obj.CleaningStartPosition ==4 
                  obj.CleaningStartPosition = 1 ;
                else
                  obj.CleaningStartPosition = obj.CleaningStartPosition + 1 ;
                end
                obj.gui.StartPositionKnob.Value = num2str(obj.CleaningStartPosition) ;
              else
                obj.CleaningLineNum = obj.CleaningLineNum + 1 ;
                cstate = 3 ; % move to next line when move finished
              end
            case {2,4} % vertical moves
              crdlen=double(obj.CleaningNumLines(obj.CleaningColNum))*obj.CleaningStepSize*1e3; % length of circle cord at this horizontal position
              if ym<(boundary(2)+boundary(4)/2) % move in +ve y direction
                reqmove = [0 crdlen+2*adist(2)] ;
                shutoutpos = [ym+adist(2) ym+crdlen+adist(2)] ; % x coordinates to open laser shutter
              else % move in -ve y direction
                reqmove = [0 -crdlen-2*adist(2)] ;
                shutoutpos = [ym-crdlen-adist(2) ym-adist(2)] ; % x coordinates to open laser shutter
              end
              fprintf(obj.STDOUT,'%s: Cleaning: column # %d (of %d)\n',datetime,obj.CleaningColNum,obj.CleaningNumCols);
              if obj.CleaningColNum == obj.CleaningNumCols
                cstate = 4 ; % done
                obj.CleaningColNum = 1 ; obj.CleaningLineNum = 1 ;
                if obj.CleaningStartPosition ==4 
                  obj.CleaningStartPosition = 1 ;
                else
                  obj.CleaningStartPosition = obj.CleaningStartPosition + 1 ;
                end
                obj.gui.StartPositionKnob.Value = num2str(obj.CleaningStartPosition) ;
              else
                obj.CleaningColNum = obj.CleaningColNum + 1 ;
                cstate = 3 ; % move to next line when move finished
              end
          end
%           line(obj.gui.UIAxes,shutoutpos,[ym ym],'LineWidth',2);
          obj.movemirror(reqmove);
        case 3 % Move to beginning of next line or column
          obj.State = CathodeServicesState.Cleaning_movingtonewline ;
          obj.ShutterCloseOveride = true ;
          xm = caget(obj.pvs.lsr_posx) ;
          ym = caget(obj.pvs.lsr_posy) ; % current mirror command position
          adist = obj.AcclDist ; % distance to accelerate to full velocity [mm]
          switch obj.CleaningStartPosition
            case 1 % horizontal moves, starting from bottom
              crdlen=double(obj.CleaningNumCols(obj.CleaningLineNum))*obj.CleaningStepSize*1e3; % length of circle cord at this vertical position
              if xm<(boundary(1)+boundary(3)/2) % next move in +ve x direction
                dest = [boundary(1)+boundary(3)/2-crdlen/2-adist(1) boundary(2)+obj.CleaningStepSize*1e3*double(obj.CleaningLineNum-1)+(obj.CleaningStepSize/2)*1e3] ;
              else % next move in -ve x direction
                dest = [boundary(1)+boundary(3)/2+crdlen/2+adist(1) boundary(2)+obj.CleaningStepSize*1e3*double(obj.CleaningLineNum-1)+(obj.CleaningStepSize/2)*1e3] ;
              end
              fprintf(obj.STDOUT,'%s: Cleaning: move to line # %d (of %d)\n',datetime,obj.CleaningLineNum,obj.CleaningNumLines);
            case 3
              % horizontal moves, starting from top
              crdlen=double(obj.CleaningNumCols(obj.CleaningLineNum))*obj.CleaningStepSize*1e3; % length of circle cord at this vertical position
              if xm<(boundary(1)+boundary(3)/2) % next move in +ve x direction
                dest = [boundary(1)+boundary(3)/2-crdlen/2-adist(1) boundary(2)+boundary(4)-obj.CleaningStepSize*1e3*double(obj.CleaningLineNum-1)-(obj.CleaningStepSize/2)*1e3] ;
              else % next move in -ve x direction
                dest = [boundary(1)+boundary(3)/2+crdlen/2+adist(1) boundary(2)+boundary(4)-obj.CleaningStepSize*1e3*double(obj.CleaningLineNum-1)-(obj.CleaningStepSize/2)*1e3] ;
              end
              fprintf(obj.STDOUT,'%s: Cleaning: move to line # %d (of %d)\n',datetime,obj.CleaningLineNum,obj.CleaningNumLines);
            case 2 % vertical moves, starting on left
              crdlen=double(obj.CleaningNumLines(obj.CleaningColNum))*obj.CleaningStepSize*1e3; % length of circle cord at this horizontal position
              if ym<(boundary(2)+boundary(4)/2) % next move in +ve y direction
                dest = [boundary(1)+obj.CleaningStepSize*1e3*double(obj.CleaningColNum-1)+(obj.CleaningStepSize/2)*1e3 boundary(2)+boundary(4)/2-crdlen/2-adist(2)] ;
              else % next move in -ve y direction
                dest = [boundary(1)+obj.CleaningStepSize*1e3*double(obj.CleaningColNum-1)+(obj.CleaningStepSize/2)*1e3 boundary(2)+boundary(4)/2+crdlen/2+adist(2)] ;
              end
              fprintf(obj.STDOUT,'%s: Cleaning: move to column # %d (of %d)\n',datetime,obj.CleaningColNum,obj.CleaningNumCols);
            case 4 % vertical moves, starting on right
              crdlen=double(obj.CleaningNumLines(obj.CleaningColNum))*obj.CleaningStepSize*1e3; % length of circle cord at this horizontal position
              if ym<(boundary(2)+boundary(4)/2) % next move in +ve y direction
                dest = [boundary(1)+boundary(3)-obj.CleaningStepSize*1e3*double(obj.CleaningColNum-1)-(obj.CleaningStepSize/2)*1e3 boundary(2)+boundary(4)/2-crdlen/2-adist(2)] ;
              else % next move in -ve y direction
                dest = [boundary(1)+boundary(3)-obj.CleaningStepSize*1e3*double(obj.CleaningColNum-1)-(obj.CleaningStepSize/2)*1e3 boundary(2)+boundary(4)/2+crdlen/2+adist(2)] ;
              end
              fprintf(obj.STDOUT,'%s: Cleaning: move to column # %d (of %d)\n',datetime,obj.CleaningColNum,obj.CleaningNumCols);
          end
          xlim=[obj.pvs.CCD_x1.val{1} obj.pvs.CCD_x2.val{1}].*1e-3; ylim=[obj.pvs.CCD_y1.val{1} obj.pvs.CCD_y2.val{1}].*1e-3;
          if dest(1)<xlim(1)
            dest(1)=xlim(1);
          elseif dest(1)>xlim(2)
            dest(1)=xlim(2);
          end
          if dest(2)<ylim(1)
            dest(2)=ylim(1);
          elseif dest(2)>ylim(2)
            dest(2)=ylim(2);
          end
          posnow = [caget(obj.pvs.lsr_posx) caget(obj.pvs.lsr_posy)];
          reqmove = dest - posnow ;
          if any(abs(reqmove)>obj.motrbktol)
            disp(reqmove)
            obj.movemirror(reqmove);
          else % done with this move
            cstate = 2 ;
          end
        case 4 % Cleaning complete, move back to center
          obj.ShutterCloseOveride = true ;
          CloseShutterAndVerify(obj) ;
          if any( abs( obj.CleaningCenter - obj.LaserPosition_mot ) > obj.motrbktol )
            obj.movemirror("home");
          else % done with this move
            cstate = 0 ;
            obj.State = CathodeServicesState.Cleaning_definearea ;
            obj.ShutterCloseOveride = false ;
          end
          fprintf(obj.STDOUT,'%s: Cleaning: finishing\n',datetime);
      end
    end
    function Proc_Cleaning_testpattern(obj,varargin)
      persistent tpstate boundary1 boundary2 whan
      %PROC_CLEANING_TESTPATTERN Process a test calibration autopattern
      %Proc_Cleaning_testpattern(newstate) Process current test pattern state with CathodeServicesState.Cleaning_testpattern or CathodeServicesState.Cleaning_setenergypattern
      %Proc_Cleaning_testpattern("SetTestBoundary",[x0 y0 w h]) Set cleaning pattern boundaries (Matlab rectangle function pos format)
      %Proc_Cleaning_testpattern("SetEnergyBoundary",[x0 y0 w h]) Set cleaning pattern boundaries (Matlab rectangle function pos format)
      
      % Initialize state
      if isempty(tpstate)
        tpstate=0;
      end
      
      % Process boundary set command
      if nargin>1
        if varargin{1}=="SetTestBoundary"
          boundary1=varargin{2};
          return
        elseif varargin{1}=="SetEnergyBoundary"
          boundary2=varargin{2};
          return
        elseif varargin{1}=="Stop"
          tpstate=8;
        else
          newstate = varargin{1} ;
        end
      end
      
      switch obj.State
        case CathodeServicesState.Cleaning_testpattern
          boundary=boundary1;
        case CathodeServicesState.Cleaning_setenergypattern
          boundary=boundary2;
      end
      
      % If laser motion currently in progress, no further action required, otherwise shutter should be closed
      if ~obj.LaserMotorStopped
        return
      end
      
      switch tpstate % take action based on current state
        case 0 % Start process actions
          % Telescope should be inserted (small spot size on cathode)
          if strcmp(obj.pvs.laser_telescope.val{1},'OUT')
            if isempty(whan) || ~ishandle(whan)
              whan=warndlg('Laser Telescope Optics not inserted, cannot start Test Cleaning Sequence','Telescope not inserted');
            end
            return
          end

          % Need to have previously defined cleaning program area
          if obj.State ~= CathodeServicesState.Cleaning_definearea
            if isempty(whan) || ~ishandle(whan)
              whan=warndlg('Cleaning area not yet defined, or different program in progress, stop any in-use program or use "Define Cleaning Program Areas" button first','Cleaning Area Undefined');
            end
            return
          end
          
          % Boundary vector should be filled
          if isempty(boundary1)
            if isempty(whan) || ~ishandle(whan)
              whan=warndlg('No area boundary defined, push "Define cleaning program areas" button first','No Test area boundary');
            end
            return
          end

          % Make sure image streaming enabled and laser shutter initially closed
          obj.gui.StreamImageButton.Value = 1 ;
          obj.gui.StreamImageButton.ValueChangedFcn(obj.gui,obj.gui.StreamImageButton) ;
          if ~obj.CloseShutterAndVerify()
            return
          end
          tpstate = 1 ;
          obj.State = newstate ; % watchdog method will repeatedly call this function until aborted
          fprintf(obj.STDOUT,'%s: Cleaning_testpattern: start\n',datetime);
        case 1 % move to right edge of test pattern square from center
          obj.ShutterCloseOveride = true ; % don't open shutter until on auto pattern path
          dest = [boundary(1)+boundary(3) boundary(2)+boundary(4)/2] ;
          reqmove = dest - obj.LaserPosition_mot ;
          if any(abs(reqmove)>obj.motrbktol)
            obj.movemirror(reqmove,"fast");
          else % done with this move
            tpstate = 2 ;
          end
          obj.ClearPIMG();
          fprintf(obj.STDOUT,'%s: Cleaning_testpattern: step 1\n',datetime);
        case 2 % move to top-right of test pattern square
          obj.ShutterCloseOveride = false ; % ok to open shutter when moving now
          dest = [boundary(1)+boundary(3) boundary(2)+boundary(4)] ;
          reqmove = dest - obj.LaserPosition_mot ;
          if any(abs(reqmove)>obj.motrbktol)
            obj.movemirror(reqmove);
          else % done with this move
            tpstate = 3 ;
          end
          obj.ClearPIMG();
          fprintf(obj.STDOUT,'%s: Cleaning_testpattern: step 2\n',datetime);
        case 3 % move to top-left of test pattern square
          obj.ShutterCloseOveride = false ; % ok to open shutter when moving now
          dest = [boundary(1) boundary(2)+boundary(4)] ;
          reqmove = dest - obj.LaserPosition_mot ;
          if any(abs(reqmove)>obj.motrbktol)
            obj.movemirror(reqmove);
          else % done with this move
            tpstate = 4 ;
          end
          obj.ClearPIMG();
          fprintf(obj.STDOUT,'%s: Cleaning_testpattern: step 3\n',datetime);
        case 4 % move to bottom-left of test pattern square
          obj.ShutterCloseOveride = false ; % ok to open shutter when moving now
          dest = [boundary(1) boundary(2)] ;
          reqmove = dest - obj.LaserPosition_mot ;
          if any(abs(reqmove)>obj.motrbktol)
            obj.movemirror(reqmove);
          else % done with this move
            tpstate = 5 ;
          end
          obj.ClearPIMG();
          fprintf(obj.STDOUT,'%s: Cleaning_testpattern: step 4\n',datetime);
        case 5 % move to bottom-right of test pattern square
          obj.ShutterCloseOveride = false ; % ok to open shutter when moving now
          dest = [boundary(1)+boundary(3) boundary(2)] ;
          reqmove = dest - obj.LaserPosition_mot ;
          if any(abs(reqmove)>obj.motrbktol)
            obj.movemirror(reqmove);
          else % done with this move
            if obj.State == CathodeServicesState.Cleaning_testpattern % include diagonal moves for test pattern, not for energy set pattern
              tpstate = 6 ;
            else
              tpstate = 2 ;
            end
          end
          obj.ClearPIMG();
          fprintf(obj.STDOUT,'%s: Cleaning_testpattern: step 5\n',datetime);
        case 6 % move diagonally to top-left
          obj.ShutterCloseOveride = false ; % ok to open shutter when moving now
          dest = [boundary(1) boundary(2)+boundary(4)] ;
          reqmove = dest - obj.LaserPosition_mot ;
          if any(abs(reqmove)>obj.motrbktol)
            obj.movemirror(reqmove);
          else % done with this move
            tpstate = 7 ;
          end
          obj.ClearPIMG();
          fprintf(obj.STDOUT,'%s: Cleaning_testpattern: step 6\n',datetime);
        case 7 % move diagonally to bottom-right
          obj.ShutterCloseOveride = false ; % ok to open shutter when moving now
          dest = [boundary(1)+boundary(3) boundary(2)] ;
          reqmove = dest - obj.LaserPosition_mot ;
          if any(abs(reqmove)>obj.motrbktol)
            obj.movemirror(reqmove);
          else % done with this move
            tpstate = 2 ;
          end
          obj.ClearPIMG();
          fprintf(obj.STDOUT,'%s: Cleaning_testpattern: step 7\n',datetime);
        case 8 % stop process and return to center of cleaning pattern area
          obj.ShutterCloseOveride = true ; % don't open shutter until on auto pattern path
          CloseShutterAndVerify(obj) ;
          obj.ClearPIMG(); % Clear integrated image data
          dest = obj.CleaningCenter ;
          reqmove = dest - obj.LaserPosition_mot ;
          if any(abs(reqmove)>obj.motrbktol)
            obj.movemirror("home");
          else % done with this move
            tpstate = 0 ;
            obj.State = CathodeServicesState.Cleaning_definearea ;
            obj.ShutterCloseOveride = false ; 
          end
          fprintf(obj.STDOUT,'%s: Cleaning_testpattern: finishing\n',datetime);
      end
      
    end
    function movemirror(obj,dpos,cmd)
      %MOVEMIRROR Command laser mirror relative move or go home
      %movemirror(delta-pos) Sets velocity based on object properties
      %movemirror(delta-pos,"fast") Uses "home" motor velocity
      %movemirror("home") Go to home position defined by CleaningCenter or MapCenter property
      %opens laser shutter if in auto mode, then make relative move
      
      if ~exist('dpos','var') || ( ~isequal(dpos,"home") && ( ~isnumeric(dpos) || length(dpos)~=2) )
        error('Incorrect movemirror command structure')
      end
      xm = caget(obj.pvs.lsr_posx) ;
      ym = caget(obj.pvs.lsr_posy) ; % current mirror command position
      if isequal(dpos,"home")
        if obj.gui.TabGroup.SelectedTab ==  obj.gui.TabGroup.Children(1) % Laser Cleaning Tab
          dpos = obj.CleaningCenter - [xm ym] ;
        elseif obj.gui.TabGroup.SelectedTab ==  obj.gui.TabGroup.Children(2) % QE Map Tab
          dpos = obj.MapCenter - [xm ym] ;
        end
        vel = obj.MotorVeloHome ;
      elseif exist('cmd','var') && cmd=="fast"
        vel = obj.MotorVeloHome ;
      else
        vel = obj.MotorVelo ;
      end
      if ~any(abs(dpos))>obj.motrbktol
        return
      end
      caput(obj.pvs.lsr_xvel,vel);
      caput(obj.pvs.lsr_yvel,vel);
      xnew = xm+dpos(1);
      ynew = ym+dpos(2);
      caput(obj.pvs.lsr_xmov,xnew);
      caput(obj.pvs.lsr_ymov,ynew);
      obj.LaserMotorStopped=false;
    end
    function guicmd(obj,cmd,varargin)
      %GUICMD Pass commands from GUI
      switch string(cmd)
        case "acq-cont"
          caput(obj.pvs.CCD_acqsetmode,2);
          caput(obj.pvs.CCD_acqset,1);
        case "acq-single"
          obj.imupdate=true; % force refresh of all graphics
          caput(obj.pvs.CCD_acqsetmode,0);
          caput(obj.pvs.CCD_acqset,1);
        case "acq-stop"
          caput(obj.pvs.CCD_acqset,0);
        case "stop-reset"
          obj.StopResetGUI;
        otherwise
          error('Unknown command: %s',cmd)
      end
    end
    function StopResetGUI(obj,cmd)
      if obj.gui.TabGroup.SelectedTab ==  obj.gui.TabGroup.Children(1) % Laser Cleaning Tab
        sbut = obj.gui.STOPButton ;
      elseif obj.gui.TabGroup.SelectedTab ==  obj.gui.TabGroup.Children(2) % QE Map Tab
        sbut = obj.gui.STOPButton_2 ;
      end
      if ~exist('cmd','var')
        cmd=sbut.Text;
      end
      switch cmd
        case 'STOP'
          obj.ShutterCloseOveride = true ;
          obj.CloseShutterAndVerify ;
          caput(obj.pvs.lsr_stopx,1); caput(obj.pvs.lsr_stopy,1);
          switch obj.State
            case {CathodeServicesState.Cleaning_testpattern,CathodeServicesState.Cleaning_setenergypattern}
              obj.Proc_Cleaning_testpattern("Stop");
            case {CathodeServicesState.Cleaning_movingtonewline,CathodeServicesState.Cleaning_linescan}
              obj.Proc_Cleaning("Stop");
            case {CathodeServicesState.QEMap_movingtonewline,CathodeServicesState.QEMap_linescan}
              obj.Proc_QEMap("Stop");
          end
        case 'RESET'
          obj.gui.CLOSESwitch.Enable='on';
          sbut.Text='STOP';
          sbut.BackgroundColor='red';
          obj.AutoStop("reset");
          drawnow
      end
    end
    function isclosed = CloseShutterAndVerify(obj)
      persistent han
      if caget(obj.pvs.laser_shutterStatIn) && ~caget(obj.pvs.laser_shutterStatOut)
        isclosed=true;
        return
      end
      caput(obj.pvs.laser_shutterCtrl,'Yes');
      timeout=3;
      t0=tic;
      isclosed = false ;
      while ~caget(obj.pvs.laser_shutterStatIn) || caget(obj.pvs.laser_shutterStatOut)
        pause(0.1);
        if toc(t0)>timeout
          if isempty(han) || ~ishandle(han)
            han=errordlg('MPS Laser Shutter Not reported closed, check!','MPS Laser Shutter not closed');
          end
          return
        end
      end
      isclosed = true ;
    end
    function isopen = OpenShutterAndVerify(obj)
      persistent han
      if ~caget(obj.pvs.laser_shutterStatIn) && caget(obj.pvs.laser_shutterStatOut)
        isopen=true;
        return
      end
      caput(obj.pvs.laser_shutterCtrl,'No');
      timeout=3;
      t0=tic;
      isopen = false ;
      while caget(obj.pvs.laser_shutterStatIn) || ~caget(obj.pvs.laser_shutterStatOut)
        pause(0.1);
        if toc(t0)>timeout
          if isempty(han) || ~ishandle(han)
            han=errordlg('MPS Laser Shutter Not reported open, check!','MPS Laser Shutter not open');
          end
          return
        end
      end
      isopen = true ;
    end
    function AutoStop(obj,reasons,txt)
      persistent lastreasons prevstate
      if isequal(reasons,"reset")
        lastreasons="none";
        if ~isempty(prevstate) % put back into state when error issued
          obj.State=prevstate;
        end
        if exist('ABORTREQ','file') % Delete abort request file- generated by remote process
          delete('ABORTREQ');
        end
        return
      end
      isclosed = obj.CloseShutterAndVerify ;
      if isclosed; obj.gui.CLOSESwitch.Enable=false; end
      if isequal(reasons,lastreasons) % process warnings etc if new failure reasons
        return
      else
        lastreasons=reasons;
      end
      prevstate = obj.State ;
      obj.State = CathodeServicesState(6) ;
      set(obj.gui.CleaningStatusEditField,'Value',text(obj.State));
      obj.gui.STOPButton.Text='RESET';
      obj.gui.STOPButton.BackgroundColor='yellow';
      fprintf(obj.STDERR,'%s: F2_CathodeServicesApp AutoStop:\n%s\n',datetime,reasons(:));
      if exist('txt','var') && ~isempty(txt)
        fprintf(obj.STDERR,'%s\n',split(txt,';'));
      end
      waitfor(warndlg(sprintf('Auto shutdown Cathode Services Program:  \n%s\nCHECK LASER AND MPS SHUTTER STATUS  \nPush Reset Button to re-start',reasons(:)),'MPS Laser Shutter Inserted'));
    end
    function shutdown(obj)
      %SHUTDOWN Actions to perform when closing GUI: including exit Matlab session
      try
        % Store configuration state for re-load on next start
        obj.SaveConfig;
        % Restore initial PVs
        caput(obj.pvs.lsr_xvel,obj.initvelo(1));
        caput(obj.pvs.lsr_yvel,obj.initvelo(2));
        % Stop timers and exit matlab session
        stop(obj.pvlist);
        fprintf('Cleanup PV processes...\n');
        obj.pvlist.Cleanup;
        fprintf('Done with cleanup.\n');
      catch ME
        fprintf(obj.STDERR,ME.message);
      end
    end
    function ClearPIMG(obj)
      %CLEARPIMG Clear persistent image and other integrated data
      obj.pimg=[];
      obj.qint_f=[];
      obj.qint_t=[];
      obj.lint=[];
    end
    function SaveConfig(obj,fname)
      %SAVECONFIG Save key configuration data
      %SaveCongig() Saves "default" configuration data, loaded on next startup (this is done on app exit)
      %SaveConfig(fname) Save configuration data with user supplied file name
      configdata = struct;
      for iconf=1:length(obj.configprops)
        configdata.(obj.configprops(iconf)) = obj.(obj.configprops(iconf)) ;
      end
      if ~exist('fname','var')
        fname='F2_CathodeServices_configdata';
      end
      save(fname,'configdata');
    end
    function LoadConfig(obj,fname)
      %LOADCONFIG Load saved configuration parameters
      %LoadConfig() Loads "default" configuration parameters (from last time app exited)
      %LoadConfig(fname) Loads configuration parameters from user supplied file name
      if ~exist('fname','var') || isempty(fname)
        fname='F2_CathodeServices_configdata';
      end
      if ~exist(fname,'file') && ~exist(sprintf('%s.mat',fname),'file')
        error('Cannot find file: %s',fname)
      end
      ld=load(fname);
      if ~isfield(ld.configdata,'version') || ld.configdata.version~=obj.version
        error('Version mismatch between saved configuration file and running app')
      end
      fn=fieldnames(ld.configdata);
      for ifn=1:length(fn)
        if ~strcmp(fn{ifn},'version')
          obj.(fn{ifn})=ld.configdata.(fn{ifn});
        end
      end
      % If not the main GUI environment, bail here
      if isempty(obj.gui)
        return
      end
      % Write restored PV values and update GUIs
      obj.SetLimits("GunVacuumRange",obj.GunVacuumRange);
      obj.SetLimits("ImageIntensityRange",obj.ImageIntensityRange);
      obj.SetLimits("LaserEnergyRange",obj.LaserEnergyRange);
      obj.SetLimits("LaserSpotSizeRange",obj.LaserSpotSizeRange);
      obj.SetLimits("LaserPosition_tol",obj.LaserPosition_tol);
      obj.SetLimits("LaserFluenceRange",obj.LaserFluenceRange);
      obj.gui.HomeVELOmmsEditField.Value = double(obj.MotorVeloHome) ;
      obj.gui.StepSizeEditField.Value = double(obj.CleaningStepSize*1e6) ;
      obj.gui.PulsesateachpositionSpinner.Value = double(obj.CleaningNumPulsesPerStep) ;
      obj.gui.CleaningRadiusmmEditField.Value = double(obj.CleaningRadius*1e3) ;
      obj.gui.ccentEdit_x.Value = double(obj.CleaningCenter(1)) ;
      obj.gui.ccentEdit_y.Value = double(obj.CleaningCenter(2)) ;
      obj.gui.StartPositionKnob.Value = num2str(obj.CleaningStartPosition) ;
      obj.gui.ImageRotation0Menu.Text = sprintf('Image Rotation = %d',obj.imrot);
      obj.SetMirrCal(obj.VCC_mirrcal);
    end
    function SaveImage(obj,fname)
      if ~exist('fname','var')
        error('No file name provided');
      end
      try
        img=get(obj.gui.UIAxes.Children(end),'CData');
        xdata=get(obj.gui.UIAxes.Children(end),'XData');
        ydata=get(obj.gui.UIAxes.Children(end),'YData');
        save(fname,'img','xdata','ydata');
      catch ME
        errordlg('Error saving image data: unexpected data format','Image Save Failed');
        throw(ME)
      end
    end
    function LoadImage(obj,fname)
      % Load image data
      if ~exist(fname,'file') && ~exist(sprintf('%s.mat',fname),'file')
        error('Cannot find file: %s',fname)
      end
      try
        ld=load(fname,'img','xdata','ydata');
      catch ME
        errorglg('Failed to load image data, incorrect save format?','Image Load Failed');
        throw(ME)
      end
      % Display image
      han=obj.gui.UIAxes;
      hold(han,'off');
      cla(han);
      axis(han,[ld.xdata(1) ld.xdata(end) ld.ydata(1) ld.ydata(end)]),hold(han,'on')
      imagesc(han,ld.img,'XData',ld.xdata,'YData',ld.ydata); xlabel(han,'X [mm]'); ylabel(han,'Y [mm]');
      axis(han,'image');
    end
  end
  % get/set methods
  methods
    function SetMirrCal(obj,cal)
      %SETMIRRCAL Set calibration coefficients for mirror position readback
      %SetMirrCal([offX,offY,scalX,scalY]) 
      if ~exist('cal','var') || length(cal)~=4
        error('Must supply cal as 1x4 vector [offX,offY,scalX,scalY]');
      end
      obj.VCC_mirrcal=cal;
      obj.pvs.lsr_posx.conv = obj.VCC_mirrcal([3 1]) ;
      obj.pvs.lsr_posy.conv = obj.VCC_mirrcal([4 2]) ;
      obj.gui.MotorCALMenu.Text = sprintf('Motor CAL = [%g %g %g %g]',cal) ;
    end
    function SetLimits(obj,par,limits)
      %SETLIMITS Set lower,upper limits for PV and derived parameters
      %SetLimits("GunVacuumRange",[low,high])
      %SetLimits("ImageIntensityRange",[low,high])
      %SetLimits("LaserEnergyRange",[low,high])
      %SetLimits("LaserSpotSizeRange",[low,high])
      %SetLimits("LaserFluence",[low,high])
      %SetLimits("LaserPosition_tol",tol)
      
      % Changing PV object limits causes GUI objects to be updated
      limits=double(limits);
      try
        obj.(par)=limits;
        switch string(par)
          case "GunVacuumRange"
            obj.pvs.gun_vacuum.limits = limits ;
            caput(obj.pvs.watchdog_gunvaclimitHI,1e-9*double(limits(2)));
            caput(obj.pvs.watchdog_gunvaclimitLO,1e-9*double(limits(1)));
          case "ImageIntensityRange" % no associated PV, just change range on GUI gauge
            obj.pvs.CCD_intensity.limits = limits ;
            rng=range(limits); buf=0.1;
            obj.gui.ImageIntensityGauge.Limits = double([limits(1)-rng*buf,limits(2)+rng*buf]) ;
            obj.gui.ImageIntensityGauge.ScaleColors = [1,0,0;0,1,0;1,0,0] ;
            obj.gui.ImageIntensityGauge.ScaleColorLimits = ...
              double([limits(1)-rng*buf,limits(1);limits(1),limits(2);limits(2),limits(2)+rng*buf]) ;
          case "LaserEnergyRange"
            obj.pvs.laser_energy.limits = limits ;
            caput(obj.pvs.watchdog_laserlimitHI,double(limits(2)));
            caput(obj.pvs.watchdog_laserlimitLO,double(limits(1)));
          case "LaserSpotSizeRange"
            obj.pvs.CCD_spotsize.limits = limits ;
          case "LaserFluenceRange" % no associated PV, just change range on GUI gauge
            lims2=ceil((limits(2)+range(limits)*0.15)*10)/10;
            obj.gui.Gauge_3.Limits = [limits(1) lims2] ;
            obj.gui.Gauge_3.ScaleColorLimits = [limits(1) limits(2); limits(2) lims2 ];
            obj.gui.Gauge_3.ScaleColors = {'green','red'} ;
          case "LaserPosition_tol"
            obj.gui.LaserPosTolMenu.Text = sprintf('Laser Pos Tol = %g um',limits*1000) ;
          otherwise
            error('Unknown limit parameter');
        end
      catch ME
        error('Error setting limits for %s :\n %s\n',par,ME.message);
      end
    end
    function numlines=get.CleaningNumLines(obj)
      switch obj.CleaningStartPosition
        case {1,3} % horizontal moves
          numlines=ceil((obj.CleaningRadius*2)/(obj.CleaningStepSize));
        case {2,4} % vertical moves
          w=abs(linspace(0.00000001,obj.CleaningRadius*2-0.0000001,obj.CleaningNumCols)-obj.CleaningRadius);
          crdlen=2.*sqrt(obj.CleaningRadius^2-w.^2);
          numlines=ceil(crdlen./(obj.CleaningStepSize));
      end
    end
    function numcols=get.CleaningNumCols(obj)
      switch obj.CleaningStartPosition
        case {1,3} % horizontal moves
          h=abs(linspace(0.00000001,obj.CleaningRadius*2-0.0000001,obj.CleaningNumLines)-obj.CleaningRadius);
          crdlen=2.*sqrt(obj.CleaningRadius^2-h.^2);
          numcols=ceil(crdlen./(obj.CleaningStepSize));
        case {2,4} % vertical moves
          numcols=ceil((obj.CleaningRadius*2)/obj.CleaningStepSize);
      end
    end
    function time=get.CleaningTimeRemaining(obj) % minutes
      switch obj.CleaningStartPosition
        case {1,3} % horizontal moves
          numcols=obj.CleaningNumCols;
          numsteps=sum(numcols);
          if obj.CleaningLineNum>1
            stepsdone=sum(numcols(1:obj.CleaningLineNum-1))+obj.CleaningColNum-1;
          else
            stepsdone=obj.CleaningColNum-1;
          end
          time = (double(numsteps-stepsdone)*double(obj.CleaningNumPulsesPerStep)) / double(obj.RepRate) / 60 ;
        case {2,4} % vertical moves
          numlines=obj.CleaningNumLines;
          numsteps=sum(numlines);
          if obj.CleaningColNum>1
            stepsdone=sum(numlines(1:obj.CleaningColNum-1))+obj.CleaningLineNum-1;
          else
            stepsdone=obj.CleaningLineNum-1;
          end
          time = (double(numsteps-stepsdone)*double(obj.CleaningNumPulsesPerStep)) / double(obj.RepRate) / 60 ;
      end
    end
    function numlines=get.MapNumLines(obj)
      switch obj.MapStartPosition
        case {1,3} % horizontal moves
          numlines=ceil((obj.MapRadius*2)/(obj.MapStepSize));
        case {2,4} % vertical moves
          w=abs(linspace(0.00000001,obj.MapRadius*2-0.0000001,obj.MapNumCols)-obj.MapRadius);
          crdlen=2.*sqrt(obj.MapRadius^2-w.^2);
          numlines=ceil(crdlen./(obj.MapStepSize));
      end
    end
    function numcols=get.MapNumCols(obj)
      switch obj.MapStartPosition
        case {1,3} % horizontal moves
          h=abs(linspace(0.00000001,obj.MapRadius*2-0.0000001,obj.MapNumLines)-obj.MapRadius);
          crdlen=2.*sqrt(obj.MapRadius^2-h.^2);
          numcols=ceil(crdlen./(obj.MapStepSize));
        case {2,4} % vertical moves
          numcols=ceil((obj.MapRadius*2)/obj.MapStepSize);
      end
    end
    function time=get.MapTimeRemaining(obj) % minutes
      switch obj.MapStartPosition
        case {1,3} % horizontal moves
          numcols=obj.MapNumCols;
          numsteps=sum(numcols);
          if obj.MapLineNum>1
            stepsdone=sum(numcols(1:obj.MapLineNum-1))+obj.MapColNum-1;
          else
            stepsdone=obj.MapColNum-1;
          end
          time = (double(numsteps-stepsdone)*double(obj.MapNumPulsesPerStep)) / double(obj.RepRate) / 60 ;
        case {2,4} % vertical moves
          numlines=obj.MapNumLines;
          numsteps=sum(numlines);
          if obj.MapColNum>1
            stepsdone=sum(numlines(1:obj.MapColNum-1))+obj.MapLineNum-1;
          else
            stepsdone=obj.MapLineNum-1;
          end
          time = (double(numsteps-stepsdone)*double(obj.MapNumPulsesPerStep)) / double(obj.RepRate) / 60 ;
      end
    end
    function fluence=get.LaserFluence(obj) % uJ/cm^2
      fluence = obj.LaserEnergy / (1e-2*obj.LaserSpotSize)^2 ;
    end
    function velo = get.MotorVelo(obj) % mm/s
      t = double(obj.CleaningNumPulsesPerStep) / double(obj.RepRate) ; % time to traverse 1 step size
      velo = (double(obj.CleaningStepSize)*1000) / t ; % velocity in mm / s
    end
    function adist = get.AcclDist(obj) % mm
      adist = obj.MotorVelo .* [ obj.pvs.lsr_xaccl.val{1} obj.pvs.lsr_yaccl.val{1} ] ;
    end
    function freq=get.wd_freq(obj)
      if obj.State == CathodeServicesState.Cleaning_linescan
        tave = mean(obj.wd_time_noncleaning,'omitnan');
      else
        tave = mean(obj.wd_time,'omitnan');
      end
      freq = 1/tave ;
      if isnan(freq)
        freq=0;
      end
    end
    function freqerr=get.wd_freqerr(obj)
      freqerr = std(1./obj.wd_time,'omitnan');
      if isnan(freqerr)
        freqerr=0;
      end
    end
    function set.buflen(obj,newlen)
      if newlen<10 || newlen>10000
        return
      end
      oldlen=obj.buflen;
      if newlen>oldlen
        oldvals=obj.poshistory;
        obj.poshistory = nan(4,newlen,'single') ;
        obj.poshistory(:,1:oldlen)=oldvals;
      elseif newlen<oldlen
        obj.poshistory=obj.poshistory(:,1:newlen);
      end
      obj.buflen=newlen;
    end
  end
  % watchdog / GUI updaters
  methods(Access=private)
    function watchdogUD(obj,~,~) % called whenever a monitored pv value change
      persistent t0
      % If in an error state, don't do anything until error is cleared (with GUI reset button)
      if iserror(obj.State)
        return
      end
      % Drop update if last one still processing
      if obj.wd_running
        return
      end
      % Limit rate to twice reported laser firing rate 
      if ~isempty(t0) && toc(t0)<(0.5/obj.RepRate)
        return
      end
      obj.wd_running=true;
      try
        obj.watchdog();
        if ~isempty(t0)
          if obj.State ~= CathodeServicesState.Cleaning_linescan
            obj.wd_time_noncleaning(obj.wd_time_ind_noncleaning) = toc(t0);
            if obj.wd_time_ind_noncleaning>=length(obj.wd_time_noncleaning)
              obj.wd_time_ind_noncleaning = 1 ;
            else
              obj.wd_time_ind_noncleaning = obj.wd_time_ind_noncleaning + 1 ;
            end
          end
          obj.wd_time(obj.wd_time_ind) = toc(t0);
          if obj.wd_time_ind>=length(obj.wd_time)
            obj.wd_time_ind = 1 ;
          else
            obj.wd_time_ind = obj.wd_time_ind + 1 ;
          end
        end
        t0=tic;
        obj.gui.EditField_15.Value=obj.wd_freq;
      catch ME
        obj.wd_running=false;
        rethrow(ME)
      end
      obj.wd_running=false;
    end
    function watchdog(obj,~,~) % called from watchdogUD timer when a PV value changes
      %WATCHDOG Process PV value chages, compute running state and take corresponding actions
      persistent prevVals repind lastepicswatchdog tic_epicswatchdog lastpos fhan lastStateConsidered sufficientRate

      % Poke EPICS watchdog keepalive PV
      caput(obj.pvs.watchdog_keepalive,1);
      caput(obj.pvs.watchdog_keepalive,1);
      
      % Reasons to auto close laser shutter and put app in error state
      reasons = ["Automated laser pattern enabled without streaming CCD image",...  % 1
        "Repeated PV readings",...                                                  % 2
        "Gun RF not OFF",...                                                        % 3
        "EPICS watchdog isalive beacon stopped incrementing",...                    % 4
        "Laser not in motion with shutter OPEN whilst in cleaning mode",...         % 5
        "Detected laser position mismatches expectation from mirror motors",...     % 6
        "Gun Vacuum Out of range",...                                               % 7
        "Image Intensity Out of range",...                                          % 8
        "Laser energy Out of range",...                                             % 9
        "Laser spot size Out of range",...                                          % 10
        "Laser Position Out of Range",...                                           % 11
        "Laser telescope not inserted",...                                          % 12
        "Laser fluence out of range",...                                            % 13
        "Integrated image count too high (motors stuck?)" ] ;                       % 14
      inerr = false(1,length(reasons)) ;
      etxt=""; % Additional error text to display goes here, split character is ";"
      
      % Actions to take based on telescope in/out state
      if strcmp(obj.pvs.laser_telescope.val{1},'IN') % telescope inserted (small laser spot)
        if ~isautopattern(obj.State) && obj.State~=CathodeServicesState.Cleaning_definearea && ...
            obj.State~=CathodeServicesState.Standby_cleaninglasermode && obj.State~=CathodeServicesState.QEMap_definearea
          obj.State = CathodeServicesState.Standby_cleaninglasermode ;
        end
      else
        if isautopattern(obj.State)
          inerr(12) = true ;
        elseif obj.State~=CathodeServicesState.Cleaning_definearea && obj.State~=CathodeServicesState.Standby_opslasermode && obj.State~=CathodeServicesState.QEMap_definearea
          obj.State = CathodeServicesState.Standby_opslasermode ;
        end
      end
      obj.CCD_stream = obj.pvs.CCD_acq.val{1}==1 & obj.pvs.CCD_acqmode.val{1}==2 ; % is in continuous mode and acquiring?
      
      % Update time remaining
      if strcmp(obj.gui.TabGroup.SelectedTab.Title,'Laser Cleaning')
        timeremain=obj.CleaningTimeRemaining;
        if obj.CleaningLineNum==1 && obj.CleaningColNum==1
          obj.gui.min00secGauge.Limits=[0 ceil(timeremain)];
        end
        obj.gui.min00secGauge.Value=timeremain;
        obj.gui.min00secGaugeLabel.Text=sprintf('%d min %02d sec',floor(timeremain),round((timeremain-floor(timeremain))*60));
      else
        timeremain=obj.MapTimeRemaining;
        if obj.MapLineNum==1 && obj.MapColNum==1
          obj.gui.min00secGauge_2.Limits=[0 ceil(timeremain)];
        end
        obj.gui.min00secGauge_2.Value=timeremain;
        obj.gui.min00secGauge_2Label.Text=sprintf('%d min %02d sec',floor(timeremain),round((timeremain-floor(timeremain))*60));
      end
      
      % Get current PV & other derived values of interest and set corresponding properties
      epicswatchdog = obj.pvs.watchdog_isalive.val{1} ;
      obj.GunVacuum = obj.pvs.gun_vacuum.val{1} ;
      obj.LaserEnergy = obj.pvs.laser_energy.val{1} ;
      obj.LaserPosition_mot = [obj.pvs.lsr_posx.val{1} obj.pvs.lsr_posy.val{1}];
      gunrfon = ~strcmp(obj.pvs.gun_rfstate.val{1},'OFF') ;
      obj.LaserSpotSize = obj.pvs.CCD_spotsize.val{1} ; % use max(std_x std_y) as circular spot size
      obj.ImageIntensity = obj.pvs.CCD_intensity.val{1} ;
      obj.LaserPosition_img = [obj.pvs.CCD_xpos.val{1} obj.pvs.CCD_ypos.val{1}] ;
      newreprate = obj.pvs.laser_reprate.val{1} ;
      if newreprate ~= obj.RepRate
        if newreprate<=0
          obj.RepRate = 30 ;
        else
          obj.RepRate = newreprate ;
        end
        obj.pvlist.pset('timeout',0.01 + 1/double(obj.RepRate));
      end
      
      % Store buffered data if requested
      if obj.bufacq && ( isempty(lastpos) || any(abs(obj.LaserPosition_img-lastpos)>obj.LaserPosition_tol) ) % store positions in history buffer if substantially moved
        lastpos = obj.LaserPosition_img ;
        obj.poshistory(1,obj.bufpos) = obj.LaserPosition_img(1) ;
        obj.poshistory(2,obj.bufpos) = obj.LaserPosition_img(2) ;
        obj.poshistory(3,obj.bufpos) = obj.LaserPosition_mot(1) ;
        obj.poshistory(4,obj.bufpos) = obj.LaserPosition_mot(2) ;
        if obj.bufpos >= obj.buflen
          obj.bufpos=1;
        else
          obj.bufpos=obj.bufpos+1;
        end
        % Plot calibration data
        if ~isempty(obj.CalibHan) && all(ishandle(obj.CalibHan)) % plot mirror vs image position history in diagnostic window if requested
          plot(obj.CalibHan(1),obj.poshistory(3,:),obj.poshistory(1,:),'rx');
          grid(obj.CalibHan(1),'on');
          xlabel(obj.CalibHan(1),'Mirror Motor Reported H Position [mm]');
          ylabel(obj.CalibHan(1),'Image Reported H Position [mm]');
          plot(obj.CalibHan(2),obj.poshistory(4,:),obj.poshistory(2,:),'bo');
          grid(obj.CalibHan(2),'on');
          xlabel(obj.CalibHan(2),'Mirror Motor Reported V Position [mm]');
          ylabel(obj.CalibHan(2),'Image Reported V Position [mm]');
        end
      end
      
      % Write CCD image to axis if image changed
      % - if on cleaning line scan, only draw image if update rate is fast enough
      if isempty(lastStateConsidered) || lastStateConsidered~=obj.State % keep same decision until State changes
        lastStateConsidered = obj.State ;
        sufficientRate = true ;
        if (obj.State == CathodeServicesState.Cleaning_linescan || obj.State == CathodeServicesState.QEMap_linescan) && (obj.wd_freq < obj.RepRate) 
          sufficientRate = false ;
        end
      end
      if ~sufficientRate
        obj.getimg ; % Get and integrate persistent image array
      elseif obj.gui.DetachButton.Value % stream image to remote figure window
        if isempty(fhan) || ~ishandle(fhan)
          fhan=axes;
          obj.imdraw('reset');
        end
        obj.imdraw(fhan) ;
      else % stream image to GUI figure window
        obj.imdraw(obj.gui.UIAxes) ;
      end
      
      % Check for integrated image pixels rising above threshold value- tests if motors got stuck in some way not detected by other means
      if strcmp(obj.gui.TabGroup.SelectedTab.Title,'Laser Cleaning')
        if exist('ABORTREQ','file') || ( ~isempty(obj.pimg) && max(obj.pimg(:)) > ( double(obj.CleaningNumPulsesPerStep) * double(obj.ImageCIntMax) * double(obj.ImageIntensityRange(2)) ) )
          inerr(14) = true ;
        end
      else
        if exist('ABORTREQ','file') || ( ~isempty(obj.pimg) && max(obj.pimg(:)) > ( double(obj.MapNumPulsesPerStep) * double(obj.ImageCIntMax) * double(obj.ImageIntensityRange(2)) ) )
          inerr(14) = true ;
        end
      end
      
      % CCD image should be streaming in auto pattern moving States, check that here
      if isautopattern(obj.State) && ~obj.CCD_stream
        inerr(1) = true ;
      end
      
      % Check watched values are changing
      if obj.CheckRepVals
        nrepmax=10; % max number of repeated readings to allow
        watchpv=["gun_vacuum","laser_energy","lsr_posx","lsr_posy","CCD_xpos","CCD_ypos","CCD_intensity"];
        if ~obj.CCD_stream % only check vars related to CCD if in streaming mode
          iwatch=1:4;
        else
          iwatch=1:length(watchpv);
        end
        if isempty(repind)
          repind=zeros(1,length(watchpv));
          prevVals=nan(1,length(watchpv));
        end
        repval=false(1,length(watchpv));
        for ipv=iwatch
          if isequal(obj.pvs.(watchpv(ipv)).vals{1},prevVals{ipv})
            repval(ipv)=true;
          end
          prevVals{ipv} = obj.pvs.(watchpv(ipv)).vals{1} ;
        end
        repind(repval)=repind(repval)+1;
        repind(~repval)=0;
        if any(repind>=nrepmax)
          for irep=find(repind>=nrepmax)
            etxt=etxt+sprintf(">%d Repeated readings for %s;",nrepmax,watchpv(irep));
          end
          inerr(2) = true ;
        end
      end
      
      % Check Gun RF is off if it is supposed to be
      if isgunoffstate(obj.State) && gunrfon
        inerr(3) = true ;
      end
      
      % Check EPICS watchdog running (check at <1Hz, and command autostop if in an automatic operating mode)
      if isempty(tic_epicswatchdog) || toc(tic_epicswatchdog)>1.5
        if ~isempty(lastepicswatchdog) && epicswatchdog==lastepicswatchdog
          inerr(4) = true ;
          obj.gui.RunningLamp.Color='red';
          tic_epicswatchdog=tic;
        else
          obj.gui.RunningLamp.Color='green';
          tic_epicswatchdog=tic;
        end
        lastepicswatchdog=epicswatchdog;
      end
      
      % If in small spot mode, shutter should only be open when laser in motion
       % If laser motion currently in progress, no further action required, otherwise shutter should be closed
      if obj.State ~= CathodeServicesState.Standby_opslasermode
        if obj.LaserMotorStopped==1 && ~obj.CloseShutterAndVerify
          inerr(5) = true ;
        end
      end
      
      % Check laser is where the mirror motors say it is (error if auto pattern and if not calibrating this)
      if any(abs(obj.LaserPosition_img-obj.LaserPosition_mot)>obj.LaserPosition_tol)
        if obj.State~=CathodeServicesState.Cleaning_testpattern && caget(obj.pvs.laser_shutterStatIn) && ~obj.ShutterCloseOveride
          inerr(11) = true ;
        end
        obj.gui.InrangeLamp.Color='red';
      else
        obj.gui.InrangeLamp.Color='green';
      end
      
      % Check values in range
      if obj.GunVacuum > obj.GunVacuumRange(2) || obj.GunVacuum < obj.GunVacuumRange(1)
        etxt=etxt+sprintf("Gun Vacuum Out of range: val= %g range= [%g %g];",obj.GunVacuum,obj.GunVacuumRange);
        inerr(7) = true ;
      end
      if obj.ImageIntensity > obj.ImageIntensityRange(2)
        etxt=etxt+sprintf("Image Intensity Out of range: val= %g range= [%g %g];",obj.ImageIntensity,obj.ImageIntensityRange);
        inerr(8) = true ;
      end
      if obj.LaserEnergy < obj.LaserEnergyRange(1) || obj.LaserEnergy > obj.LaserEnergyRange(2)
        etxt=etxt+sprintf("Laser energy Out of range: val= %g range= [%g %g];",obj.LaserEnergy,obj.LaserEnergyRange);
        inerr(9) = true ;
      end
      if obj.LaserSpotSize < obj.LaserSpotSizeRange(1)
        etxt=etxt+sprintf("Laser spot size Out of range: val= %g range= [%g %g];",obj.LaserSpotSize,obj.LaserSpotSizeRange);
        inerr(10) = true ;
      end
      % Deal with laser fluence update and range checking
      fluence = obj.LaserFluence ;
      obj.gui.Gauge_3.Value = double(fluence) ;
      if fluence<obj.LaserFluenceRange(1) || fluence>obj.LaserFluenceRange(2)
        obj.gui.Gauge_3.BackgroundColor=[0.7 0 0];
        inerr(13) = true ;
      else
        obj.gui.Gauge_3.BackgroundColor=[1 1 1];
      end
      
      % If in error state, check shutter off and control disabled if in auto mode: if laser fluence limit exeeded stop anyway
      if ( fluence>obj.LaserFluenceRange(2) || any(inerr) ) && ~caget(obj.pvs.laser_shutterStatIn)
        obj.AutoStop(reasons(inerr),etxt);
      else
        txt = text(obj.State) ;
        if strcmp(obj.gui.TabGroup.SelectedTab.Title,'Laser Cleaning')
          if obj.State == CathodeServicesState.Cleaning_linescan || obj.State == CathodeServicesState.Cleaning_movingtonewline
            switch obj.CleaningStartPosition
              case {1,3} % horizontal moves
                nline = obj.CleaningNumLines ;
                iline = obj.CleaningLineNum ;
              otherwise
                nline = obj.CleaningNumCols ;
                iline = obj.CleaningColNum ;
            end
            txt=sprintf("%s (Line %d/%d)",txt,iline,nline);
          end
          obj.gui.CleaningStatusEditField.Value=txt;
        else
          if obj.State == CathodeServicesState.QEMap_linescan || obj.State == CathodeServicesState.QEMap_movingtonewline
            switch obj.MapStartPosition
              case {1,3} % horizontal moves
                nline = obj.MapNumLines ;
                iline = obj.MapLineNum ;
              otherwise
                nline = obj.MapNumCols ;
                iline = obj.MapColNum ;
            end
            txt=sprintf("%s (Line %d/%d)",txt,iline,nline);
          end
          obj.gui.MappingStatusEditField.Value=txt;
        end
      end
      
      % Process automatic pattern steps
      switch obj.State
        case {CathodeServicesState.Cleaning_testpattern,CathodeServicesState.Cleaning_setenergypattern}
          obj.Proc_Cleaning_testpattern(obj.State);
        case {CathodeServicesState.Cleaning_movingtonewline,CathodeServicesState.Cleaning_linescan}
          obj.Proc_Cleaning();
        case {CathodeServicesState.QEMap_movingtonewline,CathodeServicesState.QEMap_linescan}
          obj.Proc_QEMap();
      end
      
      % force drawnow?
      if obj.dnCMD
        drawnow;
      end
      
    end
  end
  methods
    function [nx,ny,img] = getimg(obj)
      nx=obj.pvs.CCD_nx.val{1}; ny=obj.pvs.CCD_ny.val{1};
      obj.pvs.CCD_img.nmax=round(nx*ny);
      img=reshape(caget(obj.pvs.CCD_img),ny,nx);
      if obj.imrot>0 % rotate image by multiples of 90 degrees
        img=rot90(img,obj.imrot);
      end
      % If automated program running, integrate image -> abort if max integrated signal goes above ImageCIntMax threhold
      if isautopattern(obj.State) && obj.pvs.laser_shutterStatOut.val{1}
        if isempty(obj.pimg) || ~isequal(size(img),size(obj.pimg))
          obj.pimg=img;
        else
          obj.pimg=obj.pimg+img;
        end
        % If QE Mapping, integrate charge and laser data here
        if obj.State == CathodeServicesState.QEMap_linescan
          imref = double(img)./max(double(img(:))) ;
          if isempty(obj.qint_f)
            obj.qint_f = imref.*double(obj.pvs.fcup_val.val{1}) ;
            obj.qint_t = imref.*double(obj.pvs.torr_val.val{1}) ;
            obj.lint = imref.*double(obj.pvs.laser_energy.val{1}) ;
          else
            obj.qint_f = obj.qint_f + imref.*double(obj.pvs.fcup_val.val{1}) ;
            obj.qint_t = obj.qint_t + imref.*double(obj.pvs.torr_val.val{1}) ;
            obj.lint = obj.lint + imref.*double(obj.pvs.laser_energy.val{1}) ;
          end
        end
      end
      % Present integrated image for display if appropriate
      if ~isempty(obj.pimg)
        if obj.State==CathodeServicesState.Cleaning_linescan || obj.State==CathodeServicesState.Cleaning_movingtonewline || obj.State==CathodeServicesState.Cleaning_definearea
          img=obj.pimg;
        elseif obj.State==CathodeServicesState.QEMap_linescan || obj.State==CathodeServicesState.QEMap_movingtonewline || obj.State==CathodeServicesState.QEMap_definearea
          if obj.gui.UseLaserDataSwitch.Value=='L'
            src = obj.lint ;
          else
            src = double(obj.pimg) ;
          end
          if obj.gui.UseTorroidDataSwitch.Value=='F'
            qdat = obj.qint_f ;
          else
            qdat = obj.qint_t ;
          end
          src(src==0)=1; % protect agains divide by 0 errors
          img = qdat ./ src ;
        end
      end
    end
    function imdraw(obj,axhan)
      %IMDRAW VCC image drawing routine
      persistent tic_img tic_cmp lastimcount lastax
      
      if isequal(axhan,'reset')
        lastax=[];
        return
      end
      
      imdrawn=false;
      if obj.imupdate || isempty(lastimcount) || obj.pvs.CCD_counter.val{1}~=lastimcount
        % Update array size to get if required
        [nx,ny,img] = obj.getimg ;
        xax=[obj.pvs.CCD_x1.val{1} obj.pvs.CCD_x2.val{1}].*1e-3; yax=[obj.pvs.CCD_y1.val{1} obj.pvs.CCD_y2.val{1}].*1e-3; % Axis ranges
        % Update image in axes window at imupdate rate if intensity above threshold
        if obj.imupdate || isempty(tic_img) || ( ( obj.pvs.CCD_intensity.val{1} > obj.ImageIntensityRange(1) ) && toc(tic_img)>1/obj.imudrate )
          tic_img = tic ;
          if isempty(lastax) || ~isequal(lastax,[nx ny xax yax]) || obj.imupdate
            hold(axhan,'off');
            cla(axhan);
            axis(axhan,[xax yax]),hold(axhan,'on')
            imagesc(axhan,img,'XData',linspace(xax(1),xax(2),nx),'YData',linspace(yax(1),yax(2),ny)); xlabel(axhan,'X [mm]'); ylabel(axhan,'Y [mm]');
            axis(axhan,'image');
          else
            if length(axhan.Children)>1
              delete(axhan.Children(1:end-1));
            end
            axhan.Children(end).CData=img;
          end
          lastax=[nx ny xax yax];
          lastimcount=obj.pvs.CCD_counter.val{1};
          grid(axhan,'on'); axhan.Layer='top';
        end
        imdrawn=true;
      end

      % Draw on complications
      if imdrawn || isempty(tic_cmp) || toc(tic_cmp)>1/obj.imudrate
        tic_cmp = tic ;
        if length(axhan.Children)>1
          delete(axhan.Children(1:end-1));
        end
        hold(axhan,'on');
        % Plot expected positions
        plot(axhan,obj.LaserPosition_mot(1),obj.LaserPosition_mot(2),'rx','MarkerSize',20,'LineWidth',2); % position from laser mirrors
        plot(axhan,obj.LaserPosition_img(1),obj.LaserPosition_img(2),'k.','MarkerSize',20); % centroid calculated by EPICS based on image
        if uint8(obj.State)>1 % for all states other than standby, draw cleaning areas
          % Draw circle showing cleaning area and rectangles showing test and energy set patterns
          if isqemapstate(obj.State)
            xc=obj.MapCenter(1); yc=obj.MapCenter(2);
            r=obj.MapRadius.*1e3;
            ssize=obj.MapStepSize.*1e3*5;
          else
            xc=obj.CleaningCenter(1); yc=obj.CleaningCenter(2);
            r=obj.CleaningRadius.*1e3;
            ssize=obj.CleaningStepSize.*1e3*5;
          end
          x0=xc-r; y0=yc-r;
          % Cleaning area
          rectangle(axhan,'Position',[x0,y0,2*r,2*r],'Curvature',1,'EdgeColor','k','LineWidth',3,'LineStyle','-');
          plot(axhan,xc,yc,'k+','MarkerSize',20,'LineWidth',3);
          if isqemapstate(obj.State)
            obj.Proc_QEMap("SetBoundary",[x0,y0,2*r,2*r]);
          else
            obj.Proc_Cleaning("SetBoundary",[x0,y0,2*r,2*r]);
          end
          % Energy determination square pattern outside cleaning area
          rectangle(axhan,'Position',[x0-ssize,y0-ssize,2*r+2*ssize,2*r+2*ssize],'EdgeColor',[0.8500 0.3250 0.0980],'LineWidth',2,'LineStyle','--');
          % Test square pattern inside cleaning area
          xt=sqrt((2*r)^2/2)-ssize;
          rectangle(axhan,'Position',[xc-xt/2,yc-xt/2,xt,xt],'EdgeColor',[0.8500 0.3250 0.0980],'LineWidth',2,'LineStyle','--');
          if ~isqemapstate(obj.State)
            obj.Proc_Cleaning_testpattern("SetTestBoundary",[xc-xt/2,yc-xt/2,xt,xt]);
            obj.Proc_Cleaning_testpattern("SetEnergyBoundary",[x0-ssize,y0-ssize,2*r+2*ssize,2*r+2*ssize]);
          end
        end
        if ~isempty(obj.gui) && imdrawn
          drawnow limitrate
        end
      elseif imdrawn && ~isempty(obj.gui)
        drawnow limitrate
      end
      obj.imupdate = false ;
    end
  end
end

