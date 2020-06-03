classdef PV < handle
  % PV Control system process variable info, access methods and GUI links
 
  events
    PVStateChange % notifies upon successfull fetching of new PV value
  end
  properties
    name string = "none" % user supplied name for PV
    pvname string = "none" % Control system PV string (can be a vector associating this PV to multiple control PVs)
    monitor=false; % Flag this PV to be monitored
    type PVtype = PVtype.EPICS % Control system type (choose from listed Constant types)
    debug uint8 {mustBeMember(debug,[0,1,2])} = 2 % 0=live, 1=read only, 2=no connection
    units string = "none" % User specified units
    conv double % Scalar Conversion factor PV -> reported value or vector of polynomial coefficients (for use with polyval)
    guihan = 0 % gui handle to assoicate with this PV (e.g. to write out a variable or control a switch etc) (can be vector of handles)
    val cell % Last read value (can be any type and length depending on number and type of linked pvname PVs)
%     guiprec uint8 = 3 % Precision (significant figures) to use for GUI displays
    limits double {mustBeNumeric, mustBeGoodLimits(limits)} % Numerical upper/lower bounds for PV numeric values
    guiprefs GUI_PREFS % GUI preferences
    pvlogic char = '&' % logic used to combine multiple linked PVs for some GUI events, or negate, set to one of: '&','|','XOR','~','~&','~|','MAX','MIN','SUM'
    STDOUT = 1 % Standard echo output destination
    STDERR = 2 % Standard error output destination
    nmax uint32 % limit number of returned values (per control system PV) empty=default: get everything available
    putwait logical = false % wait for confirmation of pvput command
    timeout double {mustBePositive} = 3 % timout in s for waiting for a new value asynchrounously
    RunStartDelay = 3 % Wait this long after issuing Run method command to start polling of PVs (e.g. to allow startup scripts to complete)
  end
  properties(SetAccess=protected)
    pvdatatype % Data type used by java channel access client (set on constuction to force, else use default determined from control system) can be cell array of length pvname or scalar java class
    mode string = "r" % Access mode: "r", or "rw"
    polltime single = 1 % Rate at which to update monitored PVs
    utimer % Holder for update timer object
    context % EPICS java ca client context object (needs to be passed to contructor)
    channel cell % ca channel(s) for provided pvname(s)
  end
  properties(Access=private)
    lastlims % last limits value
    lastnm % last nmax value
    lastconv % last conversion values
    nativeclass % Matlab class type determined when opened channel and first read data
    future % asynchronous get handle
    future_tic % timestamp when asynchronous get initiated
  end
  properties(Constant)
    errcol=[0.7 0 0];
    caver string = "1.2.2" % EPICS CA version to use (ca-<version>.jar must be on matlab search path)
  end
  methods
    function obj=PV(cntxt,varargin)
      % PV Create control system proces variable object
      % ControlPV = PV(context [,prop_name,prop_val,...]);
      %  context: output from call to static PV.Initialize method (one time call per Matlab session)
      if nargin<1
        error('Must provide ''context'' (from output of static Initialize method');
      end
      obj.context=cntxt;
      obj.guiprefs = GUI_PREFS ; % Use default preferences unless overriden by user
      if nargin>1
        for iarg=2:2:nargin
          obj.(lower(varargin{iarg-1}))=varargin{iarg};
        end
      end
      % Loop through provided args again and take any action that requires
      % full set of parameters set first
      if nargin>1
        for iarg=2:2:nargin
          if strcmp(varargin{iarg-1},"mode")
            SetMode(obj,varargin{iarg});
          end
        end
      end
    end
    function SetMode(obj,mode)
      %MODE Read/write mode for interface with GUIs
      mode=string(mode);
      if ~isempty(obj.guihan)
        switch mode
          case "r"
          case "rw"
            % Set GUI callbacks on editable fields or toggle  or state
            % buttons
            for ihan=1:length(obj.guihan) % Loop over all linked handles
              h=obj.guihan(ihan);
              if h==0 || ~ishandle(h); continue; end
              if isprop(h,'ValueChangedFcn')
                set(h,'ValueChangedFcn',@obj.guiCallback);
              end
            end
          otherwise
            error('Unsupported mode: use "r" or "rw"');
        end
      end
      obj.mode=mode;
    end
    function SetPolltime(obj,polltime)
      %SETPOLLTIME Change update rate of timer polling monitored PVs
      
      % If in list context, then use first PV object to control timer
      if ~isempty(obj(1).utimer)
        obj(1).utimer.Period=double(round(polltime*1000))/1000;
      end
      obj(1).polltime = polltime ;
    end
    function run(obj,asyn,polltime,evobj,evname)
      %RUN Start update timer for monitored PVs
      % run(asyn,polltime) Starts timer to update monitored PVs at obj.polltime
      %   intervals, poll PVs at given period (polltime) [s]
      %   if asyn==true, use asynchrous gets
      % run(asyn,polltime,evobj,evname) Notify provided event object (list) and event
      %   name if any of the monitored lists of PVs updates
      
      % If there is already a timer, stop it first
      if ~isempty(obj(1).utimer)
        obj(1).utimer.stop;
      end
      
      % If in list context, then use first PV object to control timer
      if exist('evobj','var') && exist('evname','var')
        obj(1).utimer=timer('Period',double(round(obj(1).polltime*1000))/1000,...
          'ExecutionMode','fixedRate','TimerFcn',{@obj.utimerRun,asyn,evobj,evname},'ErrorFcn',@obj.utimerErr);
      else
        obj(1).utimer=timer('Period',double(round(obj(1).polltime*1000))/1000,...
          'ExecutionMode','fixedRate','TimerFcn',{@obj.utimerRun,asyn},'ErrorFcn',@obj.utimerErr);
      end
      obj(1).SetPolltime(polltime);
      obj(1).utimer.StartDelay=obj.RunStartDelay; % Give startup scripts chance to finish running
      obj(1).utimer.start;
    end
    function stop(obj)
      %STOP stop update timer running
      obj(1).utimer.stop;
      waitfor(obj(1).utimer.Running,'off');
    end
    function [newval,updt]=caget(obj,asyn)
      %CAGET Fetch control system values for this PV
      % [newval,updated] = caget(obj)
      %  new value inserted into val property and returned to newval. If
      %  no new value available (on monitored PVs), then last value is
      %  returned. update variable indicates if value has changed or not.
      % [newval,updated] = caget(obj,true)
      %  Use asynchronous get command: new values are not immediately fetched.
      %  Subsequent calls to caget will update new variables as they become
      %  available. Designed to be used in a polling loop. updt flag only returns true
      %  when data actually fetched in asynchronous context
      
      if ~exist('asyn','var') % default to synchronous gets
        asyn=false;
      end
      
      % If passing a vector of PV objects, loop through and get each
      updt=false(1,length(obj));
      if length(obj)>1
        newval=cell(1,length(obj));
        for iobj=1:length(obj)
          [newval{iobj},updt(iobj)]=obj(iobj).caget(asyn);
        end
        return
      else
        newval=obj.val;
        if length(newval)==1
          newval=newval{1};
        end
      end
      
      % If EPICS type, then must have provided context link
      if isempty(obj.context) && obj.type==PVtype.EPICS
        error('No context object provided, caget/caput will not work')
      end
      
      % force update for the first call and link to previous values
      lastval=obj.val;
      
      % currently only supporting EPICS, also skip if debug flag set >1
      if obj.type~=PVtype.EPICS
        warning('Unsupported controls type, skipping caget %s',obj.name);
        return
      elseif obj.debug>1
        warning('Debug set>1, skipping caget %s',obj.name);
        return
      end
      
      % Create channel and Register monitor if not already done so
      obj.genchannel();
      
      % Now do the fetching, and perform any value conversions
      updt=false;
      % Initialize future cell array if not performed any asynchronous tasks yet
      if asyn && isempty(obj.future)
        obj.future=cell(1,length(obj.pvname)) ;
        obj.future_tic=zeros(1,length(obj.pvname));
      end
      for ipv=1:length(obj.pvname)
        if islogical(obj.channel{ipv}) && ~obj.channel{ipv}
          error('Channel cleared (with Cleanup method?) for PV: %s',obj.pvname(ipv));
        end
        if asyn && isempty(obj.future{ipv}) % Launch asynchronous get thrread
          obj.future{ipv} = obj.channel{ipv}.getAsync();
          obj.future_tic(ipv) = tic ;
          return
        elseif asyn && ~isempty(obj.future{ipv}) % Get data from previously lanched asyn thread if available
          if obj.future{ipv}.isDone() % fetch data if asyn thread returned
            cavals = obj.future{ipv}.get() ;
            obj.future{ipv} = [] ;
          elseif toc(obj.future_tic(ipv))>obj.timeout % warn and skip if asyn command times out
            warning('Timeout waiting for PV: %s, skipping...',obj.pvname(ipv));
            obj.future{ipv}=[];
            return
          else % just skip this caget if still waiting for asyn thread and hasn't yet reached timeout
            return
          end
        else
          cavals = obj.channel{ipv}.get();
        end
        if ~isempty(obj.nmax) && length(cavals)>obj.nmax % default to getting everything
          cavals=cavals(1:obj.nmax);
        end
        if ~isempty(obj.conv) && length(obj.conv)==1 && isnumeric(cavals)
          cavals = cavals .* obj.conv ;
        elseif ~isempty(obj.conv) && length(obj.conv)>1 && isnumeric(cavals)
          cavals = polyval(obj.conv,cavals) ;
        elseif iscell(cavals) && length(cavals)==1
          cavals=cavals{1};
        end
        if length(obj.val)<ipv || ~isequal(cavals,obj.val{ipv})
          updt=true;
        end
        obj.val{ipv}=cavals;
      end
      if length(obj.val)==1
        newval = cavals ;
      else
        newval = obj.val ;
      end
      if ~updt; return; end % If nothing changed, nothing else to do
      
      % Notify listeners that PV value has changed
      notify(obj,'PVStateChange');
      
      % Update GUI links if live and values have changed
      nvals=length(obj.val);
      lims=obj.limits;
      pvl=obj.pvlogic;
      for ihan=1:length(obj.guihan) % Loop over all linked handles
        h=obj.guihan(ihan);
        if h==0 || ~ishandle(h); continue; end
        if isequal(lastval,obj.val) && isequal(lims,obj.lastlims); continue; end % loop if values haven't changed
        switch class(h)
          case 'matlab.ui.control.Lamp' % set lamp to green if all associated PVs evaluate to true ('ON', 'YES', or >0), else red
            negate=false;
            if pvl(1)=='~'
              negate=true;
              if length(pvl)>1; pvl=pvl(2:end); end
            end
            on=nan(1,nvals);
            for ival=1:nvals
              if strcmpi(obj.val{ival},'ON') || strcmpi(obj.val{ival},'YES') || strcmpi(obj.val{ival},'IN') || (isnumeric(obj.val{ival}) && obj.val{ival}>0)
                on(ival)=1;
              elseif strcmpi(obj.val{ival},'OFF') || strcmpi(obj.val{ival},'NO') || strcmpi(obj.val{ival},'OUT') || ~(isnumeric(obj.val{ival}) && obj.val{ival}>0)
                on(ival)=0;
              end
            end
            if any(isnan(on))
              h.Color=obj.guiprefs.guiLampCol{3};
            else
              if nvals>1
                for ival=2:nvals
                  if strcmp(pvl,'|')
                    on(1) = on(1) | on(ival) ;
                  elseif strcmp(pvl,'XOR')
                    on(1) = xor(on(1),on(ival)) ;
                  else
                    on(1) = on(1) & on(ival) ;
                  end
                end
              end
            end
            if negate; on=~on; end
            if on(1)
              h.Color=obj.guiprefs.guiLampCol{2};
            else
              h.Color=obj.guiprefs.guiLampCol{1};
            end
          case {'matlab.ui.control.ToggleSwitch','matlab.ui.control.Switch','matlab.ui.control.RockerSwitch'} % set lower toggle value if 'OFF' or 0 or 'NO'
            negate=false;
            if pvl(1)=='~'
              negate=true;
              if length(pvl)>1; pvl=pvl(2:end); end
            end
            on=false(1,nvals);
            for ival=1:nvals
              if strcmpi(obj.val{ival},'ON') || strcmpi(obj.val{ival},'YES') || strcmpi(obj.val{ival},'IN') || (isnumeric(obj.val{ival}) && obj.val{ival}>0)
                on(ival)=true;
              end
            end
            if negate; on=~on; end
            if nvals>1
              for ival=2:nvals
                if strcmp(pvl,'|')
                  on(1) = on(1) | on(ival) ;
                elseif strcmp(pvl,'XOR')
                  on(1) = xor(on(1),on(ival)) ;
                else
                  on(1) = on(1) & on(ival) ;
                end
              end
            end
            if isempty(h.ItemsData)
              if on(1)
                h.Value=h.Items{2};
              else
                h.Value=h.Items{1};
              end
            else
              if on(1)
                h.Value=h.ItemsData{2};
              else
                h.Value=h.ItemsData{1};
              end
            end
          case 'matlab.ui.control.NumericEditField' % Write PV value to edit field
            % If limits have been changed, update GUI field limits also
            % only apply limits to editable fields
            if strcmp(h.Editable,'on') && ~isempty(lims) && ~isequal(lims,obj.lastlims)
              h.Limits=double(lims);
            end
            % Deal with multiple PV channels linked to this object
            if length(obj.val)>1
              if strcmpi(obj.pvlogic,'MAX')
                hval=-inf;
              elseif strcmpi(obj.pvlogic,'MIN')
                hval=inf;
              else
                hval=0;
              end
              for ival=1:length(obj.val)
                if strcmpi(obj.pvlogic,'MAX') && obj.val{ival}>hval
                  hval=obj.val{ival};
                elseif strcmpi(obj.pvlogic,'MIN') && obj.val{ival}<hval
                  hval=obj.val{ival};
                elseif strcmpi(obj.pvlogic,'SUM')
                  hval=hval+obj.val{ival};
                else
                  hval=obj.val{1};
                end
              end
            else
              hval=obj.val{1};
            end
            h.Value=hval;
            if ~isempty(lims)
              if hval<lims(1) || hval>lims(2)
                h.BackgroundColor=obj.errcol;
              elseif h.Editable
                h.BackgroundColor=[1 1 1];
              else
                h.BackgroundColor=[0 0 0];
              end
            end
          case {'matlab.ui.control.LinearGauge','matlab.ui.control.Gauge','matlab.ui.control.NinetyDegreeGauge','matlab.ui.control.SemicircularGauge'}
            % If limits have been changed, update GUI field limits and coloring also
            if ~isempty(lims) && ~isequal(lims,obj.lastlims)
              ext=obj.guiprefs.gaugeLimitExtension;
              rng=range(lims);
              scalelims=[lims(1)-rng*ext(1),lims(1);lims(1),lims(2);lims(2),lims(2)+rng*ext(2)];
              h.Limits=double([min(scalelims(:)),max(scalelims(:))]);
              h.ScaleColors=obj.guiprefs.gaugeCol;
              h.ScaleColorLimits=double(scalelims);
            end
            h.Value=obj.val{1};
            if ~isempty(lims)
              if obj.val{1}<lims(1) || obj.val{1}>lims(2)
                h.BackgroundColor=obj.errcol;
              else
                h.BackgroundColor=[1 1 1];
              end
            end
          case 'matlab.ui.control.StateButton'
            on=false(1,nvals);
            negate=false;
            if pvl(1)=='~'
              negate=true;
              if length(pvl)>1; pvl=pvl(2:end); end
            end
            for ival=1:nvals
              if strcmpi(obj.val{ival},'ON') || strcmpi(obj.val{ival},'YES') || strcmpi(obj.val{ival},'IN') || (isnumeric(obj.val{ival}) && obj.val{ival}>0)
                on(ival)=true;
              end
            end
            if negate; on=~on; end
            if nvals>1
              for ival=2:nvals
                if strcmp(pvl,'|')
                  on(1) = on(1) | on(ival) ;
                elseif strcmp(pvl,'XOR')
                  on(1) = xor(on(1),on(ival)) ;
                else
                  on(1) = on(1) & on(ival) ;
                end
              end
            end
            if on(1)
              h.Value=true;
            else
              h.Value=false;
            end
        end
      end
      obj.lastlims=obj.limits;
      obj.lastnm=obj.nmax;
      obj.lastconv=obj.conv;
    end
    function stat=caput(obj,val)
      %CAPUT Set control system values for this PV
      % caput(PV,val)
      % PV is singleton object or vector of objects
      % val can be numeric or char vector or singleton or cell array
      %  if cell array, must match length of pvname vector for singleton PV
      %  or length of PV vector.
      % stat=0 on success
      
      % If EPICS type, then must have provided context link
      if isempty(obj.context) && obj.type==PVtype.EPICS
        error('No context object provided, caget/caput will not work')
      end
      
      % If passing a vector of PV objects, loop through and put each with
      % same val or pass each val to each PV if a cell array
      if length(obj)>1
        for iobj=1:length(obj)
          if iscell(val)
            obj(iobj).caput(val{iobj});
          else
            obj(iobj).caput(val);
          end
        end
        return
      end
      
      % Create channel and Register monitor if not already done so
      obj.genchannel();
      
      % Perform control system write operation or write operation to
      % command line if debug>0
      stat=0;
      if obj.debug==0
        for ipv=1:length(obj.pvname)
          if iscell(val)
            putval=val{ipv};
          else
            putval=val;
          end
          if ~strcmp(class(putval),obj.nativeclass{ipv}) % cast to a class that java client expecting
            putval=cast(putval,obj.nativeclass{ipv});
          end
          if obj.type==PVtype.EPICS
            if islogical(obj.channel{ipv}) && ~obj.channel{ipv}
              error('Channel cleared (with Cleanup method?) for PV: %s',obj.pvname(ipv));
            end
            if obj.putwait
              obj.channel{ipv}.put(putval(:)) ;
            else
              obj.channel{ipv}.putNoWait(putval(:)) ;
            end
          end
        end
      else
        fprintf(obj.STDOUT,'!DEBUG>0: caput %s %s\n',obj.pvname,num2str(val));
      end
      
    end
    function guiCallback(obj,src,~,~)
      
      switch src.Type
        case {'uitoggleswitch','uiswitch','uirockerswitch'}
          caput(obj,src.Value);
%           if (~isempty(src.ItemsData) && isequal(src.Value,src.ItemsData{1})) || ...
%             isequal(src.Value,src.Items{1}) % OFF state
%             caput(obj,0);
%           else % ON state
%             caput(obj,1);
%           end
        case 'uinumericeditfield'
          val=double(src.Value);
          if ~isempty(obj.limits)
            if val<obj.limits(1) || val>obj.limits(2)
              fprintf(obj.STDERR,'Trying to write value outside of set limits, aborting...\n');
              return
            end
          end
          caput(obj,val);
        otherwise
          fprintf(obj.STDERR,'Unsupported PV write operations from this GUI object: %s\n',src.Type);
      end
    end
    function pvstruct = struct(obj)
      %STRUCT Generate a structure array from object list
      pvstruct = builtin('struct',[]);
      names=[obj.name];
      % Names must be unique, if not append incremental number
      uname=unique(names);
      if length(uname)~=length(names)
        for iname=1:length(names)
          nnam=find(ismember(names,names(iname)));
          if length(nnam)>1
            nc=1;
            for iname2=nnam(:)'
              names(iname2)=names(iname2)+"_"+nc;
            end
          end
        end
      end
      for iname=1:length(names)
        pvstruct(1).(names(iname)) = obj(iname) ;
      end
    end
    function disp(obj)
      if length(obj)>1
        pvt=table(obj);
        display(pvt(:,{'ID','VarNames','pvname','monitor','units','limits','val'}));
      else
        builtin('disp',obj);
      end
    end
    function pvtab = table(obj)
      %TABLE Generate a table object from object list
      pvname={obj.pvname}';
      monitor=[obj.monitor]';
      val={obj.val}';
      for ival=1:length(val) % try and convert everything to simple numeric array, the rest stay as cell vectors
        try
          val{ival}=cell2mat(val{ival});
        catch
        end
      end
      type=[obj.type]';
      debug=[obj.debug]';
      units=[obj.units]';
      conv={obj.conv}';
      guihan={obj.guihan}';
      limits=zeros(length(obj),2);
      for ilimit=1:length(obj)
        if isempty(obj(ilimit).limits)
          limits(ilimit,:)=[-inf inf];
        else
          limits(ilimit,:)=obj(ilimit).limits;
        end
      end
      pvlogic={obj.pvlogic}';
      nmax=inf(length(obj),1);
      nmax(cellfun(@(x) ~isempty(x),{obj.nmax}))=[obj.nmax];
      for imax=1:length(obj)
        if isempty(obj(imax).nmax)
          nmax(imax)=inf;
        end
      end
      ID=1:length(pvname); ID=ID(:);
      pvtab = table(pvname,monitor,val,type,debug,units,conv,guihan,limits,pvlogic,nmax,ID) ;
      names=[obj.name];
      % Names must be unique, if not append incremental number
      uname=unique(names);
      if length(uname)~=length(names)
        for iname=1:length(names)
          nnam=find(ismember(names,names(iname)));
          if length(nnam)>1
            nc=1;
            for iname2=nnam(:)'
              names(iname2)=names(iname2)+"_"+nc;
            end
          end
        end
      end
      pvtab.VarNames = cellstr(names)';
    end
    function pset(obj,propname,val)
      % If passing a vector of PV objects, loop through and set each
      if length(val)==1 && length(obj)>1
        for iobj=1:length(obj)
          obj(iobj).(propname)=val ;
        end
      elseif length(val)==length(obj)
        for iobj=1:length(obj)
          obj(iobj).(propname)=val(iobj) ;
        end
      end
    end
    function Cleanup(obj)
      %CLEANUP Perform cleanup actions (call prior to exiting)
      switch obj(1).type
        case PVtype.EPICS
          cntxt=[];
          for iobj=1:length(obj)
            if ~isempty(obj(iobj).channel)
              for ichan=1:length(obj(iobj).channel)
                fprintf('Closing PV channel for: %s\n',obj(iobj).pvname(ichan));
                obj(iobj).channel{ichan}.close();
                obj(iobj).channel{ichan}=false;
              end
            end
            if isempty(cntxt) && ~isempty(obj(iobj).context)
              cntxt=obj(iobj).context;
            end
          end
          if ~isempty(cntxt)
            fprintf('Closing EPICS context...\n');
            cntxt.close();
          end
          fprintf('PV Cleanup Complete.\n');
      end
    end
  end
  methods(Access=protected)
    function utimerErr(obj,~,~) % Actions to take if timer errors
      fprintf(obj(1).STDERR,'%s: timer service crashed, restarting\n',datetime);
      obj(1).utimer.start;
    end
    function utimerRun(obj,~,~,asyn,evobj,evname) % Update timer actions
      monipv=[obj.monitor];
      if ~any(monipv); return; end
      [~,ud]=caget(obj(monipv),asyn); % get all changed values, use asynchronous gets
      % notify event if requested and anything changed
      if any(ud) && exist('evobj','var') && exist('evname','var') && ~isempty(evobj)
        notify(evobj,evname);
      end
    end
    function genchannel(obj)
      %GENCHANNEL Generate a ca channel for referenced PVs
      if ~isempty(obj.channel) % only call this once per object
        return
      end
      if ~isempty(obj.pvdatatype) && ~iscell(obj.pvdatatype)
        obj.pvdatatype={obj.pvdatatype};
      end
      pvd=cell(1,length(obj.pvname));
      for ipv=1:length(obj.pvname)
        fprintf(obj.STDOUT,'Connecting to PV: %s\n',obj.pvname(ipv));
        if ~isempty(obj.pvdatatype)
          if iscell(obj.pvdatatype)
            obj.channel{ipv} = org.epics.ca.Channels.create(obj.context,org.epics.ca.ChannelDescriptor(char(obj.pvname(ipv)),obj.pvdatatype{ipv})) ;
          else
            obj.channel{ipv} = org.epics.ca.Channels.create(obj.context,org.epics.ca.ChannelDescriptor(char(obj.pvname(ipv)),obj.pvdatatype)) ;
          end
        else
          obj.channel{ipv} = org.epics.ca.Channels.create(obj.context,char(obj.pvname(ipv))) ;
        end
        obj.val{ipv} = obj.channel{ipv}.get() ;
        obj.nativeclass{ipv}=class(obj.val{ipv});
        if isempty(obj.val)
          error('Failed to connect to channel: %s',obj.pvname(ipv))
        end
        if isempty(obj.pvdatatype)
          pvd{ipv} = obj.channel{ipv}.getProperties.get('nativeType') ;
        end
        if obj.monitor
          obj.channel{ipv}.close();
          if isempty(obj.pvdatatype)
            obj.channel{ipv} = org.epics.ca.Channels.create(obj.context,org.epics.ca.ChannelDescriptor(char(obj.pvname(ipv)),pvd{ipv},true)) ;
          elseif ~isecell(obj.pvdatatype)
            obj.channel{ipv} = org.epics.ca.Channels.create(obj.context,org.epics.ca.ChannelDescriptor(char(obj.pvname(ipv)),obj.pvdatatype,true)) ;
          else
            obj.channel{ipv} = org.epics.ca.Channels.create(obj.context,org.epics.ca.ChannelDescriptor(char(obj.pvname(ipv)),obj.pvdatatype{ipv},true)) ;
          end
        end
%         val = obj.channel.get(org.epics.ca.data.Graphic().getClass()); % Get PV metadata
      end
      if isempty(obj.pvdatatype)
        obj.pvdatatype=pvd;
      end
      % Force Matlab data types for some native java types
      for ipv=1:length(obj.pvname)
        if ~iscell(obj.pvdatatype)
          pvt=obj.pvdatatype;
        else
          pvt=obj.pvdatatype{ipv};
        end
        if endsWith(string(pvt),'Integer')
          obj.nativeclass{ipv}='int32';
          obj.val{ipv}=int32(obj.val{ipv});
        elseif endsWith(string(pvt),'Short')
          obj.nativeclass{ipv}='int16';
          obj.val{ipv}=int32(obj.val{ipv});
        end
      end
    end
  end
  methods(Static)
    function context = Initialize(type)
      %INITIALIZE One-time initialization steps for channel access client
      % context = PV.Initialize(PVtype.EPICS)
      %  Perform one-time (per Matlab instance) EPICS initialization steps using java ca client
      %  context: object required for future channel creation, pass to PV objects in constructor
      switch type
        case PVtype.EPICS
          % Add ca files to java search path
          cadir=which("ca-"+PV.caver+".jar");
          if isempty(cadir)
            error('ca jar file not found on matlab search path');
          end
          d=dir(cadir);
          jarfiles=dir(fullfile(d.folder,'*.jar'));
          for ijar=1:length(jarfiles)
            javaaddpath(fullfile(jarfiles(ijar).folder,jarfiles(ijar).name));
          end
          % Generate context with default parameters
          context = org.epics.ca.Context ;
      end
    end
  end
end

function mustBeGoodLimits(limits)
  if ~isempty(limits) && ( length(limits)~=2 || limits(1)>=limits(2) )
    error('Value assigned is not a good array for setting limts, must be [a,b], where b>a');
  end
end