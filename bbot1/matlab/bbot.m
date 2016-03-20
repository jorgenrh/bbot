function bbot(comPort)
% BalanceBot GUI
% https://github.com/jorgenrh/bbot

timerInterval =  0.001;
baudRate = 9600;
defaultPort = '/dev/tty.usbmodem16901';
   

    
%% Script
%disp('/dev/tty.usbmodem16901')
%disp('/dev/tty.HC-06-DevB')
%disp('/dev/tty.usbserial-A600bNHC')

if (nargin == 0)
    comPort = defaultPort;
end

com = serial(comPort);
tmr = timer();
gui = struct();
plt = struct();
runPlot = 0;
initGUI();
initPlot();

status = 0;
try
    status = serialConnect();
catch
    disp(['Error: Could not connect to ' comPort])
end

if (status)
    tic
    runPlot = 1;
    startTimer();
    serialWrite('Q', 0) % request PID values
    serialWrite('R', 0) % request offset value
else
    disp('Error: Serial port not connected')
end
    




    %% Timer functions
    function startTimer()
        tmr = timer('Name', 'bbotTimer', ...
                'ExecutionMode', 'singleShot', ...
                'BusyMode', 'queue', ...
                'Period', timerInterval, ...
                'ExecutionMode', 'fixedSpacing');
        tmr.TimerFcn = {@readTimer_cb};
        start(tmr)
    end

    function stopTimer()
        stop(tmr)
        delete(tmr)
        if (~isempty(timerfind))
            delete(timerfind);
        end
    end

    %% Serial functions

    function status = serialConnect()
        com = serial(comPort);
        set(com, 'DataBits', 8);
        set(com, 'StopBits', 1);
        set(com, 'BaudRate', baudRate);
        set(com, 'Parity', 'none');
        set(com, 'Terminator', 'LF');
        %set(com, 'ReadAsyncMode', 'manual');
        fopen(com);
        %readasync(com);
        
        if (strcmp(get(com, 'status'),'open'))
            status = 1;
            disp('Serial port open')
        else
            status = 0;
            disp('Error opening serial port')        
        end
    end

    function serialRead()
        while (com.BytesAvailable > 0) 
        
            %disp('got data')
            
            [line, count, msg] = fgetl(com);
            [data, num] = sscanf(line, '%c;%f;%f;%f;%f');
            %[data, num] = sscanf(line, '%c;%i;%i;%i;%i');
                        
            disp(line)
            
            if (num > 0)
                chr = char(data(1));
                switch chr
                    case 'G'
                        updatePlot(data(2:end))
                        %pause(timerInterval) % smoother?
                    case 'Q'
                        updatePID(data(2:end))
                    case 'O'
                        updateOffset(data(2:end))
                    otherwise
                        %disp(['Unknown command: ' chr])
                        disp(data)
                        %disp(chr(data))
                end
            end
            
        end
    end

    function serialWrite(cmd, data)

        %while (~strcmp(get(com, 'TransferStatus'), 'idle'))
            % wait for iiit, waai...
        %end
        
        str = sprintf('%s', cmd);
        if (data ~= 0)
            str = sprintf('%c%s', [cmd, num2str(data)]);
        end
        %fprintf(com, '%s', str);
        %stopasync(com);
        fprintf(com, str); % default %s + terminator
        %disp('sending data')
    end

    function serialDisconnect()
        
        if (strcmp(get(com, 'status'), 'open'))
            fclose(com);
        end
        
        if (~isempty(instrfind))
            fclose(instrfind);
            delete(instrfind);
        end
    end

    %% GUI functions
    function exitFigure()
        stopTimer()
        serialDisconnect()
        delete(gui.h)
        clear plt gui tmr com runPlot;
        disp('GUI closed');
    end

    function updatePlot(data)

        %disp('updateplot')
        if (length(data) >= 2)
        
            sd = data(1);
            pd = data(2);

            now = toc;
            % add new data, drop first value
            plt.sdata = [plt.sdata(2:end); sd];
            plt.pdata = [plt.pdata(2:end); pd];
            plt.xtime = [plt.xtime(2:end); now];
            
            if (runPlot)
                set(gui.gxText, 'string', sd);
                set(gui.gpidText, 'string', pd);

                set(plt.splot, 'xdata', plt.xtime, 'ydata', plt.sdata);
                set(plt.pplot, 'xdata', plt.xtime, 'ydata', plt.pdata);
                set(gui.ax, 'xlim', [plt.xtime(1) now]);
                drawnow;
            end
        end    
        
    end

    function updatePID(data)
        
       if (length(data) >= 3)
           set(gui.pidPslide, 'value', data(1));
           set(gui.pidPedit, 'string', num2str(data(1)));
           set(gui.pidIslide, 'value', data(2));
           set(gui.pidIedit, 'string', num2str(data(2)));
           set(gui.pidDslide, 'value', data(3));
           set(gui.pidDedit, 'string', num2str(data(3)));           
       end         
       
    end

    function updateOffset(data)
        
        if (length(data) >= 1)
           set(gui.offsetSlide, 'value', data(1))
           set(gui.offsetEdit, 'string', num2str(data(1)));
        end
        
    end

    function initGUI()
        gui.screen = get(0, 'screensize');
        gui.h = figure('numbertitle', 'off',...
                    'menubar', 'none', ...
                    'units', 'pixels', ...
                    'position', [gui.screen(3)/2-300, gui.screen(4)/2-200, 800, 440], ...
                    'name', 'BalanceBot', ...
                    'resize', 'off', ...
                    'CloseRequestFcn', {@exitFig_cb});
        gui.ax = axes('box', 'on', ...
                    'units', 'pixels', ...
                    'position', [40 80 720 300]);
        gui.disconnectBtn = uicontrol('style', 'pushbutton', 'string', 'Disconnect', ...
                    'pos', [40 390 100 30], 'parent', gui.h, ...
                    'callback', {@disconnectBtn_cb}, 'fontsize', 12);
        gui.gridBtn = uicontrol('style', 'pushbutton', 'string', 'Grid', ...
                    'pos', [500 390 80 30], 'parent', gui.h, ...
                    'callback', {@gridBtn_cb}, 'fontsize', 12);
        gui.pauseBtn = uicontrol('style', 'togglebutton', 'string', 'Pause', ...
                    'pos', [590 390 80 30], 'parent', gui.h, ...
                    'callback', {@pauseBtn_cb}, 'value', 0, 'fontsize', 12);
        gui.motorBtn = uicontrol('style', 'pushbutton', 'string', 'Motor on/off', ...
                    'pos', [680 390 80 30], 'parent', gui.h, ...
                    'callback', {@motorBtn_cb}, 'value', 0, 'fontsize', 12);
        gui.gxText = uicontrol('style','edit', 'parent', gui.h,...
                    'position',[425 395 60 20],...
                    'string', '--', 'fontsize', 14);
        gui.gpidText = uicontrol('style','edit', 'parent', gui.h,...
                    'position',[360 395 60 20],...
                    'string', '--', 'fontsize', 14);
        gui.pidPslide = uicontrol('style','slide', 'parent', gui.h, ...
                    'position', [40 20 80 20], 'min', -50,'max', 50, 'val', 2);                
        gui.pidPedit = uicontrol('style','edit', 'parent', gui.h, ...
                    'position', [40 40 80 18], 'string', 'P');                
        gui.pidIslide = uicontrol('style','slide', 'parent', gui.h, ...
                    'position', [130 20 80 20], 'min', -80,'max', 80, 'val', 2);                
        gui.pidIedit = uicontrol('style','edit', 'parent', gui.h, ...
                    'position', [130 40 80 18], 'string', 'I');                
        gui.pidDslide = uicontrol('style','slide', 'parent', gui.h, ...
                    'position', [220 20 80 20], 'min', -80,'max', 80, 'val', 2);                
        gui.pidDedit = uicontrol('style','edit', 'parent', gui.h, ...
                    'position', [220 40 80 18], 'string', 'D');                
        set([gui.pidPslide, gui.pidPedit], 'call', {@pidP_callback});
        set([gui.pidIslide, gui.pidIedit], 'call', {@pidI_callback});
        set([gui.pidDslide, gui.pidDedit], 'call', {@pidD_callback});
        gui.requestPIDBtn = uicontrol('style', 'pushbutton', 'string', 'Get PID', ...
                    'pos', [310 25 80 30], 'parent', gui.h, ...
                    'callback', {@requestPID_cb}, 'fontsize', 12);
        gui.calibrateBtn = uicontrol('style', 'pushbutton', 'string', 'Calibrate', ...
                    'pos', [680 25 80 30], 'parent', gui.h, ...
                    'callback', {@calibrate_cb}, 'fontsize', 12);
        gui.offsetSlide = uicontrol('style','slide', 'parent', gui.h, ...
                    'position', [500 20 80 20], 'min', -30,'max', 30, 'val', 2);                
        gui.offsetEdit = uicontrol('style','edit', 'parent', gui.h, ...
                    'position', [500 40 80 18], 'string', 'offset'); 
        set([gui.offsetSlide, gui.offsetEdit], 'call', {@setOffset_cb});
        gui.requestOffsetBtn = uicontrol('style', 'pushbutton', 'string', 'Get Offset', ...
                    'pos', [590 25 80 30], 'parent', gui.h, ...
                    'callback', {@requestOffset_cb}, 'fontsize', 12);

    end

    function initPlot()
        plt.buflen = 100;
        plt.sdata = zeros(plt.buflen, 1);
        plt.pdata = zeros(plt.buflen, 1);
        plt.xtime = zeros(plt.buflen, 1);

        hold on
        plt.splot = plot(plt.xtime, plt.sdata, 'r');
        plt.pplot = plot(plt.xtime, plt.pdata, 'b');
        set(gui.ax, 'ylim', [-270 270]);
        drawnow;
    end

    %% GUI Callbacks
    
    function disconnectBtn_cb(h, event)
        exitFigure();
    end

    function gridBtn_cb(h, event)
        grid
    end

    function pauseBtn_cb(h, event)
        runPlot = 1-runPlot;
    end

    function motorBtn_cb(h, event)
        serialWrite('M', 0)
    end
    
    function calibrate_cb(h, event)
        serialWrite('C', 0)
    end

    function requestPID_cb(h, event)
        serialWrite('Q', 0)
    end
    
    function exitFig_cb(h, event)
        exitFigure();
    end

    function readTimer_cb(t, event, woot)
        serialWrite('G', 0)
        serialRead()        
    end

    function pidP_callback(h, event)
        switch h  % Who called?
            case gui.pidPedit
                L = get(gui.pidPslide, {'min','max','value'});  % Get the slider's info.
                E = str2double(get(h, 'string'));  % Numerical edit string.
                if E >= L{1} && E <= L{2}
                    set(gui.pidPslide, 'value', E)  % E falls within range of slider.
                else
                    set(h, 'string', L{3}) % User tried to set slider out of range. 
                end
            case gui.pidPslide
                set(gui.pidPedit, 'string', get(h, 'value')) % Set edit to current slider.
            otherwise
            % Do nothing
        end
        serialWrite('P', get(gui.pidPslide, 'value'))
    end

    function pidI_callback(h, event)
        switch h  
            case gui.pidIedit
                L = get(gui.pidIslide, {'min','max','value'});  % Get the slider's info.
                E = str2double(get(h, 'string'));  % Numerical edit string.
                if E >= L{1} && E <= L{2}
                    set(gui.pidIslide, 'value', E)  % E falls within range of slider.
                else
                    set(h, 'string', L{3}) % User tried to set slider out of range. 
                end
            case gui.pidIslide
                set(gui.pidIedit, 'string', get(h, 'value')) % Set edit to current slider.
            otherwise
            % Do nothing
        end
        serialWrite('I', get(gui.pidIslide, 'value'))
    end

    function pidD_callback(h, event)
        switch h
            case gui.pidDedit
                L = get(gui.pidDslide, {'min','max','value'});  % Get the slider's info.
                E = str2double(get(h, 'string'));  % Numerical edit string.
                if E >= L{1} && E <= L{2}
                    set(gui.pidDslide, 'value', E)  % E falls within range of slider.
                else
                    set(h, 'string', L{3}) % User tried to set slider out of range. 
                end
            case gui.pidDslide
                set(gui.pidDedit, 'string', get(h, 'value')) % Set edit to current slider.
            otherwise
            % Do nothing
        end
        serialWrite('D', get(gui.pidDslide, 'value'))
    end

    function requestOffset_cb(h, event)
        serialWrite('R', 0)
    end

    function setOffset_cb(h, event)
       switch h
            case gui.offsetEdit
                L = get(gui.offsetEdit, {'min','max','value'});  % Get the slider's info.
                E = str2double(get(h, 'string'));  % Numerical edit string.
                if E >= L{1} && E <= L{2}
                    set(gui.offsetEdit, 'value', E)  % E falls within range of slider.
                else
                    set(h, 'string', L{3}) % User tried to set slider out of range. 
                end
            case gui.offsetSlide
                set(gui.offsetEdit, 'string', get(h, 'value')) % Set edit to current slider.
            otherwise
            % Do nothing
       end
       serialWrite('O', get(gui.offsetSlide, 'value'))    
    end

end