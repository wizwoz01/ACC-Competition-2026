try
    fprintf('=== QUARC / MATLAB Diagnostic ===\n');

    fprintf('MATLAB Version: %s\n', version);
    fprintf('MATLAB Release: %s\n', ['R' version('-release')]);

    % QUARC should appear in ver if installed.
    v = ver;
    quarcVer = '';
    for k = 1:numel(v)
        if contains(lower(v(k).Name), 'quarc')
            quarcVer = v(k).Version;
            fprintf('QUARC Toolbox: %s (Version %s)\n', v(k).Name, v(k).Version);
        end
    end
    if isempty(quarcVer)
        fprintf('WARNING: QUARC does not appear in the MATLAB "ver" list.\n');
    end

    fprintf('\n=== QUARC Root / Library Check ===\n');
    qDir = getenv('QUARC_DIR');
    if isempty(qDir)
        fprintf('Env Var QUARC_DIR: [NOT SET]\n');
    else
        fprintf('Env Var QUARC_DIR: %s\n', qDir);
    end

    libLoc = which('quarc_library');
    if isempty(libLoc)
        fprintf('quarc_library location: [NOT FOUND]\n');
        fprintf('If you can open the QUARC library in Simulink, restart MATLAB and re-run.\n');
        return;
    end
    fprintf('quarc_library location: %s\n', libLoc);

    libDir = fileparts(libLoc);            % ...\QUARC\library\Rxxxx
    quarcRoot = fileparts(fileparts(libDir)); % ...\QUARC
    fprintf('QUARC root inferred: %s\n', quarcRoot);

    installedLibs = dir(fullfile(quarcRoot, 'library', 'R*'));
    installedLibNames = {installedLibs([installedLibs.isdir]).name};
    installedLibNames = installedLibNames(~ismember(installedLibNames, {'.','..'}));
    fprintf('Installed QUARC Simulink libraries: ');
    if isempty(installedLibNames)
        fprintf('[NONE FOUND]\n');
    else
        fprintf('%s\n', strjoin(installedLibNames, ', '));
    end

    expectedLib = ['R' version('-release')];
    expectedLibDir = fullfile(quarcRoot, 'library', expectedLib);
    if ~exist(expectedLibDir, 'dir')
        fprintf('WARNING: No QUARC Simulink library for %s found at:\n  %s\n', expectedLib, expectedLibDir);
        fprintf('This mismatch often causes missing helper functions and board support issues.\n');
    else
        fprintf('OK: Found QUARC Simulink library for %s\n', expectedLib);
    end

    fprintf('\n=== HIL Initialize (Board Type) Check ===\n');
    sfunLoc = which('hil_initialize_block');
    if isempty(sfunLoc)
        fprintf('hil_initialize_block not found on MATLAB path.\n');
        fprintf('If you can add the HIL Initialize block, this likely means the MATLAB path is incomplete.\n');
    else
        fprintf('hil_initialize_block found at: %s\n', sfunLoc);
    end

    % Extract board_type popup options from the HIL Initialize mask.
    boardTypeOptions = {};
    try
        load_system('quarc_library');
        probeModel = '__quarc_hil_probe__';
        if bdIsLoaded(probeModel)
            close_system(probeModel, 0);
        end
        new_system(probeModel);

        % Try both known library locations.
        hilLibCandidates = {
            'quarc_library/Data Acquisition/Generic/Configuration/HIL Initialize',
            'quarc_library/HIL Initialize'
        };
        added = false;
        for i = 1:numel(hilLibCandidates)
            try
                add_block(hilLibCandidates{i}, [probeModel '/HIL_Initialize'], 'Position', [30 30 200 90]);
                added = true;
                break;
            catch
            end
        end

        if ~added
            fprintf('Could not add HIL Initialize block from QUARC library (library path mismatch).\n');
        else
            m = Simulink.Mask.get([probeModel '/HIL_Initialize']);
            if ~isempty(m)
                for p = 1:numel(m.Parameters)
                    if strcmpi(m.Parameters(p).Name, 'board_type') && strcmpi(m.Parameters(p).Type, 'popup')
                        boardTypeOptions = m.Parameters(p).TypeOptions;
                        break;
                    end
                end
            end

            if isempty(boardTypeOptions)
                fprintf('Could not read board_type popup options from mask.\n');
            else
                fprintf('HIL Initialize board_type options include %d entries.\n', numel(boardTypeOptions));
                if any(strcmpi(boardTypeOptions, 'qcar2'))
                    fprintf('OK: "qcar2" is present in the mask dropdown.\n');
                else
                    fprintf('FAIL: "qcar2" is NOT present in the mask dropdown.\n');
                end
            end
        end

        close_system(probeModel, 0);
    catch ME
        fprintf('Error probing HIL Initialize mask: %s\n', ME.message);
        try, close_system('__quarc_hil_probe__', 0); catch, end
    end

    fprintf('\n=== Why Your Simulink Compile Fails ===\n');
    fprintf('Your error comes from the HIL Initialize S-function at compile time:\n');
    fprintf('  "Support for the given board type does not appear to be installed"\n');
    fprintf('That means MATLAB can load QUARC blocks, but the QUARC HIL runtime on this PC\n');
    fprintf('does not have a usable board definition/driver for the board type you set (qcar2).\n');
    fprintf('\nMost common causes:\n');
    fprintf('  1) QUARC Simulink library mismatch vs MATLAB release (you are currently using %s).\n', libDir);
    fprintf('  2) QCar2 board support package not installed/enabled in QUARC.\n');
    fprintf('  3) QUARC MATLAB registration was not run for your MATLAB version.\n');
    fprintf('\nFix checklist (recommended):\n');
    fprintf('  - Run: C:\\Program Files\\Quanser\\QUARC\\quarc_matlab_registration.exe\n');
    fprintf('    and register for MATLAB %s\n', ['R' version('-release')]);
    fprintf('  - Reinstall/modify QUARC to include QCar2 support (board type qcar2).\n');
    fprintf('  - After that, the QUARC library folder should include %s under ...\\QUARC\\library\\\n', expectedLib);

catch ME
    fprintf('Critical Error: %s\n', ME.message);
end
