function BuildDrivers(generateSimulinkBlocks)
    %mpsv.BuildDrivers Build or rebuild the driver blocks for the MPSV toolbox.
    % 
    % PARAMETERS
    % generateSimulinkBlocks ... True if simulink blocks should be generated in a new simulink model. Default value is false.
    % 
    % DETAILS
    % This MATLAB function generates all S-functions and compiles the corresponding mex binaries for the Simulink library.

    arguments
        generateSimulinkBlocks (1,1) logical = false
    end

    % print banner
    fprintf('\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n');
    fprintf('  __  __ ___  _____   __\n');
    fprintf(' |  \\/  | _ \\/ __\\ \\ / /\n');
    fprintf(' | |\\/| |  _/\\__ \\\\ V / \n');
    fprintf(' |_|  |_|_|  |___/ \\_/  \n\n');
    fprintf(' Motion Planner for Surface Vehicles\n');
    fprintf(' %s\n', char(strjoin(string(mpsv.GetVersion()),'.')));
    fprintf('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n');

    % get absolute paths
    thisDirectory = extractBefore(mfilename('fullpath'),strlength(mfilename('fullpath')) - strlength(mfilename) + 1);
    folderSimulinkLibrarySource = fullfile(thisDirectory, '..', '..', 'library', 'source');
    folderMPSVSource = fullfile(thisDirectory, '..', '..', '..', 'source');

    % navigate to library source directory
    currentWorkingDirectory = pwd();
    cd(folderSimulinkLibrarySource);

    % update library-internal code directory (contains a copy of the MPSV header-only library with modified filenames and include paths because code generation does not work with code subfolders)
    fprintf('update code directory: ');
    UpdateCodeDirectory(folderSimulinkLibrarySource, folderMPSVSource);
    fprintf('done\n');

    % generate specifications for all drivers
    defs = [];
    
    
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Driver: MPSV Path Planner
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    fprintf('define driver: MPSVPathPlanner\n');
    def = legacy_code('initialize');
    def.SFunctionName           = 'SFunctionMPSVPathPlanner';
    def.StartFcnSpec            = 'void MPSV_PathPlannerInitialize(void** work1, int16 p1, uint32 p2)';
    def.TerminateFcnSpec        = 'void MPSV_PathPlannerTerminate(void* work1)';
    def.OutputFcnSpec           = 'void MPSV_PathPlannerStep(void* work1, uint8 y1[24023], uint8 u1[65153])';
    def.HeaderFiles             = {'MPSV_DriverPathPlanner.hpp'};
    def.SourceFiles             = {'MPSV_DriverPathPlanner.cpp','MPSV_WrapperPathPlanner.cpp'};
    def.IncPaths                = {'code'};
    def.SrcPaths                = {'code'};
    def.LibPaths                = {''};
    def.HostLibFiles            = {};
    def.Options.language        = 'C++';
    def.Options.useTlcWithAccel = false;
    def.SampleTime              = 'inherited';
    defs = [defs; def];
    
    
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Driver: MPSV Asynchronous Online Planner
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    fprintf('define driver: MPSVAsynchronousOnlinePlanner\n');
    def = legacy_code('initialize');
    def.SFunctionName           = 'SFunctionMPSVAsynchronousOnlinePlanner';
    def.StartFcnSpec            = 'void MPSV_AsynchronousOnlinePlannerInitialize(void** work1, int16 p1, uint32 p2, int16 p3, uint32 p4, int32 p5, int32 p6, int32 p7, int32 p8[], uint32 p9)';
    def.TerminateFcnSpec        = 'void MPSV_AsynchronousOnlinePlannerTerminate(void* work1)';
    def.OutputFcnSpec           = 'void MPSV_AsynchronousOnlinePlannerStep(void* work1, uint8 y1[52905], uint8 u1[64193], uint8 u2[1998])';
    def.HeaderFiles             = {'MPSV_DriverAsynchronousOnlinePlanner.hpp'};
    def.SourceFiles             = {'MPSV_DriverAsynchronousOnlinePlanner.cpp','MPSV_WrapperAsynchronousOnlinePlanner.cpp'};
    def.IncPaths                = {'code'};
    def.SrcPaths                = {'code'};
    def.LibPaths                = {''};
    def.HostLibFiles            = {};
    def.Options.language        = 'C++';
    def.Options.useTlcWithAccel = false;
    def.SampleTime              = 'inherited';
    defs = [defs; def];
    
    
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Compile and generate all required files
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % generate SFunctions
    fprintf('generate S-functions: ');
    legacy_code('sfcn_cmex_generate', defs);
    fprintf('done\n');

    % compile
    cflags    = '-fopenmp -mtune=native';
    cxxflags  = '-fopenmp -mtune=native -std=c++20';
    ldflags   = '-fopenmp -mtune=native -std=c++20';
    libraries = {'-L/usr/lib','-L/usr/local/lib','-lstdc++','-lpthread','-lgomp'};
    legacy_code('compile', defs, [{['CFLAGS=$CFLAGS ',cflags],['CXXFLAGS=$CXXFLAGS ',cxxflags],['LINKFLAGS=$LINKFLAGS ',ldflags]},libraries]);

    % generate TLC
    fprintf('\ngenerate TLC: ');
    legacy_code('sfcn_tlc_generate', defs);
    fprintf('done\n');

    % generate RTWMAKECFG
    fprintf('generate rtwmakecfg: ');
    legacy_code('rtwmakecfg_generate', defs);
    fprintf('done\n');

    % generate Simulink blocks
    if(generateSimulinkBlocks)
        fprintf('generate simulink blocks: ');
        legacy_code('slblock_generate', defs);
        fprintf('done\n');
    end

    % navigate back to current path
    cd(currentWorkingDirectory);

    % print footer
    fprintf('\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n');
    fprintf(' MPSV DRIVER BUILD COMPLETED\n');
    fprintf('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n');
end

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% PRIVATE HELPER FUNCTIONS
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function UpdateCodeDirectory(folderSimulinkLibrarySource, folderMPSVSource)
    % set input and output directory (input: original MPSV source code, output: modified source code for simulink driver)
    info = what(folderMPSVSource);
    if(~isempty(info)), folderMPSVSource = info.path; end
    directoryInput = fullfile(folderMPSVSource,'mpsv');
    directoryOutput = fullfile(folderSimulinkLibrarySource,'code');

    % re-create output directory
    [~,~] = rmdir(directoryOutput, 's');
    [~,~] = mkdir(directoryOutput);

    % search for all header files in the input directory (it's a header-only library with .hpp files only)
    listings = dir(fullfile(directoryInput,['**',filesep,'*.hpp']));
    listings = listings(~[listings.isdir]);
    for i = 1:numel(listings)
        [inputFile, outputFile] = GetFilenames(listings(i), folderMPSVSource, directoryOutput);
        ConvertFiles(inputFile, outputFile);
    end
end

function [inputFile, outputFile] = GetFilenames(listing, directorySource, directoryOutput)
    inputFile = fullfile(listing.folder,listing.name);
    s = erase(listing.folder, directorySource);
    s = replace(s, {'/','\'}, '_');
    if(startsWith(s,'_'))
        s(1) = [];
    end
    outputFile = fullfile(directoryOutput,strcat(s,'_',listing.name));
end

function ConvertFiles(inputFile, outputFile)
    % read input file
    content = fileread(inputFile);

    % modify the content, e.g. adjust include paths
    lines = splitlines(content);
    for n = 1:numel(lines)
        if(startsWith(lines{n},'#include <mpsv/') && endsWith(lines{n},'.hpp>') && ~contains(lines{n},'//') && ~contains(lines{n},'/*') && ~contains(lines{n},'*/'))
            lines{n} = replace(lines{n},'/','_');
        end
    end
    content = join(lines,newline);
    s = content{1};

    % write to output file
    fileID = fopen(outputFile,'w');
    if(-1 == fileID)
        error('Could not write to file "%s"', outputFile);
    end
    fwrite(fileID, uint8(s));
    fclose(fileID);
end

