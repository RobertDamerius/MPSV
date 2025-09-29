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
    fprintf('\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n B U I L D   S I M U L I N K - D R I V E R S\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n');

    % get absolute paths
    thisDirectory = extractBefore(mfilename('fullpath'),strlength(mfilename('fullpath')) - strlength(mfilename) + 1);
    folderSimulinkLibrarySource = fullfile(thisDirectory, '..', '..', 'library', 'source');
    folderMPSVSource = fullfile(thisDirectory, '..', '..', '..', 'source');

    % navigate to library source directory
    currentWorkingDirectory = pwd();
    cd(folderSimulinkLibrarySource);

    % update library-internal code directory (contains a copy of the MPSV header-only library with modified filenames and include paths because code generation does not work with code subfolders)
    UpdateCodeDirectory(folderSimulinkLibrarySource, folderMPSVSource);

    % generate specifications for all drivers
    defs = [];
    
    
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Driver: MPSV Path Planner
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
    def.SampleTime = 'parameterized';
    defs = [defs; def];
    
    
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Driver: MPSV Asynchronous Online Planner
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def = legacy_code('initialize');
    def.SFunctionName           = 'SFunctionMPSVAsynchronousOnlinePlanner';
    def.StartFcnSpec            = 'void MPSV_AsynchronousOnlinePlannerInitialize(void** work1, int16 p1, uint32 p2, int16 p3, uint32 p4, int32 p5, int32 p6, int32 p7)';
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
    def.SampleTime = 'parameterized';
    defs = [defs; def];
    
    
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Compile and generate all required files
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % generate SFunctions
    legacy_code('sfcn_cmex_generate', defs);

    % compile
    cflags    = '-Wall -Wextra -fopenmp -mtune=native';
    cxxflags  = '-Wall -Wextra -fopenmp -mtune=native -std=c++20';
    ldflags   = '-Wall -Wextra -fopenmp -mtune=native -std=c++20';
    libraries = {'-L/usr/lib','-L/usr/local/lib','-lstdc++','-lpthread','-lgomp'};
    legacy_code('compile', defs, [{['CFLAGS=$CFLAGS ',cflags],['CXXFLAGS=$CXXFLAGS ',cxxflags],['LINKFLAGS=$LINKFLAGS ',ldflags]},libraries]);

    % generate TLC
    legacy_code('sfcn_tlc_generate', defs);

    % generate RTWMAKECFG
    legacy_code('rtwmakecfg_generate', defs);

    % generate Simulink blocks
    if(generateSimulinkBlocks)
        legacy_code('slblock_generate', defs);
    end

    % navigate back to current path
    cd(currentWorkingDirectory);
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

    % delete all files in the output directory
    delete(fullfile(directoryOutput,'*'));

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

