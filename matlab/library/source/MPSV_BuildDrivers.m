clear all;
cd(extractBefore(mfilename('fullpath'),strlength(mfilename('fullpath')) - strlength(mfilename) + 1));
sourceFiles = UpdateCodeDirectory();

% Generate specifications for all drivers
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
def.SourceFiles             = [{'MPSV_DriverPathPlanner.cpp','MPSV_WrapperPathPlanner.cpp'}, sourceFiles];
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
def.SourceFiles             = [{'MPSV_DriverAsynchronousOnlinePlanner.cpp','MPSV_WrapperAsynchronousOnlinePlanner.cpp'}, sourceFiles];
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
% Generate SFunctions
legacy_code('sfcn_cmex_generate', defs);

% Compile
cflags    = '-Wall -Wextra -fopenmp -mtune=native';
cxxflags  = '-Wall -Wextra -fopenmp -mtune=native -std=c++20';
ldflags   = '-Wall -Wextra -fopenmp -mtune=native -std=c++20';
libraries = {'-L/usr/lib','-L/usr/local/lib','-lstdc++','-lpthread','-lgomp'};
legacy_code('compile', defs, [{['CFLAGS=$CFLAGS ',cflags],['CXXFLAGS=$CXXFLAGS ',cxxflags],['LINKFLAGS=$LINKFLAGS ',ldflags]},libraries]);

% Generate TLC
legacy_code('sfcn_tlc_generate', defs);

% Generate RTWMAKECFG
legacy_code('rtwmakecfg_generate', defs);

% Generate Simulink blocks (not required, all blocks are already in the library)
% legacy_code('slblock_generate', defs);


% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% PRIVATE HELPER FUNCTIONS
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function sourceFiles = UpdateCodeDirectory()
    % Set input and output directory (input: original MPSV source code, output: modified source code for simulink driver)
    thisDirectory = extractBefore(mfilename('fullpath'),strlength(mfilename('fullpath')) - strlength(mfilename) + 1);
    directorySource = fullfile(thisDirectory,'..','..','..','source');
    info = what(directorySource);
    if(~isempty(info)), directorySource = info.path; end
    directoryInput = fullfile(directorySource,'mpsv');
    directoryOutput = fullfile(thisDirectory,'code');

    % Delete all files in the output directory
    delete(fullfile(directoryOutput,'*'));

    % Search for all header and source files in the input directory
    listings = [dir(fullfile(directoryInput,['**',filesep,'*.hpp'])); dir(fullfile(directoryInput,['**',filesep,'*.cpp']))];
    listings = listings(~[listings.isdir]);
    sourceFiles = {};
    for i = 1:numel(listings)
        % Convert file by file
        [inputFile, outputFile] = GetFilenames(listings(i), directorySource, directoryOutput);
        ConvertFiles(inputFile, outputFile);

        % Return filename
        [~,name,ext] = fileparts(outputFile);
        if(strcmp(ext, '.cpp'))
            sourceFiles = [sourceFiles, {[name, ext]}];
        end
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
    % Read input file
    content = fileread(inputFile);

    % Modify the content
    lines = splitlines(content);
    for n = 1:numel(lines)
        if(startsWith(lines{n},'#include <mpsv/') && endsWith(lines{n},'.hpp>') && ~contains(lines{n},'//') && ~contains(lines{n},'/*') && ~contains(lines{n},'*/'))
            lines{n} = replace(lines{n},'/','_');
        end
    end
    content = join(lines,newline);
    s = content{1};

    % Write to output file
    fileID = fopen(outputFile,'w');
    if(-1 == fileID)
        error('Could not write to file "%s"', outputFile);
    end
    fwrite(fileID, uint8(s));
    fclose(fileID);
end

