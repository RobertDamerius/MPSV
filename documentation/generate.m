% Make sure you have installed the SimpleDoc MATLAB package.
% https://github.com/RobertDamerius/SimpleDoc

% Set title and directories
title           = 'MPSV - Motion Planner for Surface Vehicles';
inputDirectory  = 'text';
outputDirectory = 'html';

% Use a custom navigation bar layout
layoutNavBar = [
    SimpleDoc.NavEntry(SimpleDoc.NavEntryType.none);
    SimpleDoc.NavEntry(SimpleDoc.NavEntryType.text, 'GETTING STARTED');
    SimpleDoc.NavEntry(SimpleDoc.NavEntryType.link, 'Overview', 'index.html');
    SimpleDoc.NavEntry(SimpleDoc.NavEntryType.none);
    SimpleDoc.NavEntry(SimpleDoc.NavEntryType.text, 'SIMULINK LIBRARY');
    SimpleDoc.NavEntry(SimpleDoc.NavEntryType.link, 'PathPlanner', 'library_pathplanner.html');
    SimpleDoc.NavEntry(SimpleDoc.NavEntryType.link, 'Default PathPlanner Input Bus', 'library_pathplannerinput.html');
    SimpleDoc.NavEntry(SimpleDoc.NavEntryType.link, 'Default PathPlanner Parameter Bus', 'library_pathplannerparameter.html');
    SimpleDoc.NavEntry(SimpleDoc.NavEntryType.link, 'Default PathPlanner Output Bus', 'library_pathplanneroutput.html');
    SimpleDoc.NavEntry(SimpleDoc.NavEntryType.link, 'AsyncOnlinePlanner', 'library_asynconlineplanner.html');
    SimpleDoc.NavEntry(SimpleDoc.NavEntryType.link, 'Default AsyncOnlinePlanner Input Bus', 'library_asynconlineplannerinput.html');
    SimpleDoc.NavEntry(SimpleDoc.NavEntryType.link, 'Default AsyncOnlinePlanner Parameter Bus', 'library_asynconlineplannerparameter.html');
    SimpleDoc.NavEntry(SimpleDoc.NavEntryType.link, 'Default AsyncOnlinePlanner Output Bus', 'library_asynconlineplanneroutput.html');
    SimpleDoc.NavEntry(SimpleDoc.NavEntryType.link, 'Serialize AsyncOnlinePlanner Input Bus', 'library_serializeasynconlineplannerinput.html');
    SimpleDoc.NavEntry(SimpleDoc.NavEntryType.link, 'Serialize AsyncOnlinePlanner Parameter Bus', 'library_serializeasynconlineplannerparameter.html');
    SimpleDoc.NavEntry(SimpleDoc.NavEntryType.link, 'Deserialize AsyncOnlinePlanner Output Bus', 'library_deserializeasynconlineplanneroutput.html');
    SimpleDoc.NavEntry(SimpleDoc.NavEntryType.link, 'Select Current Point On Trajectory', 'library_selectcurrentpointontrajectory.html');
];

% Generate HTML documentation
SimpleDoc.Make(title, inputDirectory, outputDirectory, layoutNavBar, false);

% View documentation in MATLAB browser
open(fullfile(outputDirectory,'index.html'));
