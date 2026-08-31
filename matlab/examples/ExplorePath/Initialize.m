% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% ASV SETTINGS
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
asv = mpsv.designer.ASV();

% geometry (for plotting)
asv.geometry.verticesVehicleShape   = [3.25 -3.25 -3.25 3.25; 1.65 1.65 -1.65 -1.65];

% dynamical model
asv.model.matF = [
    -0.0573463 0 0 0 0 0 0 0 0 -0.0138584 0 0
    0 -0.0335198 0.0697331 0 0 0 0 0 0 0 -0.226787 0.825414
    0 -0.00127197 -0.0268032 0 0 0 0 0 0 0 -0.00460644 -0.606744
];
asv.model.matB = [
    0.000368714 0 0
    0 0.000346504 -0.000162273
    0 8.61905e-06 0.000101692
];
asv.model.vecTimeconstantsXYN = [0.797; 0.766; 0.687];
asv.model.vecTimeconstantsInput = [4; 4; 4];
asv.model.vecSatXYN = [400; 160; 250];


% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% SIMULATION SETTINGS
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
simulation.sampletime = 0.02;
simulation.stoptime = 180.0;
simulation.examplePath = [0 0 0; 10 0 0; 20 0 deg2rad(90); 30 0 deg2rad(90); 30 20 deg2rad(90); 20 20 deg2rad(90); 10 20 0; 0 20 0]';


% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% TUNE CONTROLLER
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
controlEffort        = 0.7;
desiredRadiusX       = 6.0;
manualLimitRadiusX   = 8.0;
manualLimitRadiusY   = 6.0;
manualLimitRadiusPsi = deg2rad(30);

result = mpsv.designer.AutoTune(asv, controlEffort, desiredRadiusX);
K = result.explorePath.matK;
rx = result.explorePath.maxRadiusX;
ry = result.explorePath.maxRadiusY;
rpsi = result.explorePath.maxRadiusPsi;

fprintf('AutoTuner:\n');
fprintf('    K:'); disp(K);
fprintf('    rx: %.4f\n', rx);
fprintf('    ry: %.4f\n', ry);
fprintf('    rpsi: %.4f\n\n', rpsi);

rx = min(rx, manualLimitRadiusX);
ry = min(ry, manualLimitRadiusY);
rpsi = min(rpsi, manualLimitRadiusPsi);
fprintf('Applying Manual Limits:\n');
fprintf('    rx: %.4f\n', rx);
fprintf('    ry: %.4f\n', ry);
fprintf('    rpsi: %.4f\n\n', rpsi);
