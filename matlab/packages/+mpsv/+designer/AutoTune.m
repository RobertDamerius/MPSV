function tparam = AutoTune(asv, controlEffort)
    %mpsv.designer.AutoTune Calculate all tuning parameters for the MPSV motion planner based on given vehicle parameters and
    % one intuitive tuning parameter.
    % 
    % PARAMETER
    % asv           ... Given vehicle settings structure of type mpsv.designer.ASV.
    % controlEffort ... The amount of control effort in range (0,1] used to drive the vehicle in a steady-state in longitudinal direction.
    % 
    % RETURN
    % tunedParameters ... The tuned parameter structure of type mpsv.designer.Tuning.
    % 
    % DETAILS
    % > timings
    %   The computation time on reset is set to 20 percent of the maximum computation times. The additional ahead planning time
    %   is max(0.2, 10*sampletime).
    % 
    % > skeletalPoints
    %   The skeletal points are two points [xmax; ymean] and [xmin; ymean], where xmax and xmin are the maximum and minimum
    %   vertex coordinates of the vehicle shape vertices, respectively, and ymean is the average y coordinate of the
    %   vehicle shape vertices.
    % 
    % > costMapResolution, costMapModBreakpoints
    %   The modulo factor for breakpoints is set to 10 and the resolution is 1 percent of the largest vehicle dimension.
    % 
    % > matK, maxRadiusX, maxRadiusY, maxRadiusPsi
    %   The controller gain and guidance radii are connected to each other. First, the longitudinal stop distance is
    %   calculated based on the maximum force according to the saturation value of the model. The desired radius in x
    %   direction is then set to 1.2 * stopDistance. Afterwards, the controller gain is adjusted in a bisection search
    %   until the control effort would match the specified controlEffort value. The guidance radii are calculated based
    %   on the controller gain matrix and the steady-state force and velocity.
    % 
    % > minRadiusPosition
    %   This value is the lowest value of maxRadiusX, maxRadiusY as well as the minimum vehicle dimension.
    % 
    % > rangePose, rangeUVR, rangeXYN
    %   The range is calculated based on vehicle dimension, maximum velocity and maximum force, e.g. 5 percent of the
    %   maximum value. For sway speed, 10 percent is used. The angular range is fixed to 5 degrees.
    % 
    % > samplingRangePosition, samplingRangeAngle
    %   The angular sampling range around a path is fixed to 30 degrees. The positional range is calculated based on the
    %   guidance radius.
    % 
    % > maxInputPathLength
    %   This length is calculated from the maximum velocity and total replanning time and the greatest guidance radius
    %   such that a subgoal pose is not within the guidance radius within the total replanning time.
    % 
    % > maxUVR (information-purpose only)
    %   This value represents the maximum velocity that would be reached when driving with a maximum force according to
    %   the model saturation value. Each DoF is considered separately, assuming that matB of the model is mainly
    %   diagonal.
    % 
    % > steadyStateUVR (information-purpose only)
    %   This value represents the steady-state velocity that is reached when driving with a nominal force of
    %   controlEffort*satXYN, where satXYN is the input saturation value of the model. Each DoF is considered separately,
    %   assuming that matB of the model is mainly diagonal.
    arguments (Input)
        asv (1,1) mpsv.designer.ASV
        controlEffort (1,1) double {mustBeFinite, mustBeNonnegative, mustBeLessThanOrEqual(controlEffort,1)} = 0.74
    end
    arguments (Output)
        tparam (1,1) mpsv.designer.Tuning
    end
    tparam = mpsv.designer.Tuning();

    % timings
    tparam.timing.maxComputationTimePathOnReset = 0.2 * asv.timing.maxComputationTimePath;
    tparam.timing.maxComputationTimeMotionOnReset = 0.2 * asv.timing.maxComputationTimeMotion;
    tparam.timing.additionalAheadPlanningTime = max(0.2, 10 * asv.timing.sampletime);
    totalReplanningTime = asv.timing.maxComputationTimePath + asv.timing.maxComputationTimeMotion + tparam.timing.additionalAheadPlanningTime;
    tparam.timing.additionalTrajectoryDuration = max(1, 10 * asv.timing.sampletime);
    tparam.timing.timeKeepPastTrajectory = tparam.timing.additionalAheadPlanningTime + tparam.timing.additionalTrajectoryDuration;

    % geometry
    [tparam.geometry.skeletalPoints, vehicleDimension] = mpsv.designer.details.BuildSkeletalPoints(asv.geometry.verticesVehicleShape);
    tparam.geometry.costMapResolution = max(1e-3, max(vehicleDimension) / 100);
    tparam.geometry.costMapModBreakpoints = int32(10);

    % steady-state control effort
    steadyStateXYN = controlEffort * asv.model.vecSatXYN;
    tparam.info.steadyStateUVR = FindSteadyStateVelocities(steadyStateXYN, asv.model);

    % calculate stop distance (distance required for a full stop when driving with max. velocity)
    tparam.info.maxUVR = FindSteadyStateVelocities(asv.model.vecSatXYN, asv.model);
    stopDistanceX = LongitudinalStopDistance(tparam.info.maxUVR(1), asv.model);

    % bisection search: find control gain that gives a longitudinal radius greater than the stop distance
    desiredRadiusX = 1.2 * stopDistanceX;
    speedFactorLowerBound = 0.3;
    speedFactorUpperBound = 0.95;
    for i = 1:10
        speedFactor = (speedFactorLowerBound + speedFactorUpperBound)/2;
        tparam.explorePath.matK = DesignPoseController(asv.model, speedFactor);
        radiusXYPsi = DesignGuidanceRadius(tparam.explorePath.matK, steadyStateXYN, tparam.info.steadyStateUVR, asv.model);
        tparam.explorePath.maxRadiusX = radiusXYPsi(1);
        tparam.explorePath.maxRadiusY = radiusXYPsi(2);
        tparam.explorePath.maxRadiusPsi = radiusXYPsi(3);
        if(tparam.explorePath.maxRadiusX < desiredRadiusX)
            speedFactorUpperBound = speedFactor;
        else
            speedFactorLowerBound = speedFactor;
        end
    end

    % select lowest radius
    tparam.explorePath.minRadiusPosition = min(min(vehicleDimension), min(tparam.explorePath.maxRadiusX, tparam.explorePath.maxRadiusY));

    % calculate thresholds for reaching a pose
    tparam.explorePath.rangePose(1) = 0.05 * norm(vehicleDimension/2);
    tparam.explorePath.rangePose(2) = tparam.explorePath.rangePose(2);
    tparam.explorePath.rangePose(3) = deg2rad(5);
    tparam.explorePath.rangeUVR = diag([0.05, 0.1, 0.05]) * tparam.info.maxUVR;
    tparam.explorePath.rangeXYN = 0.05 * asv.model.vecSatXYN;

    % calculate maximum input path length for motion planner
    % actually, steady-state velocity would be enough, but max velocity gives a bit more headroom
    tparam.motionPlanner.maxInputPathLength = max(tparam.info.maxUVR(1:2))*totalReplanningTime + max(tparam.explorePath.maxRadiusX, tparam.explorePath.maxRadiusY);

    % calculate sampling range for motion planner
    tparam.motionPlanner.samplingRangePosition = max(tparam.explorePath.minRadiusPosition, min(tparam.explorePath.maxRadiusX, tparam.explorePath.maxRadiusY));
    tparam.motionPlanner.samplingRangeAngle = deg2rad(30);
end

%% private helper functions
function steadyStateUVR = FindSteadyStateVelocities(steadyStateXYN, model)
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % STEADY STATE VELOCITIES
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % The non-linear differential equation for velocity is given by
    % 
    %    (u')                                                                    (X)
    %    (v') = F * [u; v; r; v*r; u*r; u*v; u^2; v^2; r^2; u^3; v^3; r^3] + B * (Y)
    %    (r')                                                                    (N)
    % 
    % Setting u' = v' = r' = 0 and setting steady-state values for the input force, one can calculate the corresponding
    % steady-state velocities. Assuming that B is mainly diagonal and off-diagonal elements can be neglected we obtain three
    % equations:
    % 
    %    0 = F(1,1)*u + F(1,7)*u*u + F(1,10)*u*u*u + B(1,1) * steadyStateXYN(1)
    %    0 = F(2,2)*v + F(2,8)*v*v + F(2,11)*v*v*v + B(2,2) * steadyStateXYN(2)
    %    0 = F(3,3)*r + F(3,9)*r*r + F(3,12)*r*r*r + B(3,3) * steadyStateXYN(3)
    % 
    % These equations are solved numerically for u, v and r, respectively. The solutions represent the steady-state velocities.
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    F = model.matF;
    B = model.matB;
    u_max = fsolve(@(u)(F(1,1)*u + F(1,7)*u*u + F(1,10)*u*u*u + B(1,1) * steadyStateXYN(1)), 1, optimoptions('fsolve','Algorithm','levenberg-marquardt','Display','off'));
    v_max = fsolve(@(v)(F(2,2)*v + F(2,8)*v*v + F(2,11)*v*v*v + B(2,2) * steadyStateXYN(2)), 1, optimoptions('fsolve','Algorithm','levenberg-marquardt','Display','off'));
    r_max = fsolve(@(r)(F(3,3)*r + F(3,9)*r*r + F(3,12)*r*r*r + B(3,3) * steadyStateXYN(3)), 1, optimoptions('fsolve','Algorithm','levenberg-marquardt','Display','off'));
    steadyStateUVR = [u_max; v_max; r_max];
end

function stopDistance = LongitudinalStopDistance(u_max, model)
    % simplified surge model parameters
    a1 = model.matF(1,1);
    a2 = model.matF(1,7);
    a3 = model.matF(1,10);
    b1 = model.matB(1,1);
    T1 = model.vecTimeconstantsXYN(1);
    T2 = model.vecTimeconstantsInput(1);
    satX = model.vecSatXYN(1);

    % select step size (h) based on greatest eigenvalue
    avg_eigenvalues = max([1e-9, abs(a1), 1/T1, 1/T2]);
    h = 1/avg_eigenvalues;
    h_over_2 = h / 2;
    h_over_6 = h / 6;

    % initial state with max speed: position, velocity, force, commanded force
    state = [0; u_max; satX; satX];

    % RK4 with limited number of steps
    stopDistance = 0;
    for i = 1:1000
        x = state;
        k1 = [x(2); a1*x(2) + a2*x(2)*x(2) + a3*x(2)*x(2)*x(2) + b1*x(3); (x(4) - x(3))/T1; (-satX - x(4))/T2];
        x = state + h_over_2*k1;
        k2 = [x(2); a1*x(2) + a2*x(2)*x(2) + a3*x(2)*x(2)*x(2) + b1*x(3); (x(4) - x(3))/T1; (-satX - x(4))/T2];
        x = state + h_over_2*k2;
        k3 = [x(2); a1*x(2) + a2*x(2)*x(2) + a3*x(2)*x(2)*x(2) + b1*x(3); (x(4) - x(3))/T1; (-satX - x(4))/T2];
        x = state + h*k3;
        k4 = [x(2); a1*x(2) + a2*x(2)*x(2) + a3*x(2)*x(2)*x(2) + b1*x(3); (x(4) - x(3))/T1; (-satX - x(4))/T2];
        state = state + h_over_6*(k1 + k2+k2 + k3+k3 + k4);
        stopDistance = max(stopDistance, state(1));
        if(state(2) < 0)
            break;
        end
    end
end

function K = DesignPoseController(model, speedFactor)
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % POSE CONTROLLER DESIGN
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % The linearized state space model using the internal feedback linearization is given by
    % 
    %    ( dp' )   ( 0 I 0 0 )   (dp)   ( 0 )
    %    ( z1' )   ( 0 0 I 0 )   (z1)   ( 0 )
    %    ( z2' ) = ( 0 0 0 I ) * (z2) + ( 0 ) * uz
    %    ( z3' )   ( 0 0 0 0 )   (z3)   ( I )
    %
    % where dp is [delta_xb; delta_yb; delta_psi], z1 is [u; v; r] and z1' = z2, z2' = z3. The actual input of the non-linear
    % system is given by
    % 
    %    u_tau = B_cal^-1 * (-A_cal(x_nu) + uz)
    % 
    % where x_nu is the "real" state of the non-linear system. For state-feedback controller design of the flat-output system the
    % control law is
    % 
    %    uz = -Kz * [dp; z1; z2; z3]
    % 
    % Kz (3-by-12) is the control gain matrix to be designed. To prevent overshoot, we set four groups of three poles on the real
    % axis. Because the linearized system above is a pure integrator we are placing our poles based on the absolte eigenvalues of
    % the original non-linear system for the operating point u = v = r = 0. Then an average eigenvalue is used as a "base" group
    % pole and the three remaining groups are distributed between the base pole and the imaginary axis in the following way:
    % 
    %    p1 = -avg_eigenvalues        p2 = p1*k        p3 = p1*k^2        p4 = p1*k^3
    % 
    % Here, k in range (0,1) is a tuning parameter. For k close to 1, poles get closer to the base pole p1 and the system gets
    % "faster". For small k, the dominant pole group gets closer to the imaginary axis and makes the system "slower". k is denoted
    % as the speed-factor.
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    %avg_eigenvalues = mean([zeros(3,1); abs(eig(model.matF(:,1:3))); 1.0 ./ model.vecTimeconstantsXYN; 1.0 ./ model.vecTimeconstantsInput]);
    avg_eigenvalues = mean([abs(eig(model.matF(:,1:3))); 1.0 ./ model.vecTimeconstantsXYN; 1.0 ./ model.vecTimeconstantsInput]);
    sf1 = speedFactor;
    sf2 = sf1*sf1;
    sf3 = sf1*sf2;
    sf4 = sf2*sf2;
    desired_pole_groups = -avg_eigenvalues * [sf1; sf2; sf3; sf4];
    K = PolePlacement(desired_pole_groups);
end

function K = PolePlacement(pole_groups)
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % POLE PLACEMENT FOR INTEGRATOR SYSTEM
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % The integrator system is given by:
    %           ( 0  1  0  0 )     ( 0 )
    %           ( 0  0  1  0 )     ( 0 )
    %   x_dot = ( 0  0  0  1 )*x + ( 0 )*u
    %           ( 0  0  0  0 )     ( 1 )
    % 
    % We want to design a controller u = -K*x such that eig(A - B*K) are equal to desired poles. Usually one could do
    %    A = [zeros(9,3) eye(9); zeros(3,12)];
    %    B = [zeros(9,3); eye(3)];
    %    K = place(A, B, poles);
    % but because of the special structure of A and B, things get simpler. We design a control law to compensate for
    % (A,B) and replace it with desired dynamics F.
    % 
    %   u = B^(-1) * (-A*x + F*x)
    % 
    % For this case, the pseudo-inverse of B is its transpose and B^T * A = 0. Thus the control law becomes
    % 
    %   u = B^T * F * x
    % 
    % which is u = -K*x for K = -B^T * F. To get F, we design 4 first-order delay systems connected in a chain. The
    % transfer function is given by
    % 
    %   Y(s)                           1
    %   ---- = -------------------------------------------------
    %   U(s)   (T1*s + 1) * (T2*s + 1) * (T3*s + 1) * (T4*s + 1)
    % 
    % which can be written in state-space representation, such that dx/dt = F * x + G * u.
    % 
    %        (  0   1   0   0  )        ( 0  )
    %        (  0   0   1   0  )        ( 0  )
    %    F = (  0   0   0   1  ) ,  G = ( 0  )
    %        ( -k1 -k2 -k3 -k4 )        ( k1 )
    % 
    % where
    %    k1 = 1 / (T1*T2*T3*T4)
    %    k2 = (T1 + T2 + T3 + T4) / (T1*T2*T3*T4)
    %    k3 = (T1*T2 + T1*T3 + T1*T4 + T2*T3 + T2*T4 + T3*T4) / (T1*T2*T3*T4)
    %    k4 = (T1*T2*T3 + T1*T2*T4 + T1*T3*T4 + T2*T3*T4) / (T1*T2*T3*T4)
    % 
    % The final control gain matrix is then calculated by
    % 
    %    K = [k1 k2 k3 k4]
    % 
    % The timeconstants T1,...,T4 are calculated from the desired poles and because we have 3 DoF and want to place
    % groups of 3 poles, we do everything with 3-dimensional entries.
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    T1 = 1 / -pole_groups(1);
    T2 = 1 / -pole_groups(2);
    T3 = 1 / -pole_groups(3);
    T4 = 1 / -pole_groups(4);
    
    % some precalculations
    T12 = T1*T2;
    T34 = T3*T4;
    T1234 = T12*T34;

    % gains
    k1 = 1 / T1234;
    k2 = (T1 + T2 + T3 + T4) / T1234;
    k3 = (T12 + (T1 + T2)*T3 + (T1 + T2)*T4 + T34) / T1234;
    k4 = (T12*(T3 + T4) + (T1 + T2)*T34) / T1234;

    % final gain matrix for all 3 DoF
    K = [k1*eye(3), k2*eye(3), k3*eye(3), k4*eye(3)];
end

function maxRadiusXYPsi = DesignGuidanceRadius(Kz, steadyStateXYN, steadyStateUVR, model)
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % MAXIMUM POSE ERROR -> MAXIMUM GUIDANCE RADIUS
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % For the pose controller, the control law is given by
    % 
    %    uz = -Kz * [dp; z1; z2; z3] = -[Kp Kz1 Kz2 Kz3] * [dp; z1; z2; z3]
    % 
    % where z1 = [u;v;r], z2 = [u';v';r'], z3 = [u'';v'';r'']. For a stationary relationship, dp and z1 are constant. Because z1
    % does not change in time, z2 = z3 = 0 (time derivative of const z1). Therefore the control law simplifies to
    % 
    %    uz = -[Kp Kz1 Kz2 Kz3] * [dp; z1; 0; 0]
    %       = -Kp*dp - Kz1*z1
    % 
    % This can be rearranged to obtain the pose error:
    % 
    %    dp = -Kp^-1 * (Kz1*z1 + uz)
    % 
    % For a given velocity z1 and the corresponding control input uz, the pose error is obtained. However, uz is not the control
    % input for the real system. Because of feedback-linearization the actual control is calculated by
    % 
    %    u_tau = B_cal^-1 * (-A_cal(x_nu) + uz)
    % 
    % which can be rearranged to obtain uz. Then the pose error is given by
    % 
    %    dp = -Kp^-1 * (Kz1*[u;v;r] + B_cal*u_tau + A_cal([u;v;r;X;Y;N;Xc;Yc;Nc]))
    % 
    % Because we're interested in a stationary maximum pose error, u_tau = [Xc;Yc;Nc] = [X;Y;N] = [satX;satY;satN] and
    % [u;v;r] = [u_max;v_max;r_max]. Again, we're looking for a single DoF by assuming that v = r = Y = N = 0 if u = u_max and
    % X = satX. The pose error dp is defined to be the vector FROM the commanded pose TO the current pose and thus is negative if
    % if the commanded value is "in front" of the actual pose.
    % 
    %    maxRadius = -dp
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    F = model.matF;
    B = model.matB;
    TXYN = model.vecTimeconstantsXYN;
    Tf123 = model.vecTimeconstantsInput;
    Btau = diag(1.0 ./ TXYN);
    Bc = diag(1.0 ./ Tf123);
    Bcal = B*Btau*Bc;
    dx = [1 0 0] * (-Kz(:,1:3) \ (Kz(:,4:6)*[steadyStateUVR(1); 0; 0] + Bcal*[steadyStateXYN(1); 0; 0] + Acal([steadyStateUVR(1);0;0;steadyStateXYN(1);0;0;steadyStateXYN(1);0;0],F,B,TXYN,Tf123)));
    dy = [0 1 0] * (-Kz(:,1:3) \ (Kz(:,4:6)*[0; steadyStateUVR(2); 0] + Bcal*[0; steadyStateXYN(2); 0] + Acal([0;steadyStateUVR(2);0;0;steadyStateXYN(2);0;0;steadyStateXYN(2);0],F,B,TXYN,Tf123)));
    dz = [0 0 1] * (-Kz(:,1:3) \ (Kz(:,4:6)*[0; 0; steadyStateUVR(3)] + Bcal*[0; 0; steadyStateXYN(3)] + Acal([0;0;steadyStateUVR(3);0;0;steadyStateXYN(3);0;0;steadyStateXYN(3)],F,B,TXYN,Tf123)));
    maxRadiusXYPsi = -[dx; dy; dz];
end

function compensation = Acal(x_nu, matF, matB, vecTimeconstantsXYN, vecTimeconstantsInput)
    % Helper function required to calculate the compensation term of the feedback linearization.
    % The actual control input to the non-linear system augmented with the input filter is u_tau = Bcal^-1 * (-Acal(x_nu) + uz)
    % x_nu = [u;v;r;X;Y;N;Xc;Yc;Nc]
    xu = [0;0;0;x_nu]; % current state (pose, uvr, XYN, XcYcNc)

    % to one-dimensional quantities (row-major order)
    F = [matF(1,:), matF(2,:), matF(3,:)];
    B = [matB(1,:), matB(2,:), matB(3,:)];
    TX = vecTimeconstantsXYN(1);
    TY = vecTimeconstantsXYN(2);
    TN = vecTimeconstantsXYN(3);
    Tf1 = vecTimeconstantsInput(1);
    Tf2 = vecTimeconstantsInput(2);
    Tf3 = vecTimeconstantsInput(3);

    % Precalculate parameters
    invTXYN = [1.0 / TX; 1.0 / TY; 1.0 / TN];
    invTf123 = [1.0 / Tf1; 1.0 / Tf2; 1.0 / Tf3];
    M0 = zeros(9,1); % row-major order B * A_tau
    M0(1) = -B(1) / vecTimeconstantsXYN(1);
    M0(2) = -B(2) / vecTimeconstantsXYN(2);
    M0(3) = -B(3) / vecTimeconstantsXYN(3);
    M0(4) = -B(4) / vecTimeconstantsXYN(1);
    M0(5) = -B(5) / vecTimeconstantsXYN(2);
    M0(6) = -B(6) / vecTimeconstantsXYN(3);
    M0(7) = -B(7) / vecTimeconstantsXYN(1);
    M0(8) = -B(8) / vecTimeconstantsXYN(2);
    M0(9) = -B(9) / vecTimeconstantsXYN(3);
    f17_2 = 2.0 * F(7);
    f18_2 = 2.0 * F(8);
    f19_2 = 2.0 * F(9);
    f1a_3 = 3.0 * F(10);
    f1b_3 = 3.0 * F(11);
    f1c_3 = 3.0 * F(12);
    f27_2 = 2.0 * F(19);
    f28_2 = 2.0 * F(20);
    f29_2 = 2.0 * F(21);
    f2a_3 = 3.0 * F(22);
    f2b_3 = 3.0 * F(23);
    f2c_3 = 3.0 * F(24);
    f37_2 = 2.0 * F(31);
    f38_2 = 2.0 * F(32);
    f39_2 = 2.0 * F(33);
    f3a_3 = 3.0 * F(34);
    f3b_3 = 3.0 * F(35);
    f3c_3 = 3.0 * F(36);
    f1a_6 = 6.0 * F(10);
    f1b_6 = 6.0 * F(11);
    f1c_6 = 6.0 * F(12);
    f2a_6 = 6.0 * F(22);
    f2b_6 = 6.0 * F(23);
    f2c_6 = 6.0 * F(24);
    f3a_6 = 6.0 * F(34);
    f3b_6 = 6.0 * F(35);
    f3c_6 = 6.0 * F(36);

    % forced response velocity model: f(nu) = F*n(nu) + B*tau, where n(nu) = (u,v,r,v*r,u*r,u*v,u^2,v^2,r^2,u^3,v^3,r^3)^T
    forcedResponse = [0;0;0];
    vr = xu(5) * xu(6);
    ur = xu(4) * xu(6);
    uv = xu(4) * xu(5);
    uu = xu(4) * xu(4);
    vv = xu(5) * xu(5);
    rr = xu(6) * xu(6);
    uuu = uu * xu(4);
    vvv = vv * xu(5);
    rrr = rr * xu(6);
    forcedResponse(1) =  F(1)*xu(4) +  F(2)*xu(5) +  F(3)*xu(6) +  F(4)*vr +  F(5)*ur +  F(6)*uv +  F(7)*uu +  F(8)*vv +  F(9)*rr + F(10)*uuu + F(11)*vvv + F(12)*rrr + B(1)*xu(7) + B(2)*xu(8) + B(3)*xu(9);
    forcedResponse(2) = F(13)*xu(4) + F(14)*xu(5) + F(15)*xu(6) + F(16)*vr + F(17)*ur + F(18)*uv + F(19)*uu + F(20)*vv + F(21)*rr + F(22)*uuu + F(23)*vvv + F(24)*rrr + B(4)*xu(7) + B(5)*xu(8) + B(6)*xu(9);
    forcedResponse(3) = F(25)*xu(4) + F(26)*xu(5) + F(27)*xu(6) + F(28)*vr + F(29)*ur + F(30)*uv + F(31)*uu + F(32)*vv + F(33)*rr + F(34)*uuu + F(35)*vvv + F(36)*rrr + B(7)*xu(7) + B(8)*xu(8) + B(9)*xu(9);

    % forced response allocation model: A_tau * tau + B_tau * tau_c
    allocationModel = [0;0;0];
    allocationModel(1) = (xu(10) - xu(7)) * invTXYN(1);
    allocationModel(2) = (xu(11) - xu(8)) * invTXYN(2);
    allocationModel(3) = (xu(12) - xu(9)) * invTXYN(3);

    % Jacobian J = df(nu)/dnu (uvr model w.r.t. uvr)
    J = zeros(9,1);
    J(1) =  F(1) +  F(5)*xu(6) +  F(6)*xu(5) + f17_2*xu(4) + f1a_3*uu;
    J(2) =  F(2) +  F(4)*xu(6) +  F(6)*xu(4) + f18_2*xu(5) + f1b_3*vv;
    J(3) =  F(3) +  F(4)*xu(5) +  F(5)*xu(4) + f19_2*xu(6) + f1c_3*rr;
    J(4) = F(13) + F(17)*xu(6) + F(18)*xu(5) + f27_2*xu(4) + f2a_3*uu;
    J(5) = F(14) + F(16)*xu(6) + F(18)*xu(4) + f28_2*xu(5) + f2b_3*vv;
    J(6) = F(15) + F(16)*xu(5) + F(17)*xu(4) + f29_2*xu(6) + f2c_3*rr;
    J(7) = F(25) + F(29)*xu(6) + F(30)*xu(5) + f37_2*xu(4) + f3a_3*uu;
    J(8) = F(26) + F(28)*xu(6) + F(30)*xu(4) + f38_2*xu(5) + f3b_3*vv;
    J(9) = F(27) + F(28)*xu(5) + F(29)*xu(4) + f39_2*xu(6) + f3c_3*rr;

    % S0 matrix: J^2 + T, where t_m1 = (dj_m1/du, dj_m2/du, dj_m2/du)*forcedResponse, t_m2 = (dj_m1/dv, dj_m2/dv, dj_m2/dv)*forcedResponse, t_m3 = (dj_m1/dr, dj_m2/dr, dj_m2/dr)*forcedResponse for m=1,2,3
    S0 = zeros(9,1);
    S0(1) = J(1)*J(1) + J(2)*J(4) + J(3)*J(7) + (f17_2 + f1a_6*xu(4))*forcedResponse(1) +                  F(6)*forcedResponse(2) +                  F(5)*forcedResponse(3);
    S0(2) = J(1)*J(2) + J(2)*J(5) + J(3)*J(8) +                  F(6)*forcedResponse(1) + (f18_2 + f1b_6*xu(5))*forcedResponse(2) +                  F(4)*forcedResponse(3);
    S0(3) = J(1)*J(3) + J(2)*J(6) + J(3)*J(9) +                  F(5)*forcedResponse(1) +                  F(4)*forcedResponse(2) + (f19_2 + f1c_6*xu(6))*forcedResponse(3);
    S0(4) = J(4)*J(1) + J(5)*J(4) + J(6)*J(7) + (f27_2 + f2a_6*xu(4))*forcedResponse(1) +                 F(18)*forcedResponse(2) +                 F(17)*forcedResponse(3);
    S0(5) = J(4)*J(2) + J(5)*J(5) + J(6)*J(8) +                 F(18)*forcedResponse(1) + (f28_2 + f2b_6*xu(5))*forcedResponse(2) +                 F(16)*forcedResponse(3);
    S0(6) = J(4)*J(3) + J(5)*J(6) + J(6)*J(9) +                 F(17)*forcedResponse(1) +                 F(16)*forcedResponse(2) + (f29_2 + f2c_6*xu(6))*forcedResponse(3);
    S0(7) = J(7)*J(1) + J(8)*J(4) + J(9)*J(7) + (f37_2 + f3a_6*xu(4))*forcedResponse(1) +                 F(30)*forcedResponse(2) +                 F(29)*forcedResponse(3);
    S0(8) = J(7)*J(2) + J(8)*J(5) + J(9)*J(8) +                 F(30)*forcedResponse(1) + (f38_2 + f3b_6*xu(5))*forcedResponse(2) +                 F(28)*forcedResponse(3);
    S0(9) = J(7)*J(3) + J(8)*J(6) + J(9)*J(9) +                 F(29)*forcedResponse(1) +                 F(28)*forcedResponse(2) + (f39_2 + f3c_6*xu(6))*forcedResponse(3);

    % S1 matrix: J * B
    S1 = zeros(9,1);
    S1(1) = J(1)*B(1) + J(2)*B(4) + J(3)*B(7);
    S1(2) = J(1)*B(2) + J(2)*B(5) + J(3)*B(8);
    S1(3) = J(1)*B(3) + J(2)*B(6) + J(3)*B(9);
    S1(4) = J(4)*B(1) + J(5)*B(4) + J(6)*B(7);
    S1(5) = J(4)*B(2) + J(5)*B(5) + J(6)*B(8);
    S1(6) = J(4)*B(3) + J(5)*B(6) + J(6)*B(9);
    S1(7) = J(7)*B(1) + J(8)*B(4) + J(9)*B(7);
    S1(8) = J(7)*B(2) + J(8)*B(5) + J(9)*B(8);
    S1(9) = J(7)*B(3) + J(8)*B(6) + J(9)*B(9);

    % compensation = S0(uvr) * forcedResponse + S1(uvr) * allocationModel + M0 * (allocationModel + Bc*tau_c)
    tmp = [0;0;0];
    tmp(1) = allocationModel(1) + xu(10) * invTf123(1);
    tmp(2) = allocationModel(2) + xu(11) * invTf123(2);
    tmp(3) = allocationModel(3) + xu(12) * invTf123(3);
    compensation = [0;0;0];
    compensation(1) = S0(1)*forcedResponse(1) + S0(2)*forcedResponse(2) + S0(3)*forcedResponse(3) + S1(1)*allocationModel(1) + S1(2)*allocationModel(2) + S1(3)*allocationModel(3) + M0(1)*tmp(1) + M0(2)*tmp(2) + M0(3)*tmp(3);
    compensation(2) = S0(4)*forcedResponse(1) + S0(5)*forcedResponse(2) + S0(6)*forcedResponse(3) + S1(4)*allocationModel(1) + S1(5)*allocationModel(2) + S1(6)*allocationModel(3) + M0(4)*tmp(1) + M0(5)*tmp(2) + M0(6)*tmp(3);
    compensation(3) = S0(7)*forcedResponse(1) + S0(8)*forcedResponse(2) + S0(9)*forcedResponse(3) + S1(7)*allocationModel(1) + S1(8)*allocationModel(2) + S1(9)*allocationModel(3) + M0(7)*tmp(1) + M0(8)*tmp(2) + M0(9)*tmp(3);
end

