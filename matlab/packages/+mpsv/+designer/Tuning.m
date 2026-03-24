classdef Tuning < handle
    properties
        % Geometry parameters
        geometry (1,1) mpsv.designer.TuningGeometry = mpsv.designer.TuningGeometry()

        % Explore-path parameters
        explorePath (1,1) mpsv.designer.TuningExplorePath = mpsv.designer.TuningExplorePath()

        % Motion planner parameters
        motionPlanner (1,1) mpsv.designer.TuningMotionPlanner = mpsv.designer.TuningMotionPlanner()

        % Timing parameters
        timing (1,1) mpsv.designer.TuningTiming = mpsv.designer.TuningTiming()

        % Information-purpose only
        info (1,1) mpsv.designer.TuningInfo = mpsv.designer.TuningInfo()
    end
    methods
        function this = Tuning()
            % constructor required for code generation
            this.geometry = mpsv.designer.TuningGeometry();
            this.explorePath = mpsv.designer.TuningExplorePath();
            this.motionPlanner = mpsv.designer.TuningMotionPlanner();
            this.timing = mpsv.designer.TuningTiming();
            this.info = mpsv.designer.TuningInfo();
        end
    end
end
