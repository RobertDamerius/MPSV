classdef ASV < handle
    properties
        % Geometry settings of the vehicle.
        geometry (1,1) mpsv.designer.ASVGeometry = mpsv.designer.ASVGeometry()

        % Dynamical model of the vehicle.
        model (1,1) mpsv.designer.ASVModel = mpsv.designer.ASVModel()

        % Timing for trajectory generation and replanning
        timing (1,1) mpsv.designer.ASVTiming = mpsv.designer.ASVTiming()

        % Metric for the cost function.
        metric (1,1) mpsv.designer.ASVMetric = mpsv.designer.ASVMetric()
    end
    methods
        function this = ASV()
            % constructor required for code generation
            this.geometry = mpsv.designer.ASVGeometry();
            this.model = mpsv.designer.ASVModel();
            this.timing = mpsv.designer.ASVTiming();
            this.metric = mpsv.designer.ASVMetric();
        end
    end
end
