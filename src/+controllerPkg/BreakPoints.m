classdef BreakPoints < Handle

    properties
        % Fill in properties as needed
        breakPointList % (list of BreakPoint objects)

    end

    methods
    
        function computeOptimalTrajectory(obj, rocketDynamics)
            % TODO: Port Danil's work here
        end

        function dumbTraj()
            % generate trivial trajectory
        end

        function generateBreakPoints(obj)
            % TODO: Break trajectory based on accumulated velocity or
            % altitude and corresponding hyper parameters. Compute A and B
            % for each break point

        end

        function computeGains(obj, controller)
            % TODO: Call controller for each breakPoint and compute gains

        end

        function stabilityAnalysis(obj)
            % TODO: Closed-loop stability analysis and classical controller
            % performance for each breakpoint (root locus, % overshoot,
            % settling time, rise time, peak time, etc)

        end

        function gainSchedulingInterpolated()

        end

    end

end