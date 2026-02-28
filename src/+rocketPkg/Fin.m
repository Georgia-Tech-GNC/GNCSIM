% Main Authors: David Reynolds (dreynolds45@gatech.edu)

classdef Fin < rocketPkg.DatcomManager
	% FIN Represents an aerodynamic fin component on a rocket or aircraft
	properties
		finParams struct		% Structure containing geometric and placement parameters for this fin
        name (1,1) string		% String identifier for the fin (e.g., 'fin_1', 'fin_2', etc.)
	end

	methods
		% Constructor
		function obj = Fin(name, finParams, flightConParams)
			obj = obj@rocketPkg.DatcomManager(flightConParams)

            obj.name = name;
            obj.finParams = finParams;
        end

        function [fB, mB] = getForcesMoments(obj, state, deflection, currentCg, envConditions)
            arguments
                obj
                state (:,1) double
                deflection (1,1) double
                currentCg (1,1) double
                envConditions (1,1) flightPkg.AtmosphericConditions
            end

           % Rotation from inertial to body using quaternion
           q = state(obj.idx.quat);
           cBI = Utils.dirCosine(q);

           % Body frame velocity
           vB = state(obj.idx.vel);

           % Convert wind from inertial to body
           vW = cBI * envConditions.WindVector; % Wind velocity (ignored for now)

           % Add componet velocity and wind velocity
           vB = vB + vW;

           % Rotation from body frame to component (fin) frame
		   cCB = Utils.finDcm(deflection, obj.finParams.angle);

           % Component frame velocity and ang vel
           wB = state(obj.idx.omega);
           rCb = -obj.finParams.translation(:) + [currentCg; 0; 0];
           vC = cCB*(vB + cross(wB, rCb));

		   % Get component forces and moments based on
		   [fC, mC] = obj.getForcesMomentsDatcom(state, vC, envConditions);

           % Converting back to body frame
           cBC = cCB';
           fB = cBC*fC;
           mBCross = cross(rCb, fB);
           mB = cBC*mC + mBCross(:);
		end
	end
end