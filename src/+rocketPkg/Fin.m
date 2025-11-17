classdef Fin < rocketPkg.DatcomManager
	% FIN Represents an aerodynamic fin component on a rocket or aircraft

	properties
		finParams		% Structure containing geometric and placement parameters for this fin
        name			% String identifier for the fin (e.g., 'fin_1', 'fin_2', etc.)
					% DATCOM aerodynamic data
	end

	methods
		% Constructor
		function obj = Fin(name, finParams, flightConParams)
			obj = obj@rocketPkg.DatcomManager(flightConParams)

            obj.name = name;
            obj.finParams = finParams;
        end

        function [fB, mB] = getForcesMoments(obj, state, currentCg, disturbances)

           % Rotation from inertial to body using quaternion
           q = state(8:11);
           cBI = Utils.dirCosine(q);

           % Body frame velocity
           vB = state(5:7);

           % vW = cBI * disturbances.wind
           vW = cBI * [0;0;0]; % Wind velocity (ignored for now)

           vB = vB + vW;

           % Rotation from body frame to component (fin) frame
		   cCB = Utils.finDcm(0, obj.finParams.angle);

           % Component frame velocity and ang vel
           wB = state(12:14);
           rCb = -obj.finParams.translation(:) + [currentCg; 0; 0];
           vC = cCB*(vB + cross(wB, rCb));

		   % Get component forces and moments based on
		   [fC, mC] = obj.getForcesMomentsDatcom(state, vC);

           % Converting back to body frame
           cBC = cCB';
           fB = cBC*fC;
           mBRoss = cross(rCb, fB);
           mB = cBC*mC + mBRoss(:);
		end
	end
end