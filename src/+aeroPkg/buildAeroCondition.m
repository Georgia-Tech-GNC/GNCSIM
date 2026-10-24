function [aeroCon, dyn] = buildAeroCondition(state, envConditions, deltaFin)

    arguments
        state (:,1) double
        envConditions (1,1) flightPkg.AtmosphericConditions
        deltaFin (:,1) double
    end

    VAIR_FLOOR       = 1e-3;   
    ALPHA_FREEZE_DEG = 0.05;
    
    idx = StateIndices;
    rho = envConditions.Density;

    cBI = Utils.dirCosine(state(idx.quat));
    vAirB = state(idx.vel) - cBI*envConditions.WindVector;

    u = vAirB(1);
    v = vAirB(2);
    w = vAirB(3);

    vAir = norm(vAirB);

    if vAir < VAIR_FLOOR
        aeroCon = struct('M', 0, 'alphaTot', 0, 'phiAir', 0, 'deltaFin', deltaFin);
        dyn = struct('vAir', 0, 'rho', rho, 'q', 0);
        return
    end

    alphaTot = atan2d(hypot(v, w), u);

    if alphaTot < ALPHA_FREEZE_DEG
        phiAir = 0;
    else
        phiAir = atan2d(w, v); 
    end

    aeroCon.M = vAir/envConditions.SpeedOfSound;
    aeroCon.alphaTot = alphaTot;
    aeroCon.phiAir = phiAir;
    aeroCon.deltaFin = deltaFin;

    dyn.vAir = vAir;
    dyn.rho = rho;
    dyn.q = 0.5*rho*vAir^2;

end