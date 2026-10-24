function [fB, mB, hingeMoments] = getAeroForcesAndMoments(result, model, dyn, rAreftoCG)

    arguments
        result  (1,1) struct
        model (1,1) aeroPkg.AeroModel
        dyn (1,1) struct
        rAreftoCG (3,1) double
    end 
    
    q = dyn.q;

    fB = q*model.Sref*result.CF;

    mAeroRef = q*model.Sref*model.Lref*result.CM_aeroRef;
    mB = mAeroRef - cross(rAreftoCG, fB);

    hingeMoments = q*model.tabRef.S*model.tabRef.c*result.Ch;

end