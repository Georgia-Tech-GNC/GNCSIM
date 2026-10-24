classdef (Abstract) AeroModel < handle

    properties (Abstract, SetAccess = protected)
        Sref
        Lref
        tabRef

        modelVersion
        interfaceVersion
        sourceCFD
    end

    methods(Abstract)
        result = evaluateNominal(obj, aeroCon)
        realization = sampleUncertainty(obj,seed)
        d = evaluateUncertainty(obj, realization, aeroCon)
    end

    methods
        function manifest = getManifest(obj)
            manifest = struct( ...
                'modelVersion',     obj.modelVersion, ...
                'interfaceVersion', obj.interfaceVersion, ...
                'sourceCFD',        obj.sourceCFD);
        end
    end
end