classdef (Abstract) AeroModel < handle

    properties (Abstract, SetAccess = protected)
        Sref
        Lref
        tabRef

    end

    methods(Abstract)
        result = evaluateNominal(obj, aeroCon)
    end
end