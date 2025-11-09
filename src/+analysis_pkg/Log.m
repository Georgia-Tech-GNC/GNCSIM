classdef Log < handle
    properties (Access = private)
        megaStruct
    end

    methods
        function addLog(dataName, newData)
            if (isfield(megaStruct,dataName))
                disp('Data name already added, updating existing field.')
                updateLog(dataName, newData);
            else
                megaStruct.dataName = newData;
            end
        end

        function updateLog(dataName, updatedData)
            if (~isfield(megaStruct, dataName))
                disp('New data name, creating new field.')
                addLog(dataName, updatedData)
            else
                megaStruct.dataName = updatedData;
            end
        end
    end
end