classdef StateIndices
    properties (Constant)
        % Mass
        mass = 1
        
        % Position [3×1]
        pos = 2:4
        x = 2
        y = 3
        z = 4
        
        % Velocity [3×1]
        vel = 5:7
        vx = 5
        vy = 6
        vz = 7
        
        % Quaternion [4×1]
        quat = 8:11
        q0 = 8
        q1 = 9
        q2 = 10
        q3 = 11
        
        % Angular velocity [3×1]
        omega = 12:14
        omega_x = 12
        omega_y = 13
        omega_z = 14
        
        % Total size
        nStates = 14
    end
end