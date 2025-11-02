classdef SensorType
    enumeration
        GYROSCOPE
        ACCELEROMETER % Gyro + Acc make up IMU, seperate for easier sample rate + bias control
        GPS
        MAGNETOMETER  
        BAROMETER
    end
end