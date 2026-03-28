
classdef StateEnum < Simulink.IntEnumType
    enumeration
        Initialization(0)
        Calibration(1)   
        Idle(2)
        StartingMotors(3)
        Running(4)
        StoppingMotors(5)
        Error(6)
    end
end
