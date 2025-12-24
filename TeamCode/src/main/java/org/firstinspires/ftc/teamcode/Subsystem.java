package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface Subsystem {
    public void LinkComponents(HardwareMap hwMap);
    public void Initialize(HardwareMap hwMap);
    public void Run();
}