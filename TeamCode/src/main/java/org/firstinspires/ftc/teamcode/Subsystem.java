package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface Subsystem {

    public void Initialize();
    public void LinkComponents(HardwareMap hardwareMap);
    public void Run();
}
