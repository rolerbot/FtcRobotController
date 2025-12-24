package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter implements Subsystem{
    private ButtonReader Aruncare;
    private final GamepadEx ct1;
    private final double[] pozitiiAruncare = {0.225, 0.6117, 1}; // 3 pozitii aruncare
    private final Mixer mixer;
    private final Intake intake;

    public Servo ServoRidicare = null;
    /// Motor Aruncare
    public  DcMotorEx MotorAruncare = null;
    public Shooter(Mixer mixer,Intake intk,GamepadEx ct1)
    {
        this.ct1 = ct1;
        this.mixer = mixer;
        this.intake = intk;
    }
     public void LinkComponents(HardwareMap hardwareMap)
     {
        Aruncare = new ButtonReader(ct1, GamepadKeys.Button.A);
        ServoRidicare = hardwareMap.get(Servo.class, "ServoRidicare");
        MotorAruncare = hardwareMap.get(DcMotorEx.class, "MotorAruncare");
    }
    public void Initialize(HardwareMap hwMap)
    {
        LinkComponents(hwMap);
        MotorAruncare.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorAruncare.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorAruncare.setDirection(DcMotorSimple.Direction.FORWARD);

        ServoRidicare.setDirection(Servo.Direction.FORWARD);
        ServoRidicare.setPosition(0);
    }
    public void Run(){
        AruncareArtifacte();
    }
    void AruncareArtifacte()
    {
        Aruncare.readValue();
        if (Aruncare.wasJustPressed() && intake.IsStopped() && !mixer.IsEmpty() && mixer.GetTimerElapsed() == 0)
        {
            telemetry.addData("AM ARUNCAT la pozitia: ", mixer.GetCurrentPosition());
            mixer.ResetTimer();
            //int pozmixer = mixer.GetCurrentPosition();
            //if (pozmixer == 0)

            //mixer.SetPosition();
        }
    }

}
