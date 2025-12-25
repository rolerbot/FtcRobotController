package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter implements Subsystem{
    private ButtonReader Aruncare;
    private final GamepadEx ct1;
    private final double initialPosition = 0.0506;
    private final double finalPosition = 0.079;
    private final double currentPosition = initialPosition;
    private ElapsedTime runtime = new ElapsedTime();
    boolean isShooting = false;
    // private final double[] pozitiiAruncare = {0.225, 0.6117, 1}; // 3 pozitii aruncare
    private final Mixer mixer;
    private final Intake intake;
    private boolean preparingLaunch = false;
    public Servo ServoRidicare = null;
    /// Motor Aruncare
    public DcMotorEx MotorAruncare1 = null;
    public DcMotorEx MotorAruncare2 = null;
    private double motorPower = 0.6;
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
        MotorAruncare1 = hardwareMap.get(DcMotorEx.class, "MotorAruncare1");
        MotorAruncare2 = hardwareMap.get(DcMotorEx.class, "MotorAruncare2");
    }
    public void Initialize(HardwareMap hwMap)
    {
        LinkComponents(hwMap);
        MotorAruncare1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorAruncare1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorAruncare1.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorAruncare2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorAruncare2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorAruncare2.setDirection(DcMotorSimple.Direction.REVERSE);
        ServoRidicare.setDirection(Servo.Direction.FORWARD);
        ServoRidicare.setPosition(initialPosition);
    }
    public void Run(){
        Aruncare.readValue();
        if(Aruncare.wasJustPressed())
            preparingLaunch = true;
        ArtifactShooting();
    }
    public void SetPositionLever(double position){
        ServoRidicare.setPosition(position);
    }
    //public double GetCurrentPositionLever(){return currentPosition;}
    public double GetPositionLever(){
        return ServoRidicare.getPosition();
    }
    private void ResetTimer(){
        runtime.reset();
    }
    public void PowerShooterMotors(double power)
    {
        MotorAruncare1.setPower(power);
        MotorAruncare2.setPower(power);
    }
    private void StopShooterMotors(){PowerShooterMotors(0);}

    public boolean GetIsShooting() {return isShooting;}
    private void ArtifactShooting()
    {
        if (preparingLaunch && !isShooting && !mixer.IsEmpty())
        {
            isShooting = true;
            PowerShooterMotors(this.motorPower);
            ResetTimer();
            mixer.NextPosition(); // pregateste sa traga
        }
        if (isShooting && runtime.seconds() > 0.5 && runtime.seconds() <= 1) // trage
            SetPositionLever(finalPosition);
        else if(isShooting && runtime.seconds() > 1)
        {
            isShooting = false;
            SetPositionLever(initialPosition); // coboara
            mixer.NextPosition();
            mixer.RemoveArtifact();
            PowerShooterMotors(0.2);
            if(mixer.IsEmpty())
                preparingLaunch = false;
            ResetTimer();
        }
        if(mixer.IsEmpty() && mixer.GetServoPosition() != 0)
        {
            mixer.ResetServoPosition();
            StopShooterMotors();
        }
    }
}
