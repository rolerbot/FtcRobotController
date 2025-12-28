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
    private ButtonReader ThrowGreen, ThrowPurple;
    private final GamepadEx ct1;
    private Husky husky;
    private final double initialPosition = 0.0;
    private final double finalPosition = 0.37;
    private ElapsedTime runtime = new ElapsedTime();
    boolean isShooting = false;
    private int arrangedIndex = 0;
    private final Mixer mixer;
    private final Intake intake;
    private final TelemetryCustom telemetry;
    private boolean preparingLaunch = false;
    public Servo ServoRidicare = null;
    /// Motor Aruncare
    public DcMotorEx MotorAruncare1 = null;
    public DcMotorEx MotorAruncare2 = null;
    private double motorPower = 0.6;
    private  boolean arranged = false;
    private double[] artPoz = {0.4040, 0.2123, 0.5957};
    private boolean shootingAllowed = false;
    private ElapsedTime launchTime = new ElapsedTime();
    ButtonReader Aruncare2;
    private boolean launchtest = false;

    private void PozTester()
    {
        Aruncare2.readValue();
        if(Aruncare2.wasJustPressed())
        {
            intake.SetMotorPower(0.5);
            launchtest = true;
            launchTime.reset();
        }
        if(launchtest && launchTime.seconds() > 1 && launchTime.seconds() < 2)
            mixer.SetPozition(artPoz[0]);
        else if(launchtest &&  launchTime.seconds() > 2 && launchTime.seconds() < 3)
            mixer.SetPozition(artPoz[1]);
        else if(launchtest &&  launchTime.seconds() > 3 && launchTime.seconds() < 4)
        {
            mixer.SetPozition(artPoz[2]);
            launchtest = false;
        }
    }
    public Shooter(TelemetryCustom tl, Mixer mixer,Intake intk,Husky husky,GamepadEx ct1)
    {
        this.ct1 = ct1;
        this.mixer = mixer;
        this.intake = intk;
        this.telemetry = tl;
        this.husky = husky;
    }
     public void LinkComponents(HardwareMap hardwareMap)
     {
        Aruncare = new ButtonReader(ct1, GamepadKeys.Button.A);
        ThrowGreen = new ButtonReader(ct1, GamepadKeys.Button.DPAD_LEFT);
        ThrowPurple = new ButtonReader(ct1, GamepadKeys.Button.DPAD_RIGHT);
        Aruncare2 = new  ButtonReader(ct1, GamepadKeys.Button.LEFT_BUMPER);
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
        ServoRidicare.setDirection(Servo.Direction.REVERSE);
        ServoRidicare.setPosition(initialPosition);
    }
    public void Run()
    {
        Aruncare.readValue();

        if(Aruncare.wasJustPressed())
        {
            if (CanShootArranged()) arranged = true;
            shootingAllowed = true;
            arrangedIndex = 0;
        }
        if (shootingAllowed)
        {
            PrepareLaunch();
            arranged = Shoot(arranged);
        }
         //PozTester();
    }

    public void SetPositionLever(double position){
        ServoRidicare.setPosition(position);
    }

    private void ResetTimer(){
        runtime.reset();
    }
    public void PowerShooterMotors(double power)
    {
        MotorAruncare1.setPower(power);
        MotorAruncare2.setPower(power);
    }

    private void StopShooterMotors(){
        preparingLaunch = false;
        PowerShooterMotors(0);
    }

    public boolean IsNotShooting() {return !isShooting;}

    private boolean Shoot(boolean arranged)
    {
        boolean finalArranged = arranged;
        int position;

        if (arranged)
            position = mixer.GetColorPosition(husky.artifactOrder[arrangedIndex]);
        else position = 0;

        double servoPosition = artPoz[position];
        if (preparingLaunch && !isShooting && !mixer.IsEmpty())
        {
            isShooting = true;
            PowerShooterMotors(this.motorPower);
            ResetTimer();
            if(arranged)
            {
                mixer.SetPozition(servoPosition);
            }
            else mixer.NextPosition();
        }

        if (!isShooting) return finalArranged;

        int state = GetShootingState();
        switch (state)
        {
            case 1: // trage (0.8 - 1.2s)
                SetPositionLever(finalPosition);
                break;
            case 2: // coboara (1.2 - 1.4s)
                SetPositionLever(initialPosition);
                break;
            case 3: // se roteste (> 1.4s)
                isShooting = false;
                mixer.RemoveArtifact(position);
                if (mixer.IsEmpty())
                {
                    StopShooterMotors();
                    preparingLaunch = false;
                    if (intake.IsForward())
                        intake.SetPowerMax();
                    mixer.ResetServoPosition();
                    shootingAllowed = false;
                    finalArranged = false;
                    arrangedIndex = 0;
                }
                else if (!arranged)
                    mixer.NextPosition();
                else arrangedIndex++;
                ResetTimer();
                break;
        }
        return finalArranged;
    }

    private int GetShootingState()
    {
        double time = runtime.seconds();
        if (time >= 0.8 && time < 1.2)
            return 1;
        else if (time >= 1.2 && time < 1.4)
            return 2;
        else if (time >= 1.4)
            return 3;
        return 0;
    }
    private boolean CanShootArranged() //verifica daca poate trage in ordinea data de husky
    {
        if (mixer.IsEmpty())
            return false;
        return mixer.GetCountGreen() == 1 && mixer.GetCountPurple() == 2 && husky.GetID() != 0;
    }
    private void PrepareLaunch()
    {
        if(!mixer.IsEmpty() && !preparingLaunch)
        {
            preparingLaunch = true;
            PowerShooterMotors(0.45);
        }
    }
}
