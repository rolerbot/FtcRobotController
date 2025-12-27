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
    private TelemetryCustom logger;
    private Husky husky;
    private double currentPosition, offsetPosition;
    private final double initialPosition = 0; // 0 si 0.032 - final
    private final double finalPosition = 0.37;
    //private final double currentPosition = initialPosition;
    private ElapsedTime runtime = new ElapsedTime();
    boolean isShooting = false;
    // private final double[] pozitiiAruncare = {0.225, 0.6117, 1}; // 3 pozitii aruncare
    boolean shooterPrepare = false;
    private final Mixer mixer;
    private final Intake intake;
    private final TelemetryCustom telemetry;
    private boolean preparingLaunch = false;
    public Servo ServoRidicare = null;
    /// Motor Aruncare
    public DcMotorEx MotorAruncare1 = null;
    public DcMotorEx MotorAruncare2 = null;
    private double motorPower = 0.6;
    private int artifactArangement = 0;
    private double[] artPoz = {0.0206 + 0.3834, 0.0206, 0.0206 + 2 * 0.3834}; //pregatire
    public Shooter(TelemetryCustom tl, Mixer mixer,Intake intk,GamepadEx ct1)
    {
        this.ct1 = ct1;
        this.mixer = mixer;
        this.intake = intk;
        this.telemetry = tl;
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
        ServoRidicare.setDirection(Servo.Direction.REVERSE);
        ServoRidicare.setPosition(initialPosition);
    }
    public void Run()
    {
        Aruncare.readValue();
        if(!mixer.IsEmpty() && !shooterPrepare)
        {
            shooterPrepare = true;
            PrepareLaunch();
        }
        if(Aruncare.wasJustPressed())
        {
            preparingLaunch = true;
            artifactArangement = 1;
           /* if(mixer.GetCountGreen() == 1 && mixer.GetCountPurple() == 2)
                artifactArangement = 2;
            else artifactArangement = 1;*/ //Test dupa functionarea huskyului
        }
        ArangedShooting();
        Shooting();
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

    private void StopShooterMotors(){PowerShooterMotors(0);}

    public boolean GetIsShooting() {return isShooting;}
    private void Shooting()
    {
        if(artifactArangement == 1)
        {
            if (preparingLaunch && !isShooting && !mixer.IsEmpty())
            {
                isShooting = true;
                PowerShooterMotors(this.motorPower);
                ResetTimer();
                mixer.NextPosition(); // pregateste sa traga
            }
            if (isShooting && runtime.seconds() > 0.7 && runtime.seconds() <= 1) // trage
                SetPositionLever(finalPosition);
            else if (isShooting && runtime.seconds() > 1 && runtime.seconds() <= 1.25) // coboara
                SetPositionLever(initialPosition);
            else if (isShooting && runtime.seconds() > 1.25 && runtime.seconds() < 1.5) //se roteste
            {
                isShooting = false;
                mixer.RemoveArtifact();
                if (mixer.IsEmpty())
                {
                    StopShooterMotors();
                    preparingLaunch = false;
                    if (intake.IsForward())
                        intake.SetPowerMax();
                    mixer.ResetServoPosition();
                    telemetry.Log("Poz Servos", mixer.GetServoPosition());
                    shooterPrepare = false;
                    artifactArangement = 0;
                    //mixer.ReverseIncrement();
                } else
                    mixer.NextPosition();
                ResetTimer();
            }
        }
    }

    private boolean CanShootAranged()
    {
        for(int i = 0; i < 3; i++)
        {
            if(mixer.artifacte[i] != husky.artifactOrder[i])
                return false;
        }
        return true;
    }

    private void ArangedShooting()
    {
        if(artifactArangement == 2 && CanShootAranged())
        {
            for (int i = 0; i < 3; i++)
            {
                int j;
                for (j = 0; j < 3 && mixer.artifacte[j] != husky.artifactOrder[i]; j++);
                LaunchArtifact(j);
            }
        }
        else artifactArangement = 0;
    }

    private void LaunchArtifact(int index)
    {
        if (preparingLaunch && !isShooting && !mixer.IsEmpty())
        {
            isShooting = true;
            PowerShooterMotors(this.motorPower);
            ResetTimer();
            mixer.SetPozition(artPoz[index]); // pregateste sa traga
        }
        if (isShooting && runtime.seconds() > 0.7 && runtime.seconds() <= 1) // trage
            SetPositionLever(finalPosition);
        else if (isShooting && runtime.seconds() > 1 && runtime.seconds() <= 1.25) // coboara
            SetPositionLever(initialPosition);
        else if (isShooting && runtime.seconds() > 1.25 && runtime.seconds() < 1.5) //se roteste
        {
            isShooting = false;
            mixer.RemoveArtifact();
            if (mixer.IsEmpty())
            {
                StopShooterMotors();
                preparingLaunch = false;
                if (intake.IsForward())
                    intake.SetPowerMax();
                mixer.ResetServoPosition();
                telemetry.Log("Poz Servos", mixer.GetServoPosition());
                shooterPrepare = false;
                artifactArangement = 0;
            } else
                mixer.SetPozition(artPoz[index] + 0.3834 / 2);
            ResetTimer();
        }
    }
    private void PrepareLaunch()
    {
        PowerShooterMotors(0.45);
    }
}
