package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Utils.ColorToString;

public class Shooter implements Subsystem{
    private ButtonReader Aruncare;
    private ButtonReader ThrowGreen, ThrowPurple;
    private ButtonReader OvverideShooting;
    private ButtonReader VelocityChange;
    private final GamepadEx ct1, ct2;
    private final double initialPosition = 0.0;
    private final double finalPosition = 0.37;
    private ElapsedTime runtime = new ElapsedTime();
    boolean isShooting = false;
    private int arrangedIndex = 0;
    private int currentShootingPosition = 0; // Poziția curentă care se aruncă
    private final Mixer mixer;
    private final Intake intake;
    private final TelemetryCustom telemetry;
    private final Husky husky;
    private boolean preparingLaunch = false;
    private boolean shootingAllowed = false;
    private Servo ServoRidicare = null;
    /// Motor Aruncare
    private DcMotorEx MotorAruncare1 = null;
    private DcMotorEx MotorAruncare2 = null;

    // PIDF Configuration - valori din tuning
    private final double shooterF = 13.8;
    private final double shooterP = 0.0;
    private final double shooterI = 0.0;
    private final double shooterD = 0.0;

    // Velocity targets (RPM) - ajustează după nevoie
    private final double highVelocity = 1700;  // Viteza pentru aruncare normală
    private final double lowVelocity = 1500;   // Viteza pentru aruncare ușoară
    private double motorPower = lowVelocity;

    private double offsetPosition = 0.3834 / 2;
    private final double initialPosMixer = 0.0206;
    private ShootingState shootType = ShootingState.None;
    private double[] artPoz = {initialPosMixer + 3 * offsetPosition, initialPosMixer + 5 * offsetPosition, initialPosMixer + offsetPosition};
    public Shooter(TelemetryCustom tl, Mixer mixer,Intake intk,Husky husky,GamepadEx ct1, GamepadEx ct2)
    {
        this.ct1 = ct1;
        this.mixer = mixer;
        this.intake = intk;
        this.telemetry = tl;
        this.husky = husky;
        this.ct2 = ct2;
    }
     public void LinkComponents(HardwareMap hardwareMap)
     {
         Aruncare = new ButtonReader(ct1, GamepadKeys.Button.A);
         ThrowGreen = new ButtonReader(ct1, GamepadKeys.Button.Y);
         ThrowPurple = new ButtonReader(ct1, GamepadKeys.Button.X);
         VelocityChange = new ButtonReader(ct2, GamepadKeys.Button.B);
         OvverideShooting = new ButtonReader(ct2, GamepadKeys.Button.X);
         ServoRidicare = hardwareMap.get(Servo.class, "ServoRidicare");
         MotorAruncare1 = hardwareMap.get(DcMotorEx.class, "MotorAruncare1");
         MotorAruncare2 = hardwareMap.get(DcMotorEx.class, "MotorAruncare2");
     }
     private void ChangeVelocity()
     {
         if (VelocityChange.wasJustPressed())
         {
             if (motorPower == highVelocity)
             {
                 motorPower = lowVelocity;
                 SetShooterVelocity(motorPower);
             }
             else
             {
                 motorPower = highVelocity;
                 SetShooterVelocity(motorPower);
             }
         }
     }
    public void Initialize(HardwareMap hwMap)
    {
        LinkComponents(hwMap);

        // Configurare PIDF pentru ambele motoare
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(shooterP, shooterI, shooterD, shooterF);

        MotorAruncare1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorAruncare1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorAruncare1.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorAruncare1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        MotorAruncare2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorAruncare2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorAruncare2.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorAruncare2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        ServoRidicare.setDirection(Servo.Direction.REVERSE);
        ServoRidicare.setPosition(initialPosition);
    }

    public double GetCurentPower() {return motorPower;}
    public ShootingState GetShootingType(){return shootType;};

    public void Run()
    {
        ReadButtons();
        if(Aruncare.wasJustPressed() && !mixer.IsEmpty())
        {
            shootingAllowed = true;
            if (CanShootArranged()) // Elimină verificarea shootType != None
            {
                shootType = ShootingState.Arranged;
                arrangedIndex = 0;
            }
            else // Dacă nu poate aranja, aruncă nearanjat
            {
                shootType = ShootingState.Unarranged;
                arrangedIndex = 0;
            }
            telemetry.Log("ShootType:", shootType);
        }
        else if(ThrowGreen.wasJustPressed() && !mixer.IsEmpty())
        {
            shootType = ShootingState.Green;
            shootingAllowed = true;
        }
        else if(ThrowPurple.wasJustPressed() && !mixer.IsEmpty())
        {
            shootType = ShootingState.Purple;
            shootingAllowed = true;
        }
        else if(OvverideShooting.wasJustPressed())
        {
            mixer.SetArtifacts(Color.Purple, Color.Purple, Color.Purple);
            shootType = ShootingState.Unarranged;
            shootingAllowed = true;
        }

        if (shootingAllowed)
        {
            Shooting();
        }
        PrepareLaunch();
        ChangeVelocity();
    }
    private void ReadButtons()
    {
        OvverideShooting.readValue();
        VelocityChange.readValue();
        Aruncare.readValue();
        ThrowGreen.readValue();
        ThrowPurple.readValue();
    }
    public void SetPositionLever(double position){
        ServoRidicare.setPosition(position);
    }
    private void ResetTimer(){
        runtime.reset();
    }

    // Controlează motoarele cu velocity (RPM) în loc de power
    public void SetShooterVelocity(double velocity)
    {
        MotorAruncare1.setVelocity(velocity);
        MotorAruncare2.setVelocity(velocity);
    }

    private void StopShooterMotors(){
        preparingLaunch = false;
        SetShooterVelocity(0);
    }

    private void Shooting()
    {
        switch (shootType)
        {
            case Arranged:
                Ordered();
                break;
            case Unarranged:
                Unordered();
                break;
            case Purple:
            case Green:
                PurpleGreen();
                break;
            default:
                break;
        }
    }

    private void Unordered()
    {
        telemetry.Log("Unordered", "");
        if (preparingLaunch && !isShooting && !mixer.IsEmpty())
        {
            if(mixer.GetCountPurple() > 0)
                currentShootingPosition = mixer.GetColorPosition(Color.Purple);
            else if(mixer.GetCountGreen() > 0)
                currentShootingPosition = mixer.GetColorPosition(Color.Green);
            else
                currentShootingPosition = -1;
            telemetry.Log("ShootingPose:", currentShootingPosition);
        }
        ShootColor();
    }

    private void Ordered()
    {
        if (preparingLaunch && !isShooting && !mixer.IsEmpty())
        {
            currentShootingPosition = mixer.GetColorPosition(husky.artifactOrder[arrangedIndex]);
            telemetry.Log("Target Color", ColorToString(husky.artifactOrder[arrangedIndex]));
        }
        ShootColor();
    }

    private void PurpleGreen()
    {
        if (preparingLaunch && !isShooting && !mixer.IsEmpty())
        {
            if(shootType == ShootingState.Green && mixer.GetCountGreen() > 0)
                currentShootingPosition = mixer.GetColorPosition(Color.Green);
            else if(shootType == ShootingState.Purple && mixer.GetCountPurple() > 0)
                currentShootingPosition = mixer.GetColorPosition(Color.Purple);
            else
                currentShootingPosition = -1;
        }
        ShootColor();
    }

    private void ShootColor()
    {
        if (preparingLaunch && !isShooting && !mixer.IsEmpty() && currentShootingPosition >= 0)
        {
            isShooting = true;
            SetShooterVelocity(motorPower);
            ResetTimer();
            mixer.SetPozition(artPoz[currentShootingPosition]);
            telemetry.Log("Shooting position", currentShootingPosition);
        }

        if (isShooting)
        {
            switch (GetShootingState())
            {
                case 1:
                    SetPositionLever(finalPosition);
                    break;
                case 2:
                    SetPositionLever(initialPosition);
                    break;
                case 3:
                    CompleteShot(currentShootingPosition);
                    HandleNextShot();
                    telemetry.Log("Nr Mingi", mixer.artifactCount);
                    break;
            }
        }
    }

    public boolean IsNotShooting() {return !isShooting;}

    private void CompleteShot(int position)
    {
        telemetry.Log("Culoare aruncată", ColorToString(mixer.artifacte[position]));
        mixer.RemoveArtifact(position);
        ResetTimer();
    }


    private void HandleNextShot()
    {
        if (mixer.IsEmpty())
        {
            ResetShooter();
            shootingAllowed = false;
            arrangedIndex = 0;
            shootType = ShootingState.None;
        }
        else if(shootType == ShootingState.Green || shootType == ShootingState.Purple)
        {

            if((shootType == ShootingState.Green && mixer.GetCountGreen() == 0) || (shootType == ShootingState.Purple && mixer.GetCountPurple() == 0))
            {
                shootType = ShootingState.None;
                shootingAllowed = false;
                ResetShooter();
            }
            else
            {
                shootingAllowed = false; // Oprește shooting-ul curent
                isShooting = false;      // Permite reapăsarea butonului
            }
        }
        else arrangedIndex++;

        if(shootType != ShootingState.Green && shootType != ShootingState.Purple)
            isShooting = false;
    }

    public boolean GetShootingAllow(){return shootingAllowed;};


    private void ResetShooter()
    {
        StopShooterMotors();
        preparingLaunch = false;
        if (intake.IsForward())
            intake.SetPowerMax();
        mixer.ResetServoPosition();
        shootingAllowed = false;
        arrangedIndex = 0;
    }

    private int GetShootingState()
    {
        if(isShooting)
        {
            double time = runtime.seconds();
            if (time >= 1 && time < 1.35)
                return 1;
            else if (time >= 1.35 && time < 1.5)
                return 2;
            else if (time >= 1.5)
                return 3;
        }
        return 0;
    }
    private boolean CanShootArranged() // verifica daca poate trage in ordinea data de husky
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
            SetShooterVelocity(motorPower);
        }
    }
}
