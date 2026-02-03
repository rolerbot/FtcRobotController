package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.Utils.ColorToString;

public class Shooter implements Subsystem
{
    private ButtonReader Aruncare;
    private ButtonReader ThrowGreen, ThrowPurple;
    private ButtonReader OvverideShooting;
    private ButtonReader VelocityChange;
    private final GamepadEx ct1, ct2;

    private final double initialPosition = 0.257;
    private final double finalPosition = 0.48 - 0.03;
    private final double hoodInitialPosition = 0.5;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean isShooting = false;
    private int arrangedIndex = 0;
    private int currentShootingPosition = 0;
    private final Mixer mixer;
    private final Intake intake;
    private final TelemetryCustom telemetry;
    private final Husky husky;
    private final RobotAlignment robotAllignment;

    private boolean shootingAllowed = false;

    private Servo ServoRidicare = null;
    public Servo ServoHood = null;

    private double constDist = 0.0;
    private int caseSwitch = 0;

    private DcMotorEx MotorAruncare1 = null;
    private DcMotorEx MotorAruncare2 = null;

    private ShooterVoltageHelper voltageHelper = null;

    private final double BASE_SHOOTER_F = 13.9;
    private double shooterF = BASE_SHOOTER_F;
    private final double shooterP = 0.15;
    private final double shooterI = 0.0;
    private final double shooterD = 8.0;

    private double motorPower = 1500;
    private double lastMotorPower = 1500;
    private ElapsedTime brakingTimer = new ElapsedTime();
    private boolean isBraking = false;

    private boolean isLong = false;
    private boolean autoShooting = false;
    private boolean isAutoShooting = false;

    private ShootingState shootType = ShootingState.None;

    private final double offsetPosition = 0.1289;
    private final double initialPosMixer = 0.0317;

    private final double[] artPoz = {
            initialPosMixer + 3 * offsetPosition,
            initialPosMixer + 5 * offsetPosition,
            initialPosMixer + offsetPosition
    };

    private boolean autoSpinUpBoost = false;
    private ElapsedTime autoBoostTimer = new ElapsedTime();

    public Shooter(TelemetryCustom tl, Mixer mixer,Intake intk,Husky husky,RobotAlignment robotAllignment,GamepadEx ct1, GamepadEx ct2)
    {
        this.ct1 = ct1;
        this.ct2 = ct2;
        this.mixer = mixer;
        this.intake = intk;
        this.telemetry = tl;
        this.husky = husky;
        this.robotAllignment = robotAllignment;
    }

    public Shooter(TelemetryCustom tl, Mixer mixer, Intake intk, Husky husky, RobotAlignment robotAllignment, boolean isLong)
    {
        this.ct1 = null;
        this.ct2 = null;
        this.mixer = mixer;
        this.intake = intk;
        this.telemetry = tl;
        this.husky = husky;
        this.robotAllignment = robotAllignment;
        this.isLong = isLong;
        this.isAutoShooting = true;
    }
    public void LinkComponents(HardwareMap hardwareMap)
    {
        if (ct1 != null && ct2 != null)
        {
            Aruncare = new ButtonReader(ct1, GamepadKeys.Button.A);
            ThrowGreen = new ButtonReader(ct1, GamepadKeys.Button.Y);
            ThrowPurple = new ButtonReader(ct1, GamepadKeys.Button.X);
            VelocityChange = new ButtonReader(ct2, GamepadKeys.Button.B);
            OvverideShooting = new ButtonReader(ct2, GamepadKeys.Button.A);
        }

        ServoHood = hardwareMap.get(Servo.class, "ServoHood");
        ServoRidicare = hardwareMap.get(Servo.class, "ServoRidicare");
        MotorAruncare1 = hardwareMap.get(DcMotorEx.class, "MotorAruncare1");
        MotorAruncare2 = hardwareMap.get(DcMotorEx.class, "MotorAruncare2");
    }

    public void Initialize(HardwareMap hwMap)
    {
        LinkComponents(hwMap);

        voltageHelper = new ShooterVoltageHelper(hwMap, BASE_SHOOTER_F);

        ForceUpdateShooterF();

        MotorAruncare1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorAruncare1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorAruncare1.setDirection(DcMotorSimple.Direction.FORWARD);

        MotorAruncare2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorAruncare2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorAruncare2.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorAruncare1.setVelocity(0);
        MotorAruncare2.setVelocity(0);

        ServoRidicare.setDirection(Servo.Direction.FORWARD);
        ServoRidicare.setPosition(initialPosition);

        ServoHood.setDirection(Servo.Direction.FORWARD);
        ServoHood.setPosition(hoodInitialPosition);

        telemetry.Log("Shooter Init", String.format("Base F: %.2f @ 12V (voltage-compensated)", BASE_SHOOTER_F));
    }
    public ShootingState GetShootingType(){return shootType;};

    public boolean IsShootingStateNone(){return shootType == ShootingState.None;};
    public void TriggerAutonomousShooting()
    {
        if (!mixer.IsEmpty())
        {
            shootingAllowed = true;
            if (CanShootArranged())
            {
                shootType = ShootingState.Arranged;
                arrangedIndex = 0;
            }
            else
            {
                shootType = ShootingState.Unarranged;
                arrangedIndex = 0;
            }
            telemetry.Log("Auto ShootType:", shootType);
        }
    }

    public void Run()
    {
        UpdateVoltageCompensation();

        if (autoSpinUpBoost && autoBoostTimer.milliseconds() > 1800)
        {
            autoSpinUpBoost = false;

            MotorAruncare1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MotorAruncare2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
            MotorAruncare1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            MotorAruncare2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            PrepareLaunch();
            SetShooterVelocity(motorPower);

            telemetry.Log("✓ Raw→Velocity", String.format("Target: %.0f RPM @ %.2fm", motorPower, constDist));
        }

        ReadButtons();
        if (Aruncare != null && Aruncare.wasJustPressed() && !mixer.IsEmpty())
        {
            shootingAllowed = true;
            if (CanShootArranged())
            {
                shootType = ShootingState.Arranged;
                arrangedIndex = 0;
            }
            else
            {
                shootType = ShootingState.Unarranged;
                arrangedIndex = 0;
            }
            telemetry.Log("ShootType:", shootType);
        }
        else if(ThrowGreen != null && ThrowGreen.wasJustPressed() && !mixer.IsEmpty())
        {
            shootType = ShootingState.Green;
            shootingAllowed = true;
        }
        else if(ThrowPurple != null && ThrowPurple.wasJustPressed() && !mixer.IsEmpty())
        {
            shootType = ShootingState.Purple;
            shootingAllowed = true;
        }
        else if(OvverideShooting != null && OvverideShooting.wasJustPressed())
        {
            mixer.SetArtifacts();
            shootType = ShootingState.Arranged;
            isShooting = false;
            arrangedIndex = 0;
            shootingAllowed = true;
        }

        if (shootingAllowed)
            Shooting();
        PrepareLaunch();
        HoodPosition();
    }
    private void ReadButtons()
    {
        if (OvverideShooting != null && VelocityChange != null &&
                Aruncare != null && ThrowGreen != null && ThrowPurple != null)
        {
            OvverideShooting.readValue();
            VelocityChange.readValue();
            Aruncare.readValue();
            ThrowGreen.readValue();
            ThrowPurple.readValue();
        }
    }
    public void SetPositionLever(double position){
        ServoRidicare.setPosition(position);
    }
    private void ResetTimer(){
        runtime.reset();
    }

    public void SetShooterVelocity(double velocity)
    {
        double currentVelocity = MotorAruncare1.getVelocity();
        double velocityDrop = lastMotorPower - velocity;
        double velocityError = currentVelocity - velocity;

        if (autoSpinUpBoost)
        {
            return;
        }

        // Brake on ANY downward velocity command if we're overshooting
        if (velocityDrop > 7 && velocityError > 7)
        {
            if (!isBraking)
            {
                brakingTimer.reset();
                isBraking = true;
                telemetry.Log("⚡ BRAKE START", String.format("%.0f → %.0f RPM", currentVelocity, velocity));
            }

            if (brakingTimer.milliseconds() < 500)
            {
                double brakePower = -Math.min(velocityError * 2.6, 1200);

                MotorAruncare1.setVelocity(brakePower);
                MotorAruncare2.setVelocity(brakePower);

                telemetry.Log("⚡ BRAKING", String.format("%.0f RPM @ %.0fms", brakePower, brakingTimer.milliseconds()));
                return;
            }
            else
            {
                isBraking = false;
                telemetry.Log("✓ BRAKE END", String.format("Now: %.0f RPM", currentVelocity));
            }
        }
        else if (isBraking && velocityError <= 5)
        {
            isBraking = false;
            telemetry.Log("✓ BRAKE DONE", "Target reached");
        }

        if (isBraking)
        {
            return;
        }

        MotorAruncare1.setVelocity(velocity);
        MotorAruncare2.setVelocity(velocity);
        lastMotorPower = velocity;
    }

    private void UpdateVoltageCompensation()
    {
        if (voltageHelper == null) return;

        double newF = voltageHelper.GetCompensatedF(BASE_SHOOTER_F);

        if (Math.abs(newF - shooterF) > 0.01)
        {
            shooterF = newF;
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
            MotorAruncare1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            MotorAruncare2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            telemetry.Log("F Voltage-Comp", String.format("V=%.2fV → F=%.2f", voltageHelper.GetFilteredVoltage(), shooterF));
        }
    }
    public double GetMotorPower() {return motorPower;}

    public void StopShooterMotors() {
        SetShooterVelocity(0);
        lastMotorPower = 0;
        isBraking = false;
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
        if (!isShooting && !mixer.IsEmpty())
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
        if (!isShooting && !mixer.IsEmpty())
        {
            currentShootingPosition = mixer.GetColorPosition(husky.artifactOrder[arrangedIndex]);
            telemetry.Log("Target Color", ColorToString(husky.artifactOrder[arrangedIndex]));
        }
        ShootColor();
    }

    private void PurpleGreen()
    {
        if (!isShooting && !mixer.IsEmpty())
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
        if (!isShooting && !mixer.IsEmpty() && currentShootingPosition >= 0)
        {
            isShooting = true;
            caseSwitch = 0;
            ResetTimer();
            mixer.SetPozition(artPoz[currentShootingPosition]);
            telemetry.Log("Shooting position", currentShootingPosition);
        }

        if (isShooting)
        {
            switch (StateAutoTeleop())
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

    private int StateAutoTeleop()
    {
        if (isAutoShooting)
            return GetShootingStateAuto();
        return GetShootingState();
    }

    public boolean IsNotShooting() {return !isShooting;}

    private void CompleteShot(int position)
    {
        telemetry.Log("Culoare aruncată", ColorToString(mixer.artifacte[position]));
        mixer.RemoveArtifact(position);
        ResetTimer();
    }

    public boolean IsAutoShooting() { return autoShooting;}

    private void HandleNextShot()
    {
        if (mixer.IsEmpty())
        {
            ResetShooter();
            autoShooting = false;
            shootingAllowed = false;
            arrangedIndex = 0;
            shootType = ShootingState.None;
            isShooting = false;
        }
        else if(shootType == ShootingState.Green || shootType == ShootingState.Purple)
        {

            if((shootType == ShootingState.Green && mixer.GetCountGreen() == 0) || (shootType == ShootingState.Purple && mixer.GetCountPurple() == 0))
            {
                shootType = ShootingState.None;
                shootingAllowed = false;
                ResetShooter();
                isShooting = false;
            }
            else
            {
                shootingAllowed = false;
                isShooting = false;
            }
        }
        else
        {
            arrangedIndex++;
            isShooting = false;
        }
    }

    public boolean GetShootingAllow(){return shootingAllowed;};


    private void ResetShooter()
    {
        StopShooterMotors();
        if (intake.IsMoving())
            intake.SetPowerMax();
        mixer.ResetServoPosition();
        shootingAllowed = false;
        arrangedIndex = 0;
    }

    private double GetTimeDist()
    {
        double batteryVoltage = voltageHelper != null ? voltageHelper.GetFilteredVoltage() : 12.0;

        if(constDist > 3.2 && batteryVoltage < 12.3)
            return 0.31;
        else if (constDist < 3.2 && batteryVoltage < 12.3)
            return 0.2;
        return 0.15;
    }

    private int GetShootingStateOld()
    {
        if(!isShooting)
            return 0;
        double timerDependingOnDist  = GetTimeDist();
        double time = runtime.seconds();
        double currentVelocity = MotorAruncare1.getVelocity();
        double velocityTolerance = 5;
        boolean motorsReady = Math.abs(currentVelocity - motorPower) <= velocityTolerance;

        switch(caseSwitch)
        {
            case 0:
                if(time >= 0.25  && ( motorsReady || time >= timerDependingOnDist))
                {
                    caseSwitch = 1;
                    ResetTimer();
                }
                return 0;

            case 1:
                if(time >= 0.12)
                {
                    caseSwitch = 2;
                    ResetTimer();
                }
                return 1;

            case 2:
                if(time >= 0.12)
                {
                    caseSwitch = 3;
                    ResetTimer();
                }
                return 2;

            case 3:
                caseSwitch = 0;
                return 3;

            default:
                caseSwitch = 0;
                return 0;
        }
    }

    private int GetShootingState()
    {
        if(!isShooting)
            return 0;
        double timerDependingOnDist  = GetTimeDist();
        double time = runtime.seconds();

        switch(caseSwitch)
        {
            case 0:
                if(time >= timerDependingOnDist)
                {
                    caseSwitch = 1;
                    ResetTimer();
                }
                return 0;

            case 1:
                if(time >= 0.1)
                {
                    caseSwitch = 2;
                    ResetTimer();
                }
                return 1;

            case 2:
                if(time >= 0.1  )
                {
                    caseSwitch = 3;
                    ResetTimer();
                }
                return 2;

            case 3:
                caseSwitch = 0;
                return 3;

            default:
                caseSwitch = 0;
                return 0;
        }
    }

    private int GetShootingStateAuto()
    {
        if(!isShooting)
            return 0;
        double time = runtime.seconds();
        double timeout = 0.4;
        double timerLever = 0.1;
        if(mixer.GetArtifactCount() == 1)
            timerLever = 0.16;
        if(!isLong)
            timeout = 0.2;

        switch(caseSwitch)
        {
            case 0:
                if(time >= timeout)
                {
                    caseSwitch = 1;
                    ResetTimer();
                }
                return 0;

            case 1:
                if(time >= 0.12)
                {
                    caseSwitch = 2;
                    ResetTimer();
                }
                return 1;

            case 2:
                if(time >= timerLever)
                {
                    caseSwitch = 3;
                    ResetTimer();
                }
                return 2;

            case 3:
                caseSwitch = 0;
                return 3;

            default:
                caseSwitch = 0;
                return 0;
        }
    }
    public boolean CanShootArranged()
    {
        if (mixer.IsEmpty())
            return false;
        return mixer.GetCountGreen() == 1 && mixer.GetCountPurple() == 2 && husky.GetID() != 0;
    }
    private void PrepareLaunch()
    {
        if (robotAllignment != null) {
            constDist = robotAllignment.GetDistanceToTarget();
        } else {
            if(isLong)
                constDist = 3.97;
            else constDist = 2.15;
        }
        SetMotorPower();
    }

    private void SetMotorPower()
    {
        double dist = constDist * 100;
        if(constDist > 2 && constDist < 5)
            motorPower = 0.000017267*dist*dist*dist - 0.0106542*dist*dist + 2.96418*dist + 970;
        else if(constDist >= 5)
            motorPower = 1500;
        else motorPower = 1100;

        if (!autoSpinUpBoost)
        {
            SetShooterVelocity(motorPower);
        }
    }

    private void HoodPosition()
    {
        if (constDist > 2 && ServoHood.getPosition() != 0.5294)
            ServoHood.setPosition(0.5294);
        else if (constDist <= 2 && ServoHood.getPosition() != 0.5)
            ServoHood.setPosition(0.5);
    }

    public double GetVelocityCurrent() {return MotorAruncare1.getVelocity();}

    public double GetVelocityTarget() {return motorPower;}

    public boolean AutoShoot()
    {
        if (!autoShooting) return true;

        if (shootType == ShootingState.Arranged)
            Ordered();
        else
            Unordered();

        if (mixer.IsEmpty())
        {
            autoShooting = false;
            shootingAllowed = false;
            shootType = ShootingState.None;
            arrangedIndex = 0;
            ResetShooter();
            telemetry.Log("Auto Shoot", "✓ COMPLETE!");
            return true;
        }

        return false;
    }

    public void StartAutoShoot()
    {
        if (!mixer.IsEmpty() && !autoShooting)
        {
            autoShooting = true;
            shootingAllowed = true;
            isShooting = false;
            arrangedIndex = 0;

            if (CanShootArranged())
                shootType = ShootingState.Arranged;
            else
                shootType = ShootingState.Unarranged;

            telemetry.Log("⚡ Auto Shoot", String.format("Starting @ %.0f RPM", MotorAruncare1.getVelocity()));
        }
    }

    public void StartAutoBoost()
    {
        if (!autoSpinUpBoost)
        {
            autoSpinUpBoost = true;
            autoBoostTimer.reset();

            MotorAruncare1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            MotorAruncare2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            MotorAruncare1.setPower(1.0);
            MotorAruncare2.setPower(1.0);

            telemetry.Log("⚡ RAW POWER", "100% voltage boost!");
        }
    }

    public void ForceUpdateShooterF()
    {
        if (voltageHelper == null) return;

        shooterF = voltageHelper.ForceUpdate(BASE_SHOOTER_F);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
        MotorAruncare1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        MotorAruncare2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.Log("Init F Comp", String.format("V=%.2fV → F=%.2f", voltageHelper.GetRawVoltage(), shooterF));
    }

    public double GetBatteryVoltage()
    {
        if (voltageHelper == null) return 12.0;
        return voltageHelper.GetFilteredVoltage();
    }
}