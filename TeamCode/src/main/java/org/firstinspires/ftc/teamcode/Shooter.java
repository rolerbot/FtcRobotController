package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Utils.ColorToString;

public class Shooter implements Subsystem{
    private ButtonReader Aruncare;
    private ButtonReader ThrowGreen, ThrowPurple;
    private ButtonReader OvverideShooting;
    private ButtonReader VelocityChange;
    private final GamepadEx ct1, ct2; // final - initialpos = 0.189
    private final double initialPosition = 0.261; // 0.291
    private final double finalPosition = 0.48 - 0.03;
    private final double hoodInitialPosition = 0.5; //0.5294 -45 grade
    private ElapsedTime runtime = new ElapsedTime();
    boolean isShooting = false;
    private int arrangedIndex = 0;
    private int currentShootingPosition = 0; // Poziția curentă care se aruncă
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
    /// Motor Aruncare
    private DcMotorEx MotorAruncare1 = null;
    private DcMotorEx MotorAruncare2 = null;
    private ShooterVoltageHelper voltageHelper = null;  // Voltage compensation helper

    // PIDF Configuration - valori din tuning LA 12V
    private final double BASE_SHOOTER_F = 13.9;  // F tunat la 12V (V_REF)
    private double shooterF = BASE_SHOOTER_F;    // Current F (compensated)
    private final double shooterP = 0.02;
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

    // Constructor for TeleOp with gamepads
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

    // Constructor for Autonomous without gamepads
    public Shooter(TelemetryCustom tl, Mixer mixer, Intake intk, Husky husky, RobotAlignment robotAllignment)
    {
        this.ct1 = null;
        this.ct2 = null;
        this.mixer = mixer;
        this.intake = intk;
        this.telemetry = tl;
        this.husky = husky;
        this.robotAllignment = robotAllignment;
    }
     public void LinkComponents(HardwareMap hardwareMap)
     {
         // Only initialize button readers if gamepads exist (teleop mode)
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

        // Initialize voltage compensation helper
        voltageHelper = new ShooterVoltageHelper(hwMap, BASE_SHOOTER_F);

        // Get initial compensated F
        shooterF = voltageHelper.GetCompensatedF(BASE_SHOOTER_F);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(shooterP, shooterI, shooterD, shooterF);

        MotorAruncare1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorAruncare1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorAruncare1.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorAruncare1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        MotorAruncare2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorAruncare2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorAruncare2.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorAruncare2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // CRITICAL: Ensure motors start at 0 velocity (don't retain previous commands)
        MotorAruncare1.setVelocity(0);
        MotorAruncare2.setVelocity(0);

        ServoRidicare.setDirection(Servo.Direction.FORWARD);
        ServoRidicare.setPosition(initialPosition);

        ServoHood.setDirection(Servo.Direction.FORWARD);
        ServoHood.setPosition(hoodInitialPosition);

        telemetry.Log("Shooter Init", String.format("Base F: %.2f @ 12V (voltage-compensated)", BASE_SHOOTER_F));
    }
    public ShootingState GetShootingType(){return shootType;};
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
        // Update F based on filtered battery voltage (every 500ms)
        UpdateVoltageCompensation();

        ReadButtons();
        // Only process button inputs if buttons exist (teleop mode)
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
        // Only read buttons if they exist (teleop mode)
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

    // Controlează motoarele cu velocity (RPM) în loc de power
    public void SetShooterVelocity(double velocity)
    {
        MotorAruncare1.setVelocity(velocity);
        MotorAruncare2.setVelocity(velocity);
    }

    /**
     * Update F based on filtered battery voltage
     * Uses FlywheelVoltageHelper to compensate for battery drain
     * Updates PIDF coefficients only when F changes significantly
     */
    private void UpdateVoltageCompensation()
    {
        if (voltageHelper == null) return;

        double newF = voltageHelper.GetCompensatedF(BASE_SHOOTER_F);

        // Only update motors if F changed significantly (reduce motor config spam)
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

    public void StopShooterMotors()
    {
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
            caseSwitch = 0; // Reset state machine
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
                shootingAllowed = false; // Oprește shooting-ul curent
                isShooting = false;      // Permite reapăsarea butonului
            }
        }
        else
        {
            arrangedIndex++;
            isShooting = false; // Reset for next shot in arranged/unarranged mode
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
        // Get filtered battery voltage (this is the "true" battery when consistent)
        double batteryVoltage = voltageHelper != null ? voltageHelper.GetFilteredVoltage() : 12.0;

        if(constDist > 3.2)
        {
            // If battery is low (< 12V), motors need more time to reach target velocity
            if(batteryVoltage < 12.3)
                return 0.65;  // Extra time when battery is low
            else
                return 0.55;   // Normal time at full battery
        }
        else return 0.3;  // Short distance, quick shot
    }

    private int GetShootingState()
    {
        if(!isShooting)
            return 0;
        double timerDependingOnDist  = GetTimeDist();
        double time = runtime.seconds();
        double currentVelocity = MotorAruncare1.getVelocity();
        double velocityTolerance = 5; // RPM tolerance
        boolean motorsReady = Math.abs(currentVelocity - motorPower) <= velocityTolerance;

        switch(caseSwitch)
        {
            case 0: // Waiting for motors to spin up
                if(time >= 0.25  && ( motorsReady || time >= timerDependingOnDist)) // Motors ready OR timeout
                {
                    caseSwitch = 1;
                    ResetTimer();
                }
                return 0;

            case 1: // Push lever (wait 0.3s)
                if(time >= 0.1)
                {
                    caseSwitch = 2;
                    ResetTimer();
                }
                return 1;

            case 2: // Retract lever (wait 0.2s)
                if(time >= 0.135)
                {
                    caseSwitch = 3;
                    ResetTimer();
                }
                return 2;

            case 3: // Complete shot
                caseSwitch = 0; // Reset for next shot
                return 3;

            default:
                caseSwitch = 0;
                return 0;
        }
    }

    private int GetShootingState1()
    {
        if(isShooting)
        {
            double time = runtime.seconds();
            if (time >= 0.5 && time < 0.7)
                return 1;
            else if (time >= 0.7 && time < 0.85)
                return 2;
            else if (time >= 0.85)
                return 3;
        }
        return 0;
    }
    public boolean CanShootArranged() // verifica daca poate trage in ordinea data de husky
    {
        if (mixer.IsEmpty())
            return false;
        return mixer.GetCountGreen() == 1 && mixer.GetCountPurple() == 2 && husky.GetID() != 0;
    }
    private void PrepareLaunch()
    {
        // Get distance from RobotAllignment if available, otherwise use default distance
        if (robotAllignment != null) {
            constDist = robotAllignment.GetDistanceToTarget();
        } else {
            // Default distance for autonomous (assumes mid-range shooting)
            constDist = 3.4; // meters - adjust as needed for your autonomous position
        }
        SetMotorPower();
    }

    private void SetMotorPower()
    {
        double dist = constDist * 100;
        if(constDist > 2 && constDist < 5)
            motorPower = -(2.07771/10000000)*dist*dist*dist*dist+0.000254073*dist*dist*dist-0.118373*dist*dist + 26.72314*dist - 1158.4306; //1158.4306
        else if(constDist >= 5)
            motorPower = 1700;
        else motorPower = 1100;
        SetShooterVelocity(motorPower);
    }

    private void HoodPosition()
    {
        if (constDist > 2.8 && ServoHood.getPosition() != 0.5294)
            ServoHood.setPosition(0.5294);
        else if (constDist <= 2.8 && ServoHood.getPosition() != 0.5)
            ServoHood.setPosition(0.5);
    }

    public double GetVelocityCurrent() {return MotorAruncare1.getVelocity();}

    public double GetVelocityTarget() {return motorPower;}

    // ==================== AUTONOMOUS SHOOTING ===========================================================

    private boolean autoShooting = false;

    public boolean AutoShoot()
    {
        if (!autoShooting) return true; // Not shooting, done

        // Use the same shooting logic as teleop
        if (shootType == ShootingState.Arranged)
            Ordered();
        else
            Unordered();

        // Check if shooting is complete
        if (mixer.IsEmpty())
        {
            autoShooting = false;
            shootingAllowed = false;
            shootType = ShootingState.None;
            arrangedIndex = 0;
            ResetShooter();
            telemetry.Log("Auto Shoot", "✓ COMPLETE!");
            return true; // Done!
        }

        return false; // Still shooting
    }

    /**
     * Start autonomous shooting sequence
     */
    public void StartAutoShoot()
    {
        if (!mixer.IsEmpty() && !autoShooting)
        {
            autoShooting = true;
            shootingAllowed = true;
            isShooting = false;
            arrangedIndex = 0;

            // Determine shooting type (same logic as teleop)
            if (CanShootArranged())
                shootType = ShootingState.Arranged;
            else
                shootType = ShootingState.Unarranged;
            SetShooterVelocity(motorPower);
        }
    }

    // ==================== BATTERY COMPENSATION (INIT) ==================================================

    /**
     * Force immediate battery reading and F compensation during initialization
     * No filtering - instant result for autonomous init
     */
    public void ForceUpdateShooterF()
    {
        if (voltageHelper == null) return;

        shooterF = voltageHelper.ForceUpdate(BASE_SHOOTER_F);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
        MotorAruncare1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        MotorAruncare2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.Log("Init F Comp", String.format("V=%.2fV → F=%.2f", voltageHelper.GetRawVoltage(), shooterF));
    }

    /**
     * Get current battery voltage for telemetry
     */
    public double GetBatteryVoltage()
    {
        if (voltageHelper == null) return 12.0;
        return voltageHelper.GetFilteredVoltage();
    }

}
