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
import org.firstinspires.ftc.teamcode.pedroPathing.ShootingTrajectory;

import static org.firstinspires.ftc.teamcode.Utils.ColorToString;

public class Shooter implements Subsystem{
    private ButtonReader Aruncare;
    private ButtonReader ThrowGreen, ThrowPurple;
    private ButtonReader OvverideShooting;
    private ButtonReader VelocityChange;
    private final GamepadEx ct1, ct2;
    private final double initialPosition = 0.3;
    private final double finalPosition = 0.47;
    private ElapsedTime runtime = new ElapsedTime();
    boolean isShooting = false;
    private int arrangedIndex = 0;
    private int currentShootingPosition = 0; // Poziția curentă care se aruncă
    private final Mixer mixer;
    private final Intake intake;
    private final TelemetryCustom telemetry;
    private final Husky husky;
    private final RobotAllignment robotAllignment;
    private boolean preparingLaunch = false;
    private boolean shootingAllowed = false;
    private Servo ServoRidicare = null;
    private double constDist = 0.0;
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

    // Constructor for TeleOp with gamepads
    public Shooter(TelemetryCustom tl, Mixer mixer,Intake intk,Husky husky,RobotAllignment robotAllignment,GamepadEx ct1, GamepadEx ct2)
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
    public Shooter(TelemetryCustom tl, Mixer mixer, Intake intk, Husky husky, RobotAllignment robotAllignment)
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
             OvverideShooting = new ButtonReader(ct2, GamepadKeys.Button.X);
         }

         ServoRidicare = hardwareMap.get(Servo.class, "ServoRidicare");
         MotorAruncare1 = hardwareMap.get(DcMotorEx.class, "MotorAruncare1");
         MotorAruncare2 = hardwareMap.get(DcMotorEx.class, "MotorAruncare2");
     }
     private void ChangeVelocity()
     {
         if (VelocityChange != null && VelocityChange.wasJustPressed())
         {
             if (motorPower == highVelocity)
             {
                 motorPower = lowVelocity;
                 SetShooterVelocity(motorPower);
             }
             else if(motorPower == lowVelocity)
             {
                 motorPower = highVelocity;
                 SetShooterVelocity(motorPower);
             }
             else
             {
                 motorPower = lowVelocity;
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

        // CRITICAL: Ensure motors start at 0 velocity (don't retain previous commands)
        MotorAruncare1.setVelocity(0);
        MotorAruncare2.setVelocity(0);

        ServoRidicare.setDirection(Servo.Direction.FORWARD);
        ServoRidicare.setPosition(initialPosition);
    }

    public double GetCurentPower() {return motorPower;}
    public ShootingState GetShootingType(){return shootType;};

    public void Run()
    {
        ReadButtons();
        constDist = robotAllignment.GetDistanceToTarget();
        // Only process button inputs if buttons exist (teleop mode)
        if (Aruncare != null && Aruncare.wasJustPressed() && !mixer.IsEmpty())
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
            mixer.SetArtifacts(Color.Green, Color.Green, Color.Purple);
            shootType = ShootingState.Unarranged;
            isShooting = false;
            arrangedIndex = 0;
            shootingAllowed = true;
            preparingLaunch = false;
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

    public double GetMotorPower() {return motorPower;}

    public void StopShooterMotors(){
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
            SetMotorPower();
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
            SetMotorPower();
            currentShootingPosition = mixer.GetColorPosition(husky.artifactOrder[arrangedIndex]);
            telemetry.Log("Target Color", ColorToString(husky.artifactOrder[arrangedIndex]));
        }
        ShootColor();
    }

    private void PurpleGreen()
    {
        if (preparingLaunch && !isShooting && !mixer.IsEmpty())
        {
            SetMotorPower();
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
            SetMotorPower();
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
    public boolean CanShootArranged() // verifica daca poate trage in ordinea data de husky
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

    private void SetMotorPower()
    {
        motorPower = 156.941*constDist*constDist*constDist-1305.23*constDist*constDist+3731.892*constDist-2203.82786;
        SetShooterVelocity(motorPower);

    } // Example linear relation

    // ==================== AUTONOMOUS SHOOTING - SIMPLIFIED ====================

    private boolean autoShooting = false;
    private int autoShootingState = 0; // 0=idle, 1=motor_spin, 2=position_ball, 3=push, 4=retract, 5=wait_reset
    private int autoCurrentPosition = 0; // Track which ball position we're shooting
    private boolean ballPositioned = false; // Flag to track if we've positioned the ball in this cycle
    private ElapsedTime autoTimer = new ElapsedTime();
    private int indexCount = 0;
    private boolean arrangedShooting = false;

    /**
     * SIMPLIFIED autonomous shooting - call this in loop()
     * Returns true when done shooting
     */
    public boolean AutoShoot()
    {
        if (!autoShooting) return true; // Not shooting, done

        double elapsed = autoTimer.seconds();

        switch (autoShootingState)
        {
            case 1: // Spin up motors (0-1.0s) - WAIT for motors to reach speed
                if (elapsed > 1.0)
                {
                    autoShootingState = 2;
                    autoTimer.reset();
                    if(CanShootArranged())
                    {
                        arrangedShooting = true;
                        telemetry.Log("Auto Shoot", "✅ ARRANGED shooting enabled!");
                    }
                    else
                    {
                        telemetry.Log("Auto Shoot", "🔀 UNORDERED shooting (no arrangement)");
                    }
                }
                telemetry.Log("Auto Shoot", "Spinning motors... " + String.format("%.1fs", elapsed));
                break;

            case 2: // Position ball (find and position - 0.3s wait)
                if (!ballPositioned)
                {

                    if(arrangedShooting)
                    {
                        autoCurrentPosition = mixer.GetColorPosition(husky.artifactOrder[arrangedIndex]);
                        telemetry.Log("Auto Shoot", "🎯 ARRANGED: Shooting " +
                            ColorToString(husky.artifactOrder[arrangedIndex]) + " (index: " + arrangedIndex + ")");
                    }
                    else
                    {
                        if (mixer.GetCountPurple() > 0)
                            autoCurrentPosition = mixer.GetColorPosition(org.firstinspires.ftc.teamcode.Color.Purple);
                        else if (mixer.GetCountGreen() > 0)
                            autoCurrentPosition = mixer.GetColorPosition(org.firstinspires.ftc.teamcode.Color.Green);
                        else
                            autoCurrentPosition = 0; // Fallback to first position

                        // CRITICAL: Validate position is within bounds (0, 1, or 2)
                        if (autoCurrentPosition < 0 || autoCurrentPosition > 2) {
                            autoCurrentPosition = 0;
                            telemetry.Log("Auto Shoot", "⚠️ Invalid position, using 0");
                        }
                        telemetry.Log("Auto Shoot", "🔀 UNORDERED: Shooting pos " + autoCurrentPosition);
                    }


                    // GUARANTEE: Set mixer position with validated index
                    mixer.SetPozition(artPoz[autoCurrentPosition]);
                    ballPositioned = true; // Mark as positioned

                    telemetry.Log("Auto Shoot", "🔄 Mixer moving to pos " + autoCurrentPosition);
                    telemetry.Log("Mixer Target", "Servo: " + artPoz[autoCurrentPosition]);
                }

                // CRITICAL: Wait 0.3s for mixer servo to reach position!
                if (elapsed > 0.4)
                {
                    autoShootingState = 3;
                    autoTimer.reset();
                    telemetry.Log("Auto Shoot", "✓ Mixer positioned!");
                }
                else
                {
                    telemetry.Log("Auto Shoot", String.format("⏳ Mixer wait %.2fs/0.3s", elapsed));
                }
                break;

            case 3: // Push ball (0.2s)
                SetPositionLever(finalPosition);
                if (elapsed > 0.3)  // FIXED: Was 0.3s, now 0.2s
                {
                    autoShootingState = 4;
                    autoTimer.reset();
                }
                telemetry.Log("Auto Shoot", "⬆️ Pushing...");
                break;

            case 4: // Retract lever (0.2s)
                SetPositionLever(initialPosition);
                if (elapsed > 0.3)  // FIXED: Was 0.3s, now 0.2s
                {
                    autoShootingState = 5;
                    autoTimer.reset();
                }
                telemetry.Log("Auto Shoot", "⬇️ Retracting...");
                break;

            case 5: // Wait and reset (0.3s) - CRITICAL! Same as teleop
                if (elapsed > 0.4)
                {
                    // Remove the ball we just shot
                    mixer.RemoveArtifact(autoCurrentPosition);

                    if (mixer.IsEmpty())
                    {
                        // All done, stop motors and reset everything
                        arrangedIndex = 0;
                        arrangedShooting = false;
                        StopShooterMotors();
                        autoShooting = false;
                        autoShootingState = 0;
                        ballPositioned = false;
                        mixer.ResetServoPosition();
                        telemetry.Log("Auto Shoot", "✓ COMPLETE!");
                        return true; // Done!
                    }
                    else
                    {
                        // More balls, shoot next one after delay
                        if (arrangedShooting) {
                            arrangedIndex++; // CRITICAL: Increment for next ball in order!
                        }
                        ballPositioned = false; // Reset flag for next ball!
                        autoShootingState = 2; // Go to position next ball
                        autoTimer.reset();
                        telemetry.Log("Auto Shoot", "➡️ Next ball (index: " + arrangedIndex + ")");
                    }
                }
                else
                {
                    telemetry.Log("Auto Shoot", "Waiting... " + String.format("%.1fs", elapsed));
                }
                break;
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
            autoShootingState = 1;
            autoTimer.reset();
            arrangedIndex = 0; // CRITICAL: Reset for fresh start!
            arrangedShooting = false; // Will be set in State 1 if CanShootArranged()

            // CRITICAL: Start motors IMMEDIATELY when called!
            SetShooterVelocity(motorPower);

            telemetry.Log("Auto Shoot", "🚀 STARTED! Motors spinning at " + motorPower);
        }
        else if (mixer.IsEmpty())
        {
            telemetry.Log("Auto Shoot", "⚠️ Cannot start - mixer empty!");
        }
        else if (autoShooting)
        {
            telemetry.Log("Auto Shoot", "⚠️ Already shooting!");
        }
    }

    /**
     * Check if autonomous shooting is active
     */
    public boolean IsAutoShooting()
    {
        return autoShooting;
    }

    // ==================== PUBLIC METHODS FOR AUTONOMOUS ====================

    /**
     * Start shooting sequence for autonomous mode
     * Call this ONCE to trigger shooting
     */
    public void StartShooting()
    {
        if (!mixer.IsEmpty())
        {
            // Set shooting type
            if(CanShootArranged())
            {
                shootType = ShootingState.Arranged;
                // For arranged, get first color position
                currentShootingPosition = mixer.GetColorPosition(husky.artifactOrder[0]);
            }
            else
            {
                shootType = ShootingState.Unarranged;
                // For unarranged, find first available ball
                if(mixer.GetCountPurple() > 0)
                    currentShootingPosition = mixer.GetColorPosition(org.firstinspires.ftc.teamcode.Color.Purple);
                else if(mixer.GetCountGreen() > 0)
                    currentShootingPosition = mixer.GetColorPosition(org.firstinspires.ftc.teamcode.Color.Green);
                else
                    currentShootingPosition = 0; // Fallback to first position
            }

            // CRITICAL: Set ALL necessary variables for shooting to work!
            shootingAllowed = true;
            arrangedIndex = 0;
            isShooting = false; // Must be false for ShootColor() to trigger

            // FORCE preparingLaunch = true (remove the if check!)
            preparingLaunch = true;
            SetShooterVelocity(motorPower);

            telemetry.Log("Autonomous Shooting", "STARTED");
            telemetry.Log("shootType", shootType);
            telemetry.Log("currentShootingPosition", currentShootingPosition);
            telemetry.Log("preparingLaunch", "TRUE");
            telemetry.Log("shootingAllowed", "TRUE");
        }
        else
        {
            telemetry.Log("Cannot shoot", "Mixer is empty!");
        }
    }

    public void VelocityOverride()
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
