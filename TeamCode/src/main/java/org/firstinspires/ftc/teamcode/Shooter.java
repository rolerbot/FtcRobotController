package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.Utils.ColorToString;

public class Shooter implements Subsystem {
    private ButtonReader Aruncare;
    private ButtonReader ThrowGreen, ThrowPurple, ToggleFast;
    private final GamepadEx ct1, ct2;
    private final double initialPosition = 0.088;
    private final double finalPosition = 0.16; // Recovery from user typo -1.2
    private final double hoodInitialPosition = 0.5;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean isShooting = false;
    private int arrangedIndex = 0;
    private int currentShootingPosition = 0;
    private final Mixer mixer;
    private final TelemetryCustom telemetry;

    private boolean shootingAllowed = false;

    private Servo ServoRidicare = null;
    public Servo ServoHood = null;

    private double constDist = 0.0;
    private int caseSwitch = 0;

    private DcMotorEx MotorAruncare1 = null;
    private DcMotorEx MotorAruncare2 = null;

    private ShooterVoltageHelper voltageHelper = null;
    private LimeLight limelight;

    private final double BASE_SHOOTER_F = 13.9; // 13.9
    private double shooterF = BASE_SHOOTER_F;
    private final double shooterP = 0.15;
    private final double shooterI = 0.0;
    private final double shooterD = 8.0;

    private final double IDLE_VELOCITY_MAX = 1580;
    private double motorPower = 1580;
    private double targetShootingVelocity = 1580;
    private double lastMotorPower = 1580;
    private ElapsedTime brakingTimer = new ElapsedTime();
    private boolean isBraking = false;
    private boolean isLong = false;
    private boolean autoShooting = false;
    private boolean isAutoShooting = false;
    private boolean toggleFastMode = false;
    private double breakMultiplier = 4.2;
    private double velocityConstant = 650;
    private ShootingState shootType = ShootingState.None;
    private int fastSequenceIdx = 0;
    private boolean fastLeverPulled = false;
    private ElapsedTime totalShootTimer = new ElapsedTime();
    private double lastShootDuration = 0;
    private double fastMixerServoPos = 0;

    private double offsetPositionMixer = 0.09028;
    private final double initialPosMixer = 0.0827 + 2 * offsetPositionMixer;
    // testing

    private final int mixerOffsetEncoder = 1355;

    private final double[] artPoz = {
            initialPosMixer + 3 * offsetPositionMixer, // Slot 0
            initialPosMixer + 5 * offsetPositionMixer, // Slot 1
            initialPosMixer + offsetPositionMixer // Slot 2
    };

    private final double[] pozEncoder = {
            3 * mixerOffsetEncoder, // Target for Slot 0
            5 * mixerOffsetEncoder, // Target for Slot 1
            mixerOffsetEncoder // Target for Slot 2
    };

    private final int[] fastSlotPositions = {
            (int)(6682 - mixerOffsetEncoder),  // Slot 0 - was too soon, shifted later
            (int)(4027 - mixerOffsetEncoder),         // Slot 1 - was good, unchanged
            (int)(1273 - mixerOffsetEncoder)     // Slot 2 - went down too fast, shifted earlier
    };

    private final boolean[] fastSlotFired = new boolean[3];
    private final boolean[] fastSlotLeverUp = new boolean[3];
    private boolean fastSweepStarted = false;
    private int fastStartEnc = 0;
    /*
    4074, // Slot 0
            -1406, // Slot 1
            1228 // Slot 2
     */

    private boolean autoSpinUpBoost = false;
    private ElapsedTime autoBoostTimer = new ElapsedTime();

    public Shooter(TelemetryCustom tl, Mixer mixer, GamepadEx ct1, GamepadEx ct2, LimeLight limelight) {
        this.ct1 = ct1;
        this.ct2 = ct2;
        this.mixer = mixer;
        this.telemetry = tl;
        this.limelight = limelight;
    }

    public Shooter(TelemetryCustom tl, Mixer mixer, LimeLight limelight, boolean isLong) {
        this.ct1 = null;
        this.ct2 = null;
        this.mixer = mixer;
        this.telemetry = tl;
        this.isLong = isLong;
        this.isAutoShooting = true;
        this.limelight = limelight;
    }

    public void LinkComponents(HardwareMap hardwareMap) {
        if (ct1 != null && ct2 != null) {
            Aruncare = new ButtonReader(ct1, GamepadKeys.Button.A);
            ThrowGreen = new ButtonReader(ct1, GamepadKeys.Button.Y);
            ThrowPurple = new ButtonReader(ct1, GamepadKeys.Button.X);
            ToggleFast = new ButtonReader(ct2, GamepadKeys.Button.Y);
        }

        ServoHood = hardwareMap.get(Servo.class, "ServoHood");
        ServoRidicare = hardwareMap.get(Servo.class, "ServoRidicare");
        MotorAruncare1 = hardwareMap.get(DcMotorEx.class, "MotorAruncare1");
        MotorAruncare2 = hardwareMap.get(DcMotorEx.class, "MotorAruncare2");
    }

    public void Initialize(HardwareMap hwMap) {
        LinkComponents(hwMap);

        voltageHelper = new ShooterVoltageHelper(hwMap, BASE_SHOOTER_F);

        ForceUpdateShooterF();

        MotorAruncare1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorAruncare1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorAruncare1.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorAruncare2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorAruncare2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorAruncare2.setDirection(DcMotorSimple.Direction.FORWARD);

        MotorAruncare1.setVelocity(0);
        MotorAruncare2.setVelocity(0);

        ServoRidicare.setDirection(Servo.Direction.REVERSE);
        ServoRidicare.setPosition(initialPosition);

        ServoHood.setDirection(Servo.Direction.FORWARD);
        ServoHood.setPosition(hoodInitialPosition);

        telemetry.Log("Shooter Init", String.format("Base F: %.2f @ 12V (voltage-compensated)", BASE_SHOOTER_F));
    }

    public ShootingState GetShootingType() {
        return shootType;
    };

    public boolean IsShootingStateNone() {
        return shootType == ShootingState.None;
    };

    public void Run() {
        UpdateVoltageCompensation();

        if (autoSpinUpBoost && autoBoostTimer.milliseconds() > 900) {
            autoSpinUpBoost = false;

            MotorAruncare1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MotorAruncare2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
            MotorAruncare1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            MotorAruncare2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            // Force braking logic to trigger by setting lastMotorPower very high
            lastMotorPower = 2800;
            SetShooterVelocity(IDLE_VELOCITY_MAX);
        }

        ReadButtons();
        if (!mixer.IsMoving()) {
            if (Aruncare != null && Aruncare.wasJustPressed() && !mixer.IsEmpty()) {
                UpdateDistanceAndHood();
                CalculateShootingVelocity();
                runtime.reset();
                shootingAllowed = true;
                if (CanShootArranged()) {
                    shootType = ShootingState.Arranged;
                    arrangedIndex = 0;
                } else if (!toggleFastMode) {
                    shootType = ShootingState.Unarranged;
                    arrangedIndex = 0;
                } else {
                    shootType = ShootingState.UnarrangedFast;
                    arrangedIndex = 0;
                }
                totalShootTimer.reset();
            } else if (ThrowGreen != null && ThrowGreen.wasJustPressed() && !mixer.IsEmpty()) {
                UpdateDistanceAndHood();
                CalculateShootingVelocity();
                shootType = ShootingState.Green;
                shootingAllowed = true;
                totalShootTimer.reset();
            } else if (ThrowPurple != null && ThrowPurple.wasJustPressed() && !mixer.IsEmpty()) {
                UpdateDistanceAndHood();
                CalculateShootingVelocity();
                shootType = ShootingState.Purple;
                shootingAllowed = true;
                totalShootTimer.reset();
            }
        }

        if (ToggleFast.wasJustPressed())
            toggleFastMode = true;
        if (shootingAllowed)
            Shooting();

        MaintainVelocity();

    }

    private void ReadButtons() {
        if (Aruncare != null && ThrowGreen != null && ThrowPurple != null) {
            Aruncare.readValue();
            ThrowGreen.readValue();
            ThrowPurple.readValue();
        }
        if (ToggleFast != null) {
            ToggleFast.readValue();
        }
    }

    public void SetPositionLever(double position) {
        ServoRidicare.setPosition(position);
    }

    private void ResetTimer() {
        runtime.reset();
    }

    public void SetCustomDistanceMeters(double meters) {
        this.constDist = meters;
        // HoodPosition();
    }

    public void SetShooterVelocity(double velocity) {
        double currentVelocity = MotorAruncare1.getVelocity();
        double velocityDrop = lastMotorPower - velocity;
        double velocityError = currentVelocity - velocity;

        if (autoSpinUpBoost)
            return;

        // Pure proportional braking - no cap, just velocity error * multiplier
        if (velocityDrop > 7 && velocityError > 7) {
            if (!isBraking) {
                brakingTimer.reset();
                isBraking = true;
            }

            if (brakingTimer.milliseconds() < 250) {
                double effectiveMultiplier = breakMultiplier;
                if (constDist > 2.7) {
                    // Increased from 0.29 to 0.65 to ensure braking still works at match distances
                    effectiveMultiplier = breakMultiplier * 0.2;
                }

                double brakePower = -velocityError * effectiveMultiplier;

                MotorAruncare1.setVelocity(brakePower);
                MotorAruncare2.setVelocity(brakePower);
                return;
            } else {
                isBraking = false;
            }
        } else if (isBraking && velocityError <= 5) {
            isBraking = false;
        }

        if (isBraking) {
            return;
        }

        MotorAruncare1.setVelocity(velocity);
        MotorAruncare2.setVelocity(velocity);
        lastMotorPower = velocity;
    }

    private void UpdateVoltageCompensation() {
        if (voltageHelper == null)
            return;

        double newF = voltageHelper.GetCompensatedF(BASE_SHOOTER_F);

        if (Math.abs(newF - shooterF) > 0.01) {
            shooterF = newF;
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
            MotorAruncare1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            MotorAruncare2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        }
    }

    public double GetMotorPower() {
        return motorPower;
    }

    public void StopShooterMotors() {
        SetShooterVelocity(0);
        lastMotorPower = 0;
        isBraking = false;
    }

    private void Shooting() {
        switch (shootType) {
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
            case UnarrangedFast:
                UnorderedFast();
                break;
            default:
                break;
        }
    }

    private void UnorderedFast() {
        ShootFastColor2();
    }

    private void Unordered() {
        telemetry.Log("Unordered", "");
        if (!isShooting && !mixer.IsEmpty()) {
            if (mixer.GetCountPurple() > 0)
                currentShootingPosition = mixer.GetColorPosition(Color.Purple);
            else if (mixer.GetCountGreen() > 0)
                currentShootingPosition = mixer.GetColorPosition(Color.Green);
            telemetry.Log("ShootingPose:", currentShootingPosition);
        }
        ShootColor();
    }

    private void Ordered() {
        if (!isShooting && !mixer.IsEmpty()) {
            if (arrangedIndex >= 0 && arrangedIndex < limelight.artifactOrder.length) {
                currentShootingPosition = mixer.GetColorPosition(limelight.artifactOrder[arrangedIndex]);
                telemetry.Log("Target Color", ColorToString(limelight.artifactOrder[arrangedIndex]));
            } else {
                // If we finished the sequence but mixer isn't empty, shoot whatever is left
                currentShootingPosition = mixer.GetFirstAvailablePosition();
                telemetry.Log("Order Complete", "Shooting remaining balls");
            }
        }
        ShootColor();
    }

    private void PurpleGreen() {
        if (!isShooting && !mixer.IsEmpty()) {
            if (shootType == ShootingState.Green && mixer.GetCountGreen() > 0)
                currentShootingPosition = mixer.GetColorPosition(Color.Green);
            else if (shootType == ShootingState.Purple && mixer.GetCountPurple() > 0)
                currentShootingPosition = mixer.GetColorPosition(Color.Purple);
        }
        ShootColor();
    }

    private void ShootFastColor2() {
        if (!isShooting && !mixer.IsEmpty()) {
            isShooting = true;
            shootingAllowed = true;
            fastSweepStarted = false;
            fastStartEnc = mixer.MotorMixer.getCurrentPosition();

            for (int i = 0; i < fastSlotFired.length; i++) {
                fastSlotFired[i] = false;
                fastSlotLeverUp[i] = false;
            }

            ResetTimer();
            mixer.SetPozition(mixer.GetPosMax());
        }

        if (!isShooting) return;

        int enc = mixer.MotorMixer.getCurrentPosition();
        telemetry.Log("FastEnc", enc);
        telemetry.Log("StartEnc", fastStartEnc);
        telemetry.Log("SweepStarted", fastSweepStarted);
        telemetry.Log("Fired", fastSlotFired[0] + " " + fastSlotFired[1] + " " + fastSlotFired[2]);

        if (!fastSweepStarted) {
            if (Math.abs(enc - fastStartEnc) > 100) {
                fastSweepStarted = true;
            } else {
                return;
            }
        }

        for (int i = 0; i < fastSlotPositions.length; i++) {
            if (fastSlotFired[i]) continue;

            int target = fastSlotPositions[i];

            if (!fastSlotLeverUp[i] && enc <= target - 200) {
                SetPositionLever(finalPosition);
                fastSlotLeverUp[i] = true;
                telemetry.Log("Lever UP slot", i);
            }

                int postOffset = 800;
                if (fastSlotLeverUp[i] && enc <= target - postOffset) {
                    SetPositionLever(initialPosition);
                    fastSlotFired[i] = true;
                    telemetry.Log("Lever DOWN slot", i);
                }
        }

        if (runtime.milliseconds() > 1000) {
            isShooting = false;
            shootingAllowed = false;
            ResetShooter();
        }
    }

    private boolean HandleFastFlick(int currentEnc, int targetEnc) {
        if (!fastLeverPulled) {
            // Approach window: 250 ticks before
            if (currentEnc <= targetEnc + 250 && currentEnc >= targetEnc - 300) {
                fastLeverPulled = true;
                SetPositionLever(finalPosition); // FLICK UP
            }
        } else {
            // Passage window: 80 ticks after
            if (currentEnc <= targetEnc - 80) {
                fastLeverPulled = false;
                SetPositionLever(initialPosition); // DOWN FAST
                return true; // Done with this slot
            }
        }
        return false;
    }

    private void ShootColor() {
        if (!isShooting && !mixer.IsEmpty() && currentShootingPosition >= 0) {
            isShooting = true;
            caseSwitch = 0;
            ResetTimer();
            mixer.SetPozition(artPoz[currentShootingPosition]);
            if (shootType == ShootingState.Arranged) {
                arrangedIndex++;
            }

            telemetry.Log("Shooting position", currentShootingPosition);
        }

        if (isShooting) {
            switch (StateAutoTeleop()) {
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

    private int StateAutoTeleop() {
        // Now using the encoder-based state machine for both Auto and Teleop
        // as it's faster and reliable with through-bore encoder feedback.
        return GetShootingState();
    }

    public boolean IsNotShooting() {
        return !isShooting;
    }

    private void CompleteShot(int position) {
        telemetry.Log("Culoare aruncată", ColorToString(mixer.artifacte[position]));
        mixer.RemoveArtifact(position);
        ResetTimer();
    }

    public boolean IsAutoShooting() {
        return autoShooting;
    }

    private void HandleNextShot() {
        if (mixer.IsEmpty()) {
            ResetShooter();
            autoShooting = false;
            shootingAllowed = false;
            arrangedIndex = 0;
            shootType = ShootingState.None;
            isShooting = false;
            return;
        }

        if (shootType == ShootingState.Green || shootType == ShootingState.Purple) {
            if ((shootType == ShootingState.Green && mixer.GetCountGreen() == 0)
                    || (shootType == ShootingState.Purple && mixer.GetCountPurple() == 0)) {
                shootType = ShootingState.None;
                shootingAllowed = false;
                ResetShooter();
                isShooting = false;
                return;
            } else {
                shootingAllowed = false;
                isShooting = false;
                return;
            }
        }
        isShooting = false;
        // Check if we've shot all balls
        if (mixer.IsEmpty()) {
            // All balls shot - stop shooting
            ResetShooter();
            shootingAllowed = false;
            shootType = ShootingState.None;
            arrangedIndex = 0;
            telemetry.Log("✓ All balls shot", "Sequence complete");
        }
    }

    public boolean GetShootingAllow() {
        return shootingAllowed;
    };

    private void ResetShooter() {
        lastShootDuration = totalShootTimer.seconds();
        telemetry.Log("Last Shoot Duration", String.format("%.3f s", lastShootDuration));

        SetShooterVelocity(IDLE_VELOCITY_MAX);
        lastMotorPower = IDLE_VELOCITY_MAX;
        mixer.ResetServoPosition();
        SetPositionLever(initialPosition); // Always ensure lever is DOWN after reset
        shootingAllowed = false;
        isShooting = false;
        arrangedIndex = 0;
        telemetry.Log("✓ Shooter Reset", "Ready");
    }

    private int GetShootingState() {
        if (!isShooting)
            return 0;
        int enc = mixer.MotorMixer.getCurrentPosition();
        double time = runtime.seconds();
        double target = pozEncoder[currentShootingPosition];

        switch (caseSwitch) {
            case 0:
                // Wait for encoder to arrive at slot
                if (Math.abs(enc - target) < 50 || time > 0.32) {
                    caseSwitch = 1;
                    ResetTimer();
                }
                return 0;

            case 1:
                // Lever UP (Flick)
                if (time >= 0.12) {
                    caseSwitch = 2;
                    ResetTimer();
                }
                return 1;

            case 2:
                // Lever DOWN (Retract)
                if (time >= 0.08) {
                    caseSwitch = 3;
                    ResetTimer();
                }
                return 2;

            case 3:
                // Finished
                caseSwitch = 0;
                return 3;

            default:
                caseSwitch = 0;
                return 0;
        }
    }

    private int GetShootingStateAuto() {
        if (!isShooting)
            return 0;
        double time = runtime.seconds();
        double timeout = 0.4;
        double timerLever = 0.12;
        if (mixer.GetArtifactCount() == 1)
            timerLever = 0.16;
        if (!isLong)
            timeout = 0.32;

        switch (caseSwitch) {
            case 0:
                if (time >= timeout) {
                    caseSwitch = 1;
                    ResetTimer();
                }
                return 0;

            case 1:
                if (time >= 0.12) {
                    caseSwitch = 2;
                    ResetTimer();
                }
                return 1;

            case 2:
                if (time >= timerLever) {
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

    public boolean CanShootArranged() {
        if (mixer.IsEmpty())
            return false;
        return mixer.GetCountGreen() == 1 && mixer.GetCountPurple() == 2 && limelight.GetID() != 0;
    }

    private boolean useDynamicDistance = true;

    private void UpdateDistanceAndHood() {
        if (!isAutoShooting || useDynamicDistance) {
            double limeDist = limelight.GetDistanceToTargetPython();
            // double limeDist = limelight.GetDistanceToTarget();
            if (limeDist > 0) {
                constDist = limeDist;
                telemetry.Log("Distance to Target nou", String.format("%.2f meters", constDist));
            }
        }
        HoodPosition();
    }

    private void CalculateShootingVelocity() {
        // SAFETY: Clamp distance to reasonable field limits
        double safeDist = constDist;
        if (safeDist < 1) {
            safeDist = 1;
        } else if (safeDist > 4) {
            safeDist = 4;
        }

        double dist = safeDist * 100; // Convert to cm for polynomial
        if (safeDist > 1 && safeDist < 4)
            targetShootingVelocity = -(1.35657 / 1000000000) * dist * dist * dist * dist
                    + 0.0000229718 * dist * dist * dist
                    - 0.0166181 * dist * dist + 5.32713 * dist + velocityConstant;
        else if (safeDist >= 4)
            targetShootingVelocity = 1580;
        else
            targetShootingVelocity = 1000;

        motorPower = targetShootingVelocity;
    }

    public void CalculateShootingVelocityTelemetry() {
        // SAFETY: Clamp distance to reasonable field limits

        double dist = constDist * 100; // Convert to cm for polynomial
        telemetry.Log("Distance cm", constDist);
        telemetry.Log("HoodPos", ServoHood.getPosition());
        telemetry.Log("ConstVel", velocityConstant);
        telemetry.Log("Last Shoot Duration", String.format("%.3f s", lastShootDuration));

        double velocity;

        if (constDist > 1 && constDist < 3.3)
            velocity = -(1.35657 / 1000000000) * dist * dist * dist * dist + 0.0000229718 * dist * dist * dist
                    - 0.0166181 * dist * dist + 5.32713 * dist + velocityConstant;
        else if (constDist >= 3.3)
            velocity = 1550;
        else
            velocity = 1000;
        telemetry.Log("MixerPos", mixer.GetServoPosConstant());
        telemetry.Log("Velocity", velocity);
    }

    private void MaintainVelocity() {
        if (!autoSpinUpBoost) {
            if (shootingAllowed) {
                SetShooterVelocity(targetShootingVelocity);
            } else {
                SetShooterVelocity(IDLE_VELOCITY_MAX);
            }
        }
    }

    private void HoodPosition() {
        double calculateServoPos = hoodInitialPosition;
        double dist = constDist * 100;
        if (constDist >= 1 && constDist <= 2.9)
            calculateServoPos = (-0.000245041 * dist * dist + 0.163034 * dist + 476) / 1000;
        else if (constDist > 2.9)
            calculateServoPos = 0.5;
        ServoHood.setPosition(calculateServoPos);
    }

    public double GetVelocityCurrent() {
        return MotorAruncare1.getVelocity();
    }

    public double GetVelocityTarget() {
        if (shootingAllowed)
            return targetShootingVelocity;
        return IDLE_VELOCITY_MAX;
    }

    public boolean AutoShoot() {
        if (!autoShooting)
            return true;

        if (shootType == ShootingState.Arranged)
            Ordered();
        else
            Unordered();

        if (mixer.IsEmpty()) {
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

    public void StartAutoShoot() {
        if (!mixer.IsEmpty() && !autoShooting) {
            UpdateDistanceAndHood();
            CalculateShootingVelocity();
            autoShooting = true;
            shootingAllowed = true;
            isShooting = false;
            arrangedIndex = 0;

            if (CanShootArranged())
                shootType = ShootingState.Arranged;
            else
                shootType = ShootingState.Unarranged;

            telemetry.Log("⚡ Auto Shoot", String.format("Starting @ %.0f RPM → %.0f RPM", MotorAruncare1.getVelocity(),
                    targetShootingVelocity));
        }
    }

    public void StartAutoBoost() {
        if (!autoSpinUpBoost) {
            autoSpinUpBoost = true;
            autoBoostTimer.reset();

            MotorAruncare1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            MotorAruncare2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            MotorAruncare1.setPower(0.85); // Reduced from 1.0 to avoid excessive over-speed
            MotorAruncare2.setPower(0.85);

            telemetry.Log("⚡ RAW POWER", "100% voltage boost!");
        }
    }

    public void ForceUpdateShooterF() {
        if (voltageHelper == null)
            return;

        shooterF = voltageHelper.ForceUpdate(BASE_SHOOTER_F);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
        MotorAruncare1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        MotorAruncare2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.Log("Init F Comp", String.format("V=%.2fV → F=%.2f", voltageHelper.GetRawVoltage(), shooterF));
    }

    public double GetBatteryVoltage() {
        if (voltageHelper == null)
            return 12.0;
        return voltageHelper.GetFilteredVoltage();
    }
}