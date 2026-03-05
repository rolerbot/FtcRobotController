package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter implements Subsystem {
    private ButtonReader Aruncare;
    private ButtonReader ThrowGreen, ThrowPurple, ToggleFast;
    private DcMotorEx MotorRidicareBila = null;
    private final GamepadEx ct1, ct2;
    private final double hoodInitialPosition = 0.51;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean isShooting = false;
    private int arrangedIndex = 0;
    private int currentShootingPosition = 0;
    private final Mixer mixer;
    private final TelemetryCustom telemetry;

    private boolean shootingAllowed = false;
    public Servo ServoHood = null;

    private double constDist = 0.0;
    private int caseSwitch = 0;
    private DcMotorEx MotorAruncare = null;
    private ShooterVoltageHelper voltageHelper = null;
    private LimeLight limelight;
    private final double BASE_SHOOTER_F = 14; // 13.9
    // 2.97
    // 0.574
    private double shooterF = BASE_SHOOTER_F;
    private double shooterP = 0.15;
    private final double shooterI = 0.0;
    private final double shooterD = 8.0;

    // Tuning state
    private double[] tuningSteps = { 10.0, 1.0, 0.1, 0.01, 0.001 };
    private int tuningStepIndex = 1;
    private ButtonReader stepIncrease, Fincrease, Fdescrease, Pincrease, Pdecrease, VelocityUp, VelocityDown, HoodUp,
            HoodDown;
    private boolean tuningActive = true;
    private double tuningHoodPos = 0.51;

    private final double IDLE_VELOCITY_MAX = 1580;
    private double targetShootingVelocity = 1580;
    private double lastMotorPower = 1580;
    private ElapsedTime brakingTimer = new ElapsedTime();
    private boolean isBraking = false;
    private boolean isLong = false;
    private boolean autoShooting = false;
    private boolean isAutoShooting = false;
    private double breakMultiplier = 3;
    private double velocityConstant = 700;
    private final double TRANSFER_P = 10.0;
    private final double TRANSFER_I = 3.0;
    private final double TRANSFER_D = 0.0;
    private final double TRANSFER_F = 12.0;
    private final double TRANSFER_VELOCITY = 2800;
    private double hoodVelocityGain = 0.00017; // Tunable: 0.01 change per ~65 RPM error
    private ShootingState shootType = ShootingState.None;
    private ElapsedTime totalShootTimer = new ElapsedTime();
    private double lastShootDuration = 0;
    private int orderedCaseSwitch = 0;
    private int orderedSeqIndex = 0;
    private int orderedSlotIndex = 0;
    private double targetOrderedPos = 0;
    private double targetOrderedEncoder = 0;
    private int unorderedCaseSwitch = 0;
    private int unorderedSeqIndex = 0;
    private int unorderedSlotIndex = 0;
    private double targetUnorderedPos = 0;
    private double targetUnorderedEncoder = 0;
    private ElapsedTime pauseTimer = new ElapsedTime();
    private boolean goingDown = true;
    private double offsetPositionMixer = 0.09028;
    private final double initialPosMixer = 0.0827 + 2 * offsetPositionMixer;
    private final double mixerOffsetEncoder = 2715 / 2;

    private final double[] artPoz = {
            initialPosMixer + 3 * offsetPositionMixer, // Slot 0 (+3 offsets) -> 0.534
            initialPosMixer + 1 * offsetPositionMixer, // Slot 1 (+1 offset) -> 0.353
            initialPosMixer - 1 * offsetPositionMixer // Slot 2 (-1 offset) -> 0.173
    };

    private final double[] pozEncoder = {
            4050, // Slot 0 Target (Shot 1)
            1335, // Slot 1 Target (Shot 2) - Mathematically synced (4050 - 2*1357.5)
            -1380 // Slot 2 Target (Shot 3) - Mathematically synced (1335 - 2*1357.5)
    };

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

            stepIncrease = new ButtonReader(ct2, GamepadKeys.Button.B);
            Fincrease = new ButtonReader(ct2, GamepadKeys.Button.DPAD_LEFT);
            Fdescrease = new ButtonReader(ct2, GamepadKeys.Button.DPAD_RIGHT);
            Pincrease = new ButtonReader(ct2, GamepadKeys.Button.DPAD_UP);
            Pdecrease = new ButtonReader(ct2, GamepadKeys.Button.DPAD_DOWN);
            VelocityUp = new ButtonReader(ct2, GamepadKeys.Button.RIGHT_BUMPER);
            VelocityDown = new ButtonReader(ct2, GamepadKeys.Button.LEFT_BUMPER);
            HoodUp = new ButtonReader(ct2, GamepadKeys.Button.X);
            HoodDown = new ButtonReader(ct2, GamepadKeys.Button.A);
        }

        ServoHood = hardwareMap.get(Servo.class, "ServoHood");
        MotorAruncare = hardwareMap.get(DcMotorEx.class, "MotorAruncare");
        MotorRidicareBila = hardwareMap.get(DcMotorEx.class, "MotorRidicareBila");
    }

    public void Initialize(HardwareMap hwMap) {
        LinkComponents(hwMap);

        voltageHelper = new ShooterVoltageHelper(hwMap, BASE_SHOOTER_F);

        ForceUpdateShooterF();

        MotorAruncare.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorAruncare.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorAruncare.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorAruncare.setVelocity(0);

        ServoHood.setDirection(Servo.Direction.FORWARD);
        ServoHood.setPosition(hoodInitialPosition);

        MotorRidicareBila.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorRidicareBila.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorRidicareBila.setDirection(DcMotorSimple.Direction.FORWARD);

        PIDFCoefficients transferPIDF = new PIDFCoefficients(TRANSFER_P, TRANSFER_I, TRANSFER_D, TRANSFER_F);
        MotorRidicareBila.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, transferPIDF);

        telemetry.Log("Shooter Init", String.format("Base F: %.2f @ 12.5V (voltage-compensated)", BASE_SHOOTER_F));
    }

    public boolean IsShootingStateNone() {
        return shootType == ShootingState.None;
    };

    public void Run() {
        UpdateVoltageCompensation();

        if (autoSpinUpBoost && autoBoostTimer.milliseconds() > 900) {
            autoSpinUpBoost = false;

            MotorAruncare.setDirection(DcMotor.Direction.REVERSE);
            MotorAruncare.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
            MotorAruncare.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            // Force braking logic to trigger by setting lastMotorPower very high
            SetShooterVelocity(2000);
        }

        ReadButtons();
        if (Aruncare != null && Aruncare.wasJustPressed() && !mixer.IsEmpty()) {
            UpdateDistanceAndHood();
            CalculateShootingVelocity();
            runtime.reset();
            shootingAllowed = true;
            if (CanShootArranged()) {
                shootType = ShootingState.Arranged;
                arrangedIndex = 0;
            } else {
                shootType = ShootingState.Unarranged;
                arrangedIndex = 0;
            }
        } else if (ThrowGreen != null && ThrowGreen.wasJustPressed() && !mixer.IsEmpty()) {
            UpdateDistanceAndHood();
            CalculateShootingVelocity();
            shootType = ShootingState.Green;
            shootingAllowed = true;
        } else if (ThrowPurple != null && ThrowPurple.wasJustPressed() && !mixer.IsEmpty()) {
            UpdateDistanceAndHood();
            CalculateShootingVelocity();
            shootType = ShootingState.Purple;
            shootingAllowed = true;
        }

        if (!isAutoShooting) {
            if (constDist > 2.9) {
                CalculateShootingVelocity();
            }
            UpdateDistanceAndHood();
        }

        if (shootingAllowed)
            Shooting();

        HandleTuning();
        MaintainVelocity();
    }

    private void HandleTuning() {
        if (ct2 == null)
            return;

        // P 10
        // F 12.5
        stepIncrease.readValue();
        Fincrease.readValue();
        Fdescrease.readValue();
        Pincrease.readValue();
        Pdecrease.readValue();
        VelocityUp.readValue();
        VelocityDown.readValue();

        if (stepIncrease.wasJustPressed()) {
            tuningStepIndex = (tuningStepIndex + 1) % tuningSteps.length;
            tuningActive = true;
        }

        if (Fincrease.wasJustPressed()) {
            shooterF += tuningSteps[tuningStepIndex];
            tuningActive = true;
        }
        if (Fdescrease.wasJustPressed()) {
            shooterF -= tuningSteps[tuningStepIndex];
            tuningActive = true;
        }
        if (Pincrease.wasJustPressed()) {
            shooterP += tuningSteps[tuningStepIndex];
            tuningActive = true;
        }
        if (Pdecrease.wasJustPressed()) {
            shooterP -= tuningSteps[tuningStepIndex];
            tuningActive = true;
        }

        if (VelocityUp.wasJustPressed()) {
            targetShootingVelocity += 50;
            tuningActive = true;
        }
        if (VelocityDown.wasJustPressed()) {
            targetShootingVelocity -= 50;
            tuningActive = true;
        }

        HoodUp.readValue();
        HoodDown.readValue();
        if (HoodUp.wasJustPressed()) {
            tuningHoodPos += 0.003;
            tuningActive = true;
        }
        if (HoodDown.wasJustPressed()) {
            tuningHoodPos -= 0.003;
            tuningActive = true;
        }

        if (tuningActive) {
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
            MotorAruncare.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            telemetry.Log("=== TUNING ACTIVE ===", "");
            telemetry.Log("Target Vel (Bumpers)", targetShootingVelocity);
            telemetry.Log("Hood Pos (X/A)", tuningHoodPos);
            telemetry.Log("F (DPAD L/R)", shooterF);
            telemetry.Log("P (DPAD U/D)", shooterP);
            telemetry.Log("Step (B)", tuningSteps[tuningStepIndex]);
        }
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

    private void ResetTimer() {
        runtime.reset();
    }

    public void SetShooterVelocity(double velocity) {
        double currentVelocity = MotorAruncare.getVelocity();
        double velocityDrop = lastMotorPower - velocity;
        double velocityError = currentVelocity - velocity;

        if (autoSpinUpBoost)
            return;

        // Pure proportional braking
        // Trigger if:
        // 1. Target speed was explicitly dropped (standard)
        // 2. OR we are shooting and the current RPM is too high (precision settle)
        if ((velocityDrop > 7 && velocityError > 7) || (shootingAllowed && velocityError > 10)) {
            if (!isBraking) {
                brakingTimer.reset();
                isBraking = true;
            }

            if (brakingTimer.milliseconds() < 250) {
                double effectiveMultiplier = breakMultiplier;
                if (constDist > 2.9) {
                    effectiveMultiplier = breakMultiplier * 0.5; // Less harsh for long distances
                }

                double brakePower = -velocityError * effectiveMultiplier;
                MotorAruncare.setVelocity(brakePower);
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

        MotorAruncare.setVelocity(velocity);
        lastMotorPower = velocity;
    }

    private void UpdateVoltageCompensation() {
        if (voltageHelper == null || tuningActive)
            return;

        double newF = voltageHelper.GetCompensatedF(BASE_SHOOTER_F);

        if (Math.abs(newF - shooterF) > 0.01) {
            shooterF = newF;
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
            MotorAruncare.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        }
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
            default:
                break;
        }
    }

    private void Unordered() {
        telemetry.Log("Unordered State", unorderedCaseSwitch);
        if (isShooting) {
            telemetry.Log("Live Shoot Timer", String.format("%.3f s", totalShootTimer.seconds()));
        }

        if (!isShooting && !mixer.IsEmpty()) {
            MotorRidicareBila.setVelocity(TRANSFER_VELOCITY);
            isShooting = true;
            unorderedCaseSwitch = 1;
            unorderedSeqIndex = 0;
            totalShootTimer.reset();
            ResetTimer();
        }

        if (!isShooting)
            return;

        // Baseline: Starting position for 3 balls is ~0.622
        // Based on: initialPosMixer + 4 * offsetPositionMixer
        double baselinePos = initialPosMixer + 4 * offsetPositionMixer;
        double baselineEnc = 4050 + 1357.5; // Encoder baseline for the same point

        switch (unorderedCaseSwitch) {
            case 1: // Move to next slot stop in the sweep
                if (unorderedSeqIndex < 3) {
                    // Skip empty slots instantly
                    if (mixer.artifacte[unorderedSeqIndex] == Color.None) {
                        unorderedSeqIndex++;
                        return; // Process next slot in next iteration
                    }

                    // Formula: StartingPos - (1, 3, or 5) * offset
                    targetUnorderedPos = baselinePos - (1 + 2 * unorderedSeqIndex) * offsetPositionMixer;
                    targetUnorderedEncoder = baselineEnc - (1 + 2 * unorderedSeqIndex) * 1357.5;

                    mixer.SetPosition(targetUnorderedPos);
                    unorderedCaseSwitch = 2;
                    runtime.reset();
                } else {
                    // Sweep finished, continue smoothly to 0.0
                    mixer.SetPosition(0.01);
                    unorderedCaseSwitch = 4;
                    runtime.reset();
                }
                break;

            case 2: // Wait for arrival at target or timeout
                int currentEnc = (int) mixer.GetEncoderPosition();
                if (Math.abs(currentEnc - targetUnorderedEncoder) < 200 || runtime.seconds() > 0.17) {
                    pauseTimer.reset();
                    unorderedCaseSwitch = 3;
                }
                break;

            case 3: // Eject and increment index
                if (pauseTimer.seconds() > 0.03) {
                    mixer.RemoveArtifact(unorderedSeqIndex);
                    unorderedSeqIndex++;
                    unorderedCaseSwitch = 1;
                }
                break;

            case 4: // Cleanup delay at the end of sweep
                if (runtime.seconds() > 0.1) {
                    MotorRidicareBila.setVelocity(0);
                    shootType = ShootingState.None;
                    ResetShooter();
                }
                break;
        }
    }

    private void Ordered() {
        telemetry.Log("Ordered State", orderedCaseSwitch);
        telemetry.Log("Ordered Index", orderedSeqIndex);
        if (isShooting) {
            telemetry.Log("Live Shoot Timer", String.format("%.3f s", totalShootTimer.seconds()));
        }

        if (!isShooting && !mixer.IsEmpty()) {
            isShooting = true;
            orderedCaseSwitch = 1; // Find first target
            orderedSeqIndex = 0;
            totalShootTimer.reset();
            ResetTimer();
        }

        if (!isShooting)
            return;

        switch (orderedCaseSwitch) {
            case 1: // Find next color target from artifactOrder
                if (orderedSeqIndex < limelight.artifactOrder.length) {
                    Color targetColor = limelight.artifactOrder[orderedSeqIndex];
                    int slot = mixer.GetColorPosition(targetColor);

                    if (slot != -1) {
                        targetOrderedPos = artPoz[slot];
                        targetOrderedEncoder = pozEncoder[slot];
                        mixer.SetPosition(targetOrderedPos);
                        orderedSlotIndex = slot;
                        orderedCaseSwitch = 2;
                        runtime.reset(); // Timeout for encoder
                    } else {
                        orderedSeqIndex++; // Color not in mixer, try next in order
                    }
                } else {
                    // End of order, final sweep to complete
                    mixer.SetMaximPos();
                    orderedCaseSwitch = 4;
                    runtime.reset();
                }
                break;

            case 2: // Wait for arrival (3s timeout for calibration)
                int currentEnc = (int) mixer.GetEncoderPosition();
                boolean arrived = Math.abs(currentEnc - targetOrderedEncoder) < 180;
                telemetry.Log("Mixer Enc", currentEnc);
                telemetry.Log("Target Enc", targetOrderedEncoder);

                if (arrived || runtime.seconds() > 3.0) {
                    MotorRidicareBila.setVelocity(TRANSFER_VELOCITY); // Start motor once aligned
                    pauseTimer.reset();
                    orderedCaseSwitch = 3;
                }
                break;

            case 3: // Pause and mark shot
                if (pauseTimer.seconds() > 0.1) {
                    mixer.RemoveArtifact(orderedSlotIndex);
                    orderedSeqIndex++;
                    orderedCaseSwitch = 1; // Search next color
                }
                break;

            case 4: // Final finish
                if (runtime.seconds() > 0.4) {
                    MotorRidicareBila.setVelocity(0);
                    shootType = ShootingState.None;
                    ResetShooter();
                }
                break;
        }
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

    private void ShootColor() {
        if (!isShooting && !mixer.IsEmpty() && currentShootingPosition >= 0) {
            isShooting = true;
            caseSwitch = 0;
            ResetTimer();
            mixer.SetPosition(artPoz[currentShootingPosition]);
            if (shootType == ShootingState.Arranged) {
                arrangedIndex++;
            }

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
        shootingAllowed = false;
        isShooting = false;
        shootType = ShootingState.None;
        arrangedIndex = 0;
        telemetry.Log("✓ Shooter Reset", "Ready");
    }

    public boolean CanShootArranged() {
        return limelight.GetID() != 0 && mixer.GetCountGreen() == 1 && mixer.GetCountPurple() == 3; // 2 mingi mov
                                                                                                    // actually1
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
        if (tuningActive)
            return; // Locked to manual tuning velocity

        // Restore dynamic velocity for match distances
        double safeDist = constDist;
        if (safeDist < 1) {
            safeDist = 1;
        } else if (safeDist > 5.5) {
            safeDist = 5.5;
        }

        double dist = safeDist * 100; // Convert to cm for polynomial
        if (safeDist > 1 && safeDist < 5.0)
            targetShootingVelocity = -(1.35657 / 1000000000) * dist * dist * dist * dist
                    + 0.0000229718 * dist * dist * dist
                    - 0.0166181 * dist * dist + 5.32713 * dist + velocityConstant;
        else if (safeDist >= 5.0)
            targetShootingVelocity = 1580;
        else
            targetShootingVelocity = 1100;
    }

    public void CalculateShootingVelocityTelemetry() {
        double dist = constDist * 100;

        telemetry.Log("Distance cm", dist);
        telemetry.Log("HoodPos", ServoHood.getPosition());
        telemetry.Log("Last Shoot Duration", String.format("%.3f s", lastShootDuration));
        telemetry.Log("Mixer frequency", mixer.GetArtifactCount());
        telemetry.Log("Mixer pos:", mixer.ServoMixer1.getPosition());

        double velocity;
        if (constDist > 1 && constDist < 5.0)
            velocity = -(1.35657 / 1000000000) * dist * dist * dist * dist + 0.0000229718 * dist * dist * dist
                    - 0.0166181 * dist * dist + 5.32713 * dist + velocityConstant;
        else if (constDist >= 5.0)
            velocity = 1680;
        else
            velocity = 1100;

        telemetry.Log("Target Velocity", velocity);
        telemetry.Log("Current Aruncare Vel", MotorAruncare.getVelocity());
        telemetry.Log("Current Ridicare Vel", MotorRidicareBila.getVelocity());
        telemetry.Log("Mixer Enc", mixer.GetEncoderPosition());
    }

    private void MaintainVelocity() {
        if (!autoSpinUpBoost) {
            if (tuningActive) {
                SetShooterVelocity(targetShootingVelocity);
            } else if (shootingAllowed) {
                SetShooterVelocity(targetShootingVelocity);
            } else {
                SetShooterVelocity(IDLE_VELOCITY_MAX);
            }
        }
    }

    private void HoodPosition() {
        if (tuningActive) {
            ServoHood.setPosition(com.qualcomm.robotcore.util.Range.clip(tuningHoodPos, 0.0, 1.0));
            return;
        }

        double calculateServoPos = hoodInitialPosition;
        double dist = constDist * 100;

        // 1. Base distance-based calculation
        if (constDist >= 1 && constDist <= 4)
            calculateServoPos = (-0.000245041 * dist * dist + 0.163034 * dist + 475 + 70) / 1000;
        else if (constDist > 4)
            calculateServoPos = 0.55;

        // 2. Velocity-based correction
        double currentVel = MotorAruncare.getVelocity();
        double velError = currentVel - targetShootingVelocity;

        // If current velocity is LOWER than target, error is NEGATIVE,
        // which will REDUCE the hood angle (servo position).
        double correction = velError * hoodVelocityGain;

        // Limit correction to +/- 0.06 to keep it safe
        correction = com.qualcomm.robotcore.util.Range.clip(correction, -0.06, 0.06);

        double finalPos = com.qualcomm.robotcore.util.Range.clip(calculateServoPos + correction, 0.0, 1.0);
        ServoHood.setPosition(finalPos);

        telemetry.Log("Hood Base", String.format("%.3f", calculateServoPos));
        telemetry.Log("Hood Corr", String.format("%.3f", correction));
    }

    public double GetVelocityCurrent() {
        return MotorAruncare.getVelocity();
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

        // Wait for the state machine to finish properly (None)
        // instead of killing it immediately when the mixer is empty.
        // This ensures the last ball gets its pause and cleanup time.
        if (shootType == ShootingState.None) {
            autoShooting = false;
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

            telemetry.Log("⚡ Auto Shoot", String.format("Starting @ %.0f RPM → %.0f RPM", MotorAruncare.getVelocity(),
                    targetShootingVelocity));
        }
    }

    public void StartAutoBoost() {
        if (!autoSpinUpBoost) {
            autoSpinUpBoost = true;
            autoBoostTimer.reset();

            MotorAruncare.setDirection(DcMotorSimple.Direction.REVERSE);
            MotorAruncare.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            MotorAruncare.setPower(1); // Reduced from 1.0 to avoid excessive over-speed

            telemetry.Log("⚡ RAW POWER", "100% voltage boost!");
        }
    }

    public void ForceUpdateShooterF() {
        if (voltageHelper == null)
            return;

        shooterF = voltageHelper.ForceUpdate(BASE_SHOOTER_F);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
        MotorAruncare.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.Log("Init F Comp", String.format("V=%.2fV → F=%.2f", voltageHelper.GetRawVoltage(), shooterF));
    }

    public double GetBatteryVoltage() {
        if (voltageHelper == null)
            return 12.0;
        return voltageHelper.GetFilteredVoltage();
    }
}