package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter implements Subsystem {
    private ButtonReader Aruncare;
    private ButtonReader ThrowGreen, ThrowPurple, ToggleFast;
    private ButtonReader btnKpUp, btnKpDown, btnKiUp, btnKiDown, btnKdUp, btnKdDown, btnStep;
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
    private final double BASE_SHOOTER_F = 14.5;
    private double shooterF = BASE_SHOOTER_F;
    private double shooterP = 7;
    private final double shooterI = 0.2;
    private final double shooterD = 8.0;

    // ── Tuning system ────────────────────────────────────────
    private ShooterTuningClass tuning;
    private boolean tuningActive = false    ;
    private final double[] stepSizes = {0.1, 0.01, 0.001, 0.0001};
    private int stepIndex = 1;

    private final double IDLE_VELOCITY_MAX = 1580;
    private double targetShootingVelocity = 1580;
    private double targetHoodPosition = 0.51;
    private double lastMotorPower = 1580;
    private ElapsedTime brakingTimer = new ElapsedTime();
    private boolean isBraking = false;
    private boolean isLong = false;
    private boolean autoShooting = false;
    private boolean isAutoShooting = false;
    private double breakMultiplier = 6;

    private final double TRANSFER_P = 10.0;
    private final double TRANSFER_I = 3.0;
    private final double TRANSFER_D = 0.0;
    private final double TRANSFER_F = 12.0;
    private final double TRANSFER_VELOCITY = 2800;

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
    private double offsetPositionMixer = 0.09028;
    private final double initialPosMixer = 0.0827 + 2 * offsetPositionMixer;

    private final double[] artPoz = {
            initialPosMixer + 3 * offsetPositionMixer,
            initialPosMixer + 1 * offsetPositionMixer,
            initialPosMixer - 1 * offsetPositionMixer
    };

    private final double[] pozEncoder = {
            4050,
            1335,
            -1380
    };

    private boolean autoSpinUpBoost = false;
    private ElapsedTime autoBoostTimer = new ElapsedTime();

    // Tracks whether we've already braked to target for this shot
    private boolean brakeDone = false;

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
            Aruncare    = new ButtonReader(ct1, GamepadKeys.Button.A);
            ThrowGreen  = new ButtonReader(ct1, GamepadKeys.Button.Y);
            ThrowPurple = new ButtonReader(ct1, GamepadKeys.Button.X);
            ToggleFast  = new ButtonReader(ct2, GamepadKeys.Button.Y);

            btnKpUp = new ButtonReader(ct2, GamepadKeys.Button.DPAD_UP);
            btnKpDown = new ButtonReader(ct2, GamepadKeys.Button.DPAD_DOWN);
            btnKiUp = new ButtonReader(ct2, GamepadKeys.Button.DPAD_RIGHT);
            btnKiDown = new ButtonReader(ct2, GamepadKeys.Button.DPAD_LEFT);
            btnKdUp = new ButtonReader(ct2, GamepadKeys.Button.RIGHT_BUMPER);
            btnKdDown = new ButtonReader(ct2, GamepadKeys.Button.LEFT_BUMPER);
            btnStep = new ButtonReader(ct2, GamepadKeys.Button.B);
        }

        ServoHood         = hardwareMap.get(Servo.class,     "ServoHood");
        MotorAruncare     = hardwareMap.get(DcMotorEx.class, "MotorAruncare");
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

        if (ct2 != null) {
            tuning = new ShooterTuningClass(ct2, telemetry, limelight);
        }

        telemetry.Log("Shooter Init", String.format("Base F: %.2f", BASE_SHOOTER_F));
    }

    public void Run() {
        UpdateVoltageCompensation();

        if (autoSpinUpBoost && autoBoostTimer.milliseconds() > 900) {
            autoSpinUpBoost = false;
            MotorAruncare.setDirection(DcMotor.Direction.REVERSE);
            MotorAruncare.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
            MotorAruncare.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            SetShooterVelocity(2000);
        }

        ReadButtons();

        // ── Tuning mode ──────────────────────────────────────
        if (tuningActive && tuning != null) {
            boolean testShoot = tuning.Update(MotorAruncare.getVelocity());

            ShooterTuningClass.ShotParameters selected = tuning.GetSelectedRow();
            targetShootingVelocity = selected.rpm;
            targetHoodPosition = selected.hoodAngle;

            ServoHood.setPosition(targetHoodPosition);
            SetShooterVelocity(targetShootingVelocity);

            // ct2 Y = quick single ball test shoot
            if (testShoot) {
                shootingAllowed = true;
                runtime.reset();
            }
            if (shootingAllowed && shootType == ShootingState.None) {
                if (runtime.milliseconds() < 400) {
                    MotorRidicareBila.setVelocity(TRANSFER_VELOCITY);
                } else {
                    MotorRidicareBila.setVelocity(0);
                    shootingAllowed = false;
                }
            }

            // ct1 A = full shoot sequence, uses currently selected row values
            if (Aruncare != null && Aruncare.wasJustPressed() && !mixer.IsEmpty()) {
                shootingAllowed = true;
                brakeDone = false;
                runtime.reset();
                if (CanShootArranged()) {
                    shootType = ShootingState.Arranged;
                    arrangedIndex = 0;
                } else {
                    shootType = ShootingState.Unarranged;
                    arrangedIndex = 0;
                }
            }
            if (shootingAllowed && shootType != ShootingState.None)
                Shooting();

            return;
        }

        // NEW: Live Turret Tuning when not in shooter tuning mode
        if (btnStep != null) {
            if (btnStep.wasJustPressed()) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }
            double step = stepSizes[stepIndex];
            if (btnKpUp.wasJustPressed()) TurretProfiledPIDControl.Kp += step;
            if (btnKpDown.wasJustPressed()) TurretProfiledPIDControl.Kp -= step;
            if (btnKiUp.wasJustPressed()) TurretProfiledPIDControl.Ki += step;
            if (btnKiDown.wasJustPressed()) TurretProfiledPIDControl.Ki -= step;
            if (btnKdUp.wasJustPressed()) TurretProfiledPIDControl.Kd += step;
            if (btnKdDown.wasJustPressed()) TurretProfiledPIDControl.Kd -= step;
        }

        // ── Match mode ───────────────────────────────────────
        if (Aruncare != null && Aruncare.wasJustPressed() && !mixer.IsEmpty()) {
            UpdateDistanceAndHood();
            CalculateShootingVelocity();
            // Immediately start braking from idle to target
            SetShooterVelocity(targetShootingVelocity);
            brakeDone = false;
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
            SetShooterVelocity(targetShootingVelocity);
            brakeDone = false;
            shootType = ShootingState.Green;
            shootingAllowed = true;
        } else if (ThrowPurple != null && ThrowPurple.wasJustPressed() && !mixer.IsEmpty()) {
            UpdateDistanceAndHood();
            CalculateShootingVelocity();
            SetShooterVelocity(targetShootingVelocity);
            brakeDone = false;
            shootType = ShootingState.Purple;
            shootingAllowed = true;
        }

        if (!isAutoShooting) {
            if (constDist > 2.9)
                CalculateShootingVelocity();
            UpdateDistanceAndHood();
        }

        // ── PreShoot: when 3 balls ready and unordered, pre-spin transfer motor
        // and pre-drop shooter RPM to expected target so A press is instant ──
        if (!shootingAllowed && mixer.GetArtifactCount() == 3 && !CanShootArranged()) {
            MotorRidicareBila.setVelocity(TRANSFER_VELOCITY);
            // Read distance and pre-calculate target so motor is already at right RPM
            UpdateDistanceAndHood();
            CalculateShootingVelocity();
            // Don't use SetShooterVelocity here — just set directly so PIDF holds it
            MotorAruncare.setVelocity(targetShootingVelocity);
            lastMotorPower = targetShootingVelocity;
        } else if (!shootingAllowed) {
            MotorRidicareBila.setVelocity(0);
            // Back to idle when no balls or arranged
        }

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
        if (ToggleFast != null)
            ToggleFast.readValue();

        if (btnKpUp != null) {
            btnKpUp.readValue();
            btnKpDown.readValue();
            btnKiUp.readValue();
            btnKiDown.readValue();
            btnKdUp.readValue();
            btnKdDown.readValue();
            btnStep.readValue();
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

        if ((velocityDrop > 7 && velocityError > 7) || (shootingAllowed && velocityError > 10)) {
            if (!isBraking) {
                brakingTimer.reset();
                isBraking = true;
            }

            if (brakingTimer.milliseconds() < 250) {
                double effectiveMultiplier = breakMultiplier;
                if (constDist > 2.9)
                    effectiveMultiplier = breakMultiplier * 0.5;

                double brakePower = -velocityError * effectiveMultiplier;
                MotorAruncare.setVelocity(brakePower);
                return;
            } else {
                isBraking = false;
            }
        } else if (isBraking && velocityError <= 5) {
            isBraking = false;
        }

        if (isBraking)
            return;

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
            case Arranged:   Ordered();     break;
            case Unarranged: Unordered();   break;
            case Purple:
            case Green:      PurpleGreen(); break;
            default: break;
        }
    }

    private void Unordered() {
        telemetry.Log("Unordered State", unorderedCaseSwitch);

        if (!isShooting && !mixer.IsEmpty()) {
            MotorRidicareBila.setVelocity(TRANSFER_VELOCITY);
            isShooting = true;
            unorderedCaseSwitch = 1;
            unorderedSeqIndex = 0;
            totalShootTimer.reset();
            ResetTimer();
        }

        if (!isShooting) return;

        double baselinePos = initialPosMixer + 4 * offsetPositionMixer;
        double baselineEnc = 4050 + 1357.5;

        switch (unorderedCaseSwitch) {
            case 1:
                if (unorderedSeqIndex < 3) {
                    if (mixer.artifacte[unorderedSeqIndex] == Color.None) {
                        unorderedSeqIndex++;
                        return;
                    }
                    targetUnorderedPos     = baselinePos - (1 + 2 * unorderedSeqIndex) * offsetPositionMixer;
                    targetUnorderedEncoder = baselineEnc - (1 + 2 * unorderedSeqIndex) * 1357.5;
                    mixer.SetPosition(targetUnorderedPos);
                    unorderedCaseSwitch = 2;
                    runtime.reset();
                } else {
                    mixer.SetPosition(0.01);
                    unorderedCaseSwitch = 4;
                    runtime.reset();
                }
                break;

            case 2:
                int currentEnc = (int) mixer.GetEncoderPosition();
                if (Math.abs(currentEnc - targetUnorderedEncoder) < 200 || runtime.seconds() > 0.17) {
                    pauseTimer.reset();
                    unorderedCaseSwitch = 3;
                }
                break;

            case 3:
                if (pauseTimer.seconds() > 0.03) {
                    mixer.RemoveArtifact(unorderedSeqIndex);
                    unorderedSeqIndex++;
                    unorderedCaseSwitch = 1;
                }
                break;

            case 4:
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

        if (!isShooting && !mixer.IsEmpty()) {
            isShooting = true;
            orderedCaseSwitch = 1;
            orderedSeqIndex = 0;
            totalShootTimer.reset();
            ResetTimer();
        }

        if (!isShooting) return;

        switch (orderedCaseSwitch) {
            case 1:
                if (orderedSeqIndex < limelight.artifactOrder.length) {
                    Color targetColor = limelight.artifactOrder[orderedSeqIndex];
                    int slot = mixer.GetColorPosition(targetColor);
                    if (slot != -1) {
                        targetOrderedPos     = artPoz[slot];
                        targetOrderedEncoder = pozEncoder[slot];
                        mixer.SetPosition(targetOrderedPos);
                        orderedSlotIndex = slot;
                        orderedCaseSwitch = 2;
                        runtime.reset();
                    } else {
                        orderedSeqIndex++;
                    }
                } else {
                    mixer.SetMaximPos();
                    orderedCaseSwitch = 4;
                    runtime.reset();
                }
                break;

            case 2:
                int currentEnc = (int) mixer.GetEncoderPosition();
                boolean arrived = Math.abs(currentEnc - targetOrderedEncoder) < 180;
                if (arrived || runtime.seconds() > 3.0) {
                    MotorRidicareBila.setVelocity(TRANSFER_VELOCITY);
                    pauseTimer.reset();
                    orderedCaseSwitch = 3;
                }
                break;

            case 3:
                if (pauseTimer.seconds() > 0.1) {
                    mixer.RemoveArtifact(orderedSlotIndex);
                    orderedSeqIndex++;
                    orderedCaseSwitch = 1;
                }
                break;

            case 4:
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
            if (shootType == ShootingState.Arranged)
                arrangedIndex++;
        }
    }

    public boolean GetShootingAllow() { return shootingAllowed; }

    private void ResetShooter() {
        lastShootDuration = totalShootTimer.seconds();
        telemetry.Log("Last Shoot Duration", String.format("%.3f s", lastShootDuration));
        SetShooterVelocity(IDLE_VELOCITY_MAX);
        lastMotorPower = IDLE_VELOCITY_MAX;
        brakeDone = false;
        mixer.ResetServoPosition();
        shootingAllowed = false;
        isShooting = false;
        shootType = ShootingState.None;
        arrangedIndex = 0;
    }

    public boolean CanShootArranged() {
        return limelight.GetID() != 0 && mixer.GetCountGreen() == 1 && mixer.GetCountPurple() == 3;
    }

    private boolean useDynamicDistance = true;

    private void UpdateDistanceAndHood() {
        if (!isAutoShooting || useDynamicDistance) {
            double limeDist = limelight.GetDistanceToTargetPython();
            if (limeDist > 0)
                constDist = limeDist;
        }
        HoodPosition();
    }

    private void CalculateShootingVelocity() {
        if (tuning == null) return;

        double distCm = constDist * 100;
        ShooterTuningClass.ShotParameters params = tuning.GetShotParams(distCm);
        targetShootingVelocity = params.rpm;
        targetHoodPosition     = params.hoodAngle;

        telemetry.Log("Shot @ dist", String.format("%.1f cm → %.0f RPM, hood %.3f",
                distCm, targetShootingVelocity, targetHoodPosition));
    }

    public void CalculateShootingVelocityTelemetry() {
        double distCm = constDist * 100;
        telemetry.Log("Distance cm", distCm);
        telemetry.Log("HoodPos", ServoHood.getPosition());
        telemetry.Log("Last Shoot Duration", String.format("%.3f s", lastShootDuration));
        telemetry.Log("Mixer count", mixer.GetArtifactCount());
        if (tuning != null) {
            ShooterTuningClass.ShotParameters p = tuning.GetShotParams(distCm);
            telemetry.Log("Table RPM",  p.rpm);
            telemetry.Log("Table Hood", p.hoodAngle);
        }
        telemetry.Log("Current Aruncare Vel", MotorAruncare.getVelocity());
        telemetry.Log("Current Ridicare Vel", MotorRidicareBila.getVelocity());
    }

    private void MaintainVelocity() {
        if (!autoSpinUpBoost) {
            if (shootingAllowed) {
                SetShooterVelocity(targetShootingVelocity);
            } else {
                // Constantly read distance and update velocity even when not shooting
                UpdateDistanceAndHood();
                CalculateShootingVelocity();
                SetShooterVelocity(targetShootingVelocity);
            }
        }
    }

    private void HoodPosition() {
        ServoHood.setPosition(
                com.qualcomm.robotcore.util.Range.clip(targetHoodPosition, 0.0, 1.0));
    }

    public double GetVelocityCurrent() { return MotorAruncare.getVelocity(); }

    public double GetVelocityTarget() {
        return shootingAllowed ? targetShootingVelocity : IDLE_VELOCITY_MAX;
    }

    public boolean AutoShoot() {
        if (!autoShooting) return true;

        if (shootType == ShootingState.Arranged) Ordered();
        else Unordered();

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
            SetShooterVelocity(targetShootingVelocity);
            brakeDone = false;
            autoShooting = true;
            shootingAllowed = true;
            isShooting = false;
            arrangedIndex = 0;

            if (CanShootArranged()) shootType = ShootingState.Arranged;
            else shootType = ShootingState.Unarranged;
        }
    }

    public void StartAutoBoost() {
        if (!autoSpinUpBoost) {
            autoSpinUpBoost = true;
            autoBoostTimer.reset();
            MotorAruncare.setDirection(DcMotorSimple.Direction.REVERSE);
            MotorAruncare.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            MotorAruncare.setPower(1);
        }
    }

    public void ForceUpdateShooterF() {
        if (voltageHelper == null) return;
        shooterF = voltageHelper.ForceUpdate(BASE_SHOOTER_F);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
        MotorAruncare.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.Log("Init F Comp", String.format("V=%.2fV → F=%.2f", voltageHelper.GetRawVoltage(), shooterF));
    }

    public double GetBatteryVoltage() {
        if (voltageHelper == null) return 12.0;
        return voltageHelper.GetFilteredVoltage();
    }

    public void SetTuningMode(boolean active) {
        tuningActive = active;
    }

    public boolean IsShootingStateNone() { return shootType == ShootingState.None; }

    public double getTuningStep() {
        return stepSizes[stepIndex];
    }
}
