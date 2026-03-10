package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter implements Subsystem {
    private ButtonReader Aruncare;
    private ButtonReader ThrowGreen, ThrowPurple;
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

    // Prepare state machine (runs before button press when CanShootArranged)
    private int prepareCase = 0;
    private double targetPrepareEncoder = 0;
    private ElapsedTime prepareTimer = new ElapsedTime();
    private DcMotorEx MotorAruncare = null;
    private ShooterVoltageHelper voltageHelper = null;
    private LimeLight limelight;
    private final double BASE_SHOOTER_F = 13;
    private double shooterF = BASE_SHOOTER_F;
    private double shooterP = 13;
    private final double shooterI = 0.004;
    private final double shooterD = 7.0;

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

    // Ordered sequence — built dynamically at shoot-start
    private int orderedCaseSwitch = 0;
    private int orderedSeqIndex = 0;
    private double targetOrderedEncoder = 0;
    private ElapsedTime orderedTimer = new ElapsedTime();
    private int[] currentOrderedSlots = new int[3];   // which mixer slot to RemoveArtifact at each step
    private double[] currentOrderedServo = new double[3]; // servo positions in shoot order
    private double[] currentOrderedEnc = new double[3];   // encoder targets in shoot order

    private int unorderedCaseSwitch = 0;
    private int unorderedSeqIndex = 0;
    private double targetUnorderedPos = 0;
    private double targetUnorderedEncoder = 0;
    private ElapsedTime pauseTimer = new ElapsedTime();
    private double offsetPositionMixer = 0.09028;
    private final double initialPosMixer = 0.0827 + 2 * offsetPositionMixer;

    // Load positions per slot index (0, 1, 2)
    private final double[] artPoz = {
            initialPosMixer + 3 * offsetPositionMixer,
            initialPosMixer + 1 * offsetPositionMixer,
            initialPosMixer - 1 * offsetPositionMixer
    };

    // Encoder positions per slot index (0, 1, 2) — must match artPoz order
    private final double[] pozEncoder = {
            4050,
            1335,
            -1380
    };

    // Shoot baseline — same formula as Unordered (pre-positions one step past slot 0,
    // then sweeps decreasing through all slots in one direction)
    private final double SHOOT_BASELINE_POS = initialPosMixer + 4 * offsetPositionMixer;
    private final double SHOOT_BASELINE_ENC = 4050 + 1357.5;

    private ShooterDistance shooterDistance;

    private boolean autoSpinUpBoost = false;
    private ElapsedTime autoBoostTimer = new ElapsedTime();
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
            Aruncare = new ButtonReader(ct1, GamepadKeys.Button.A);
            ThrowGreen = new ButtonReader(ct1, GamepadKeys.Button.Y);
            ThrowPurple = new ButtonReader(ct1, GamepadKeys.Button.X);
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

        shooterDistance = new ShooterDistance();

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

        if (Aruncare != null && Aruncare.wasJustPressed() && !mixer.IsEmpty()) {
            UpdateDistanceAndHood();
            CalculateShootingVelocity();
            SetShooterVelocity(targetShootingVelocity);
            brakeDone = false;
            runtime.reset();
            shootingAllowed = true;
            if (CanShootArranged()) {
                shootType = ShootingState.Arranged;
                arrangedIndex = 0;
                prepareCase = 0; // reset so next prepare starts fresh
                BuildOrderedSequence();
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

        if (!shootingAllowed && CanShootArranged()) {
            RunPrepare();
            UpdateDistanceAndHood();
            CalculateShootingVelocity();
        } else if (!shootingAllowed && mixer.GetArtifactCount() == 3 && !CanShootArranged()) {
            prepareCase = 0; // reset prepare if balls change
            MotorRidicareBila.setVelocity(TRANSFER_VELOCITY);
            UpdateDistanceAndHood();
            CalculateShootingVelocity();
        } else if (!shootingAllowed) {
            prepareCase = 0;
            MotorRidicareBila.setVelocity(0);
            UpdateDistanceAndHood();
            CalculateShootingVelocity();
        }

        MaintainVelocity();
        HandleShootStates();
    }

    /**
     * Builds the ordered shoot sequence dynamically so the drum always sweeps
     * in one direction (same as Unordered), while still satisfying the required
     * color order.
     */
    private void RunPrepare() {
        switch (prepareCase) {
            case 0:
                // Build sequence and move to pre-position (one offset BEFORE first shoot slot)
                BuildOrderedSequence();

                // FIXED: We ADD the offset to sit before the target, because the sweep DECREASES values
                mixer.SetPosition(currentOrderedServo[0] + offsetPositionMixer);
                targetPrepareEncoder = currentOrderedEnc[0] + 1357.5;

                MotorRidicareBila.setVelocity(0);
                prepareTimer.reset();
                prepareCase = 1;
                break;

            case 1:
                // Wait until mixer arrives at pre-position
                int enc = (int) mixer.GetEncoderPosition();
                if (Math.abs(enc - targetPrepareEncoder) < 200 || prepareTimer.seconds() > 2.0) {
                    // Arrived — now start MotorRidicareBila
                    MotorRidicareBila.setVelocity(TRANSFER_VELOCITY);
                    prepareCase = 2;
                }
                break;

            case 2:
                // Holding position, motor running, waiting for button press
                // Nothing to do — Run() keeps MaintainVelocity going
                break;
        }
    }

    private void BuildOrderedSequence() {
        Color[] required = limelight.artifactOrder;

        // FIXED: A full revolution is 6 multiplier units, not 3.
        double FULL_REV_ENC = 6 * 1357.5;
        double FULL_REV_POS = 6 * offsetPositionMixer;

        // Temporary array to hold the chosen slots before we apply the wrap-around math
        int[] tempSlots = new int[3];
        boolean matchFound = false;

        // Try all 3 circular rotations for a unidirectional match
        int[][] rotations = {{0,1,2},{1,2,0},{2,0,1}};
        for (int[] rot : rotations) {
            int s0 = rot[0], s1 = rot[1], s2 = rot[2];
            if (mixer.artifacte[s0] == required[0]
                    && mixer.artifacte[s1] == required[1]
                    && mixer.artifacte[s2] == required[2]) {

                tempSlots[0] = s0; tempSlots[1] = s1; tempSlots[2] = s2;
                matchFound = true;
                telemetry.Log("seq", String.format("%d→%d→%d uni", s0, s1, s2));
                break;
            }
        }

        if (!matchFound) {
            // Exception: no unidirectional rotation matches.
            // Shoot green first, then remaining 2 purples.
            int greenSlot = -1;
            for (int i = 0; i < 3; i++) {
                if (mixer.artifacte[i] == Color.Green) { greenSlot = i; break; }
            }

            int p1 = -1, p2 = -1;
            for (int i = 0; i < 3; i++) {
                if (i == greenSlot) continue;
                if (p1 == -1) p1 = i; else p2 = i;
            }

            tempSlots[0] = greenSlot; tempSlots[1] = p1; tempSlots[2] = p2;
            telemetry.Log("seq", String.format("%d→%d→%d exception", greenSlot, p1, p2));
        }

        // Calculate baseline absolute positions for whatever sequence was chosen
        for (int i = 0; i < 3; i++) {
            currentOrderedSlots[i] = tempSlots[i];
            currentOrderedServo[i] = SHOOT_BASELINE_POS - (1 + 2 * tempSlots[i]) * offsetPositionMixer;
            currentOrderedEnc[i] = SHOOT_BASELINE_ENC - (1 + 2 * tempSlots[i]) * 1357.5;
        }

        // FIXED: Enforce strictly decreasing unidirectional sweep for ALL sequences (including exception)
        // We apply the wrap-around to BOTH the encoder AND the servo position so they stay synced.
        if (currentOrderedEnc[1] > currentOrderedEnc[0]) {
            currentOrderedEnc[1] -= FULL_REV_ENC;
            currentOrderedServo[1] -= FULL_REV_POS;
        }
        if (currentOrderedEnc[2] > currentOrderedEnc[1]) {
            currentOrderedEnc[2] -= FULL_REV_ENC;
            currentOrderedServo[2] -= FULL_REV_POS;
        }
    }

    private String colorName(Color c) {
        if (c == Color.Green) return "G";
        if (c == Color.Purple) return "P";
        return "N";
    }

    private void ReadButtons() {
        if (Aruncare != null && ThrowGreen != null && ThrowPurple != null) {
            Aruncare.readValue();
            ThrowGreen.readValue();
            ThrowPurple.readValue();
        }
    }

    private void ResetTimer() {
        runtime.reset();
    }

    public void SetShooterVelocity(double velocity) {
        double currentVelocity = MotorAruncare.getVelocity();
        double velocityDrop = lastMotorPower - velocity;
        double velocityError = velocity - currentVelocity;
        double breakingError = currentVelocity - velocity;

        if (autoSpinUpBoost)
            return;

        if (velocityError > 80) {
            MotorAruncare.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            MotorAruncare.setPower(1.0);
            isBraking = false;
            return;
        }

        if ((velocityDrop > 40 && breakingError > 40) || (shootingAllowed && breakingError > 150)) {
            if (!isBraking) {
                brakingTimer.reset();
                isBraking = true;
            }

            if (brakingTimer.milliseconds() < 250) {
                double effectiveMultiplier = breakMultiplier;
                if (constDist > 2.9)
                    effectiveMultiplier = breakMultiplier * 0.5;

                double brakePower = -breakingError * effectiveMultiplier;
                MotorAruncare.setVelocity(brakePower);
                return;
            } else {
                isBraking = false;
            }
        } else if (isBraking && breakingError <= 5) {
            isBraking = false;
        }

        if (isBraking)
            return;

        if (MotorAruncare.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            MotorAruncare.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
            MotorAruncare.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        }

        MotorAruncare.setVelocity(velocity);
        lastMotorPower = velocity;
    }

    private void UpdateVoltageCompensation() {
        if (voltageHelper == null)
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

    private void HandleShootStates() {
        if (shootingAllowed)
            Shooting();
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

        double baselinePos = initialPosMixer + 4 * offsetPositionMixer;
        double baselineEnc = 4050 + 1357.5;

        switch (unorderedCaseSwitch) {
            case 1:
                if (unorderedSeqIndex < 3) {
                    if (mixer.artifacte[unorderedSeqIndex] == Color.None) {
                        unorderedSeqIndex++;
                        return;
                    }
                    targetUnorderedPos = baselinePos - (1 + 2 * unorderedSeqIndex) * offsetPositionMixer;
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
                if (Math.abs(currentEnc - targetUnorderedEncoder) < 200 || runtime.seconds() > 2) {
                    pauseTimer.reset();
                    unorderedCaseSwitch = 3;
                }
                break;

            case 3:
                if (pauseTimer.seconds() > 0.03) {
                    mixer.RemoveArtifact(unorderedSeqIndex);
                    unorderedSeqIndex++;
                    unorderedCaseSwitch = 1;
                    pauseTimer.reset();
                }
                break;

            case 4:
                mixer.ResetServoPosition();
                if (runtime.seconds() > 0.8) {
                    MotorRidicareBila.setVelocity(0);
                    shootType = ShootingState.None;
                    ResetShooter();
                }
                break;
        }
    }

    private void Ordered() {
        telemetry.Log("Ordered State", orderedCaseSwitch);
        telemetry.Log("Ordered SeqIdx", orderedSeqIndex);

        if (!isShooting && !mixer.IsEmpty()) {
            isShooting = true;
            orderedCaseSwitch = 1;
            orderedSeqIndex = 0;
            totalShootTimer.reset();
            orderedTimer.reset();
            MotorRidicareBila.setVelocity(TRANSFER_VELOCITY);
        }

        if (!isShooting) return;

        switch (orderedCaseSwitch) {
            case 1: // Move to next shoot position
                if (orderedSeqIndex < 3) {
                    // Use the dynamically built sequence — always sweeps in one direction
                    mixer.SetPosition(currentOrderedServo[orderedSeqIndex]);
                    targetOrderedEncoder = currentOrderedEnc[orderedSeqIndex];
                    orderedCaseSwitch = 2;
                    orderedTimer.reset();
                } else {
                    mixer.SetPosition(0.01);
                    orderedCaseSwitch = 4;
                    orderedTimer.reset();
                }
                break;

            case 2: // Wait to arrive at position
                int curEnc = (int) mixer.GetEncoderPosition();
                if (Math.abs(curEnc - targetOrderedEncoder) < 200 || orderedTimer.seconds() > 2.0) {
                    pauseTimer.reset();
                    orderedCaseSwitch = 3;
                }
                break;

            case 3: // Ball shot — remove the correct slot, advance
                if (pauseTimer.seconds() > 0.03) {
                    mixer.RemoveArtifact(currentOrderedSlots[orderedSeqIndex]);
                    orderedSeqIndex++;
                    orderedCaseSwitch = 1;
                    pauseTimer.reset(); // prevent double-firing
                }
                break;

            case 4: // Done
                mixer.ResetServoPosition();
                if (orderedTimer.seconds() > 0.8) {
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

    public boolean GetShootingAllow() {
        return shootingAllowed;
    }

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
        return mixer.GetCountGreen() == 1 && mixer.GetCountPurple() == 3 && limelight.GetID() != 0;
    }

    private void UpdateDistanceAndHood() {
        double limeDist = limelight.GetDistanceToTargetPython();
        if (limeDist > 0)
            constDist = limeDist;
        HoodPosition();
    }

    private void CalculateShootingVelocity() {
        if (shooterDistance == null)
            return;

        double distCm = constDist * 100;
        ShooterDistance.ShotParameters params = shooterDistance.GetShotParams(distCm);
        targetShootingVelocity = params.rpm;
        targetHoodPosition = params.hoodAngle;

        telemetry.Log("Shot @ dist", String.format("%.1f cm → %.0f RPM, hood %.3f",
                distCm, targetShootingVelocity, targetHoodPosition));
    }

    public void CalculateShootingVelocityTelemetry() {
        double distCm = constDist * 100;
        telemetry.Log("Distance cm", distCm);
        telemetry.Log("HoodPos", ServoHood.getPosition());
        telemetry.Log("Last Shoot Duration", String.format("%.3f s", lastShootDuration));
        telemetry.Log("Mixer count", mixer.GetArtifactCount());
        telemetry.Log("Mixer encoder pos", mixer.GetEncoderPosition());
        if (shooterDistance != null) {
            ShooterDistance.ShotParameters p = shooterDistance.GetShotParams(distCm);
            telemetry.Log("Table RPM", p.rpm);
            telemetry.Log("Table Hood", p.hoodAngle);
        }
        telemetry.Log("Current Aruncare Vel", MotorAruncare.getVelocity());
        telemetry.Log("Current Ridicare Vel", MotorRidicareBila.getVelocity());
        telemetry.Log("shootType", shootType.toString());
        telemetry.Log("shootingAllowed", shootingAllowed);
        telemetry.Log("isShooting", isShooting);
        telemetry.Log("orderedCase", orderedCaseSwitch);
        telemetry.Log("orderedSeq", orderedSeqIndex);
        telemetry.Log("canArranged", CanShootArranged());
        if (currentOrderedSlots != null) {
            telemetry.Log("orderedSlots",
                    String.format("%d→%d→%d", currentOrderedSlots[0], currentOrderedSlots[1], currentOrderedSlots[2]));
        }
    }

    private void MaintainVelocity() {
        if (!autoSpinUpBoost) {
            if (shootingAllowed) {
                SetShooterVelocity(targetShootingVelocity);
            } else {
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

    public double GetVelocityCurrent() {
        return MotorAruncare.getVelocity();
    }

    public double GetVelocityTarget() {
        return shootingAllowed ? targetShootingVelocity : IDLE_VELOCITY_MAX;
    }

    public boolean AutoShoot() {
        if (!autoShooting)
            return true;

        if (shootType == ShootingState.Arranged)
            Ordered();
        else
            Unordered();

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

            if (CanShootArranged()) {
                shootType = ShootingState.Arranged;
                BuildOrderedSequence();
            } else {
                shootType = ShootingState.Unarranged;
            }
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

    public boolean IsShootingStateNone() {
        return shootType == ShootingState.None;
    }
}