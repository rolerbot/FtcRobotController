package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Shooter implements Subsystem {
    private ButtonReader Aruncare;
    private ButtonReader ThrowGreen, ThrowPurple;
    private ButtonReader TransferUp, TransferDown;
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

    private double constDist = 3.6; // Default fallback for ShootingPos (~3.6m)
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
    private double breakMultiplierNear = 6.3;
    private double breakMultiplierFar = 3.0;
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
    private int orderedStartIndex = 0;
    private boolean orderedReverse = false;
    private int unorderedCaseSwitch = 0;
    private int unorderedSeqIndex = 0;
    private double targetUnorderedPos = 0;
    private double targetUnorderedEncoder = 0;
    private ElapsedTime pauseTimer = new ElapsedTime();
    private ElapsedTime unorderedTimer = new ElapsedTime();
    private final double offsetPositionMixer = 0.09028;
    private final double initialPosMixer = 0.0827 + 2 * offsetPositionMixer; // = 0.26326

    // Physical shoot positions per artifacte index (measured)
    private final double[] artPoz = {
            initialPosMixer, // first ball to shoot
            initialPosMixer + 2 * offsetPositionMixer, // second ball to shoot
            initialPosMixer + 4 * offsetPositionMixer // third ball to shoot
    };
    // initialMixer - 2 * offset -third ball shoot
    // initialMixer + 6 * offset - first ball shoot
    // initialMixer + 8 * offset - second ball shoot

    private final double[] artPozOrdered = {
            initialPosMixer - offsetPositionMixer, // idx0
            initialPosMixer + offsetPositionMixer, // idx1
            initialPosMixer + 3 * offsetPositionMixer, // idx2
            initialPosMixer + 5 * offsetPositionMixer, // idx3
            initialPosMixer + 7 * offsetPositionMixer // idx4
    };

    private final double[] pozEncoder = {
            0,
            2709,
            5418
    };

    private final double[] pozEncoderOrdered = {
            -1354.5, // idx0
            1354.5, // idx1
            4063.5, // idx2
            6772.5, // idx3
            9481.5 // idx4
    };

    // which mixer slot each artPozOrdered index corresponds to
    private final int[] slotAtIndex = { 2, 1, 0, 2, 1 };

    private int[] orderedShootSlots = { 0, 1, 2 };
    private double[] orderedShootPositions = { 0, 0, 0 };
    private double[] orderedShootEncoders = { 0, 0, 0 };

    // Shoot baseline — one step past artifacte[2] (last loaded ball)
    // artifacte[0]=enc0, artifacte[1]=enc2709, artifacte[2]=enc5341
    private final double SHOOT_BASELINE_POS = initialPosMixer + 4 * offsetPositionMixer; // = 0.6244
    private final double SHOOT_BASELINE_ENC = 5418 + 1354.5; // = 6772.5

    private double GetOrderedPos(int step) {
        int idx = orderedReverse ? (orderedStartIndex - step) : (orderedStartIndex + step);
        return artPozOrdered[idx];
    }

    private double GetOrderedEnc(int step) {
        int idx = orderedReverse ? (orderedStartIndex - step) : (orderedStartIndex + step);
        return pozEncoderOrdered[idx];
    }

    private ShooterDistance shooterDistance;

    private boolean autoSpinUpBoost = false;
    private ElapsedTime autoBoostTimer = new ElapsedTime();
    private boolean prepareCalculated = false;
    private boolean wasArranged = false;
    private int manualTransferMode = 0; // 0: AUTO, 1: FORWARD, 2: REVERSE
    private int lastTagId = 0;
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
            TransferUp = new ButtonReader(ct2, GamepadKeys.Button.DPAD_UP);
            TransferDown = new ButtonReader(ct2, GamepadKeys.Button.DPAD_DOWN);
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
                DetermineOrderedSequence();
                shootType = ShootingState.Arranged;
                arrangedIndex = 0;
                prepareCase = 0; // reset so next prepare starts fresh
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
            // Both balls correct AND tag read — safe to build sequence and prepare
            RunPrepare();
            UpdateDistanceAndHood();
            CalculateShootingVelocity();
        } else if (!shootingAllowed && mixer.GetArtifactCount() == 3 && !CanShootArranged()) {
            prepareCase = 0; // not ready yet, reset
            prepareCalculated = false;
            MotorRidicareBila.setVelocity(TRANSFER_VELOCITY);
            UpdateDistanceAndHood();
            CalculateShootingVelocity();
        } else if (!shootingAllowed) {
            prepareCase = 0;
            prepareCalculated = false;
            MotorRidicareBila.setVelocity(0);
            UpdateDistanceAndHood();
            CalculateShootingVelocity();
        }

        MaintainVelocity();
        HandleShootStates();

        // Manual override toggle: Forward (Transfer), Reverse (-0.7), or AUTO
        if (TransferUp != null && TransferDown != null) {
            if (TransferUp.wasJustPressed()) {
                if (manualTransferMode != 0) {
                    manualTransferMode = 0;
                } else {
                    manualTransferMode = 1;
                }
            }
            if (TransferDown.wasJustPressed()) {
                if (manualTransferMode != 0) {
                    manualTransferMode = 0;
                } else {
                    manualTransferMode = 2;
                }
            }

            if (manualTransferMode == 1) {
                MotorRidicareBila.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                MotorRidicareBila.setVelocity(TRANSFER_VELOCITY);
            } else if (manualTransferMode == 2) {
                MotorRidicareBila.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                MotorRidicareBila.setPower(-0.7);
            }
            // In AUTO (0), the automatic setVelocity calls from Run logic will persist.
        }
    }

    public void StartBackMotorAuto() {
        MotorRidicareBila.setVelocity(TRANSFER_VELOCITY);
    }

    private void RunPrepare() {
        if (!shootingAllowed) {
            if (CanShootArranged()) {
                // Reset if we just became arranged (new ball configuration)
                int currentTagId = limelight.GetID();
                if (!wasArranged || currentTagId != lastTagId) {
                    DetermineOrderedSequence();
                    prepareCalculated = false;
                    wasArranged = true;
                    lastTagId = currentTagId;
                }
                PrepareArranged();
                int curEnc = (int) mixer.GetEncoderPosition();
                if (Math.abs(curEnc - targetPrepareEncoder) < 250)
                    MotorRidicareBila.setVelocity(TRANSFER_VELOCITY);
                else
                    MotorRidicareBila.setVelocity(0);
            } else {
                prepareCalculated = false;
                wasArranged = false;
                MotorRidicareBila.setVelocity(TRANSFER_VELOCITY);
            }
        } else {
            prepareCalculated = false;
            wasArranged = false;
        }
    }

    private void PrepareArranged() {
        // Reposition to a midpoint before the first ball in the sequence
        double dir = orderedReverse ? -1.0 : 1.0;

        double prePos = GetOrderedPos(0) - dir * offsetPositionMixer;
        double preEnc = GetOrderedEnc(0) - dir * 1354.5;

        targetPrepareEncoder = preEnc;
        mixer.SetPosition(prePos);
    }

    private void DetermineOrderedSequence() {
        int greenSlot = -1;
        for (int i = 0; i < 3; i++) {
            if (mixer.artifacte[i] == Color.Green) {
                greenSlot = i;
                break;
            }
        }
        int tagId = limelight.GetID();
        if (tagId < 21 || tagId > 23 || greenSlot == -1)
            return;
        int caseId = (tagId - 21) * 3 + greenSlot;
        switch (caseId) {
            // --- TARGET 21: G P P ---
            case 0: // Mixer G P P -> Shoot G P P (Order: 0, 1, 2)
                SetSequence(2, true);
                break;
            case 1: // Mixer P G P -> Shoot P P G (Order: 0, 2, 1)
                SetSequence(2, false);
                break;
            case 2: // Mixer P P G -> Shoot G P P (Order: 2, 1, 0)
                SetSequence(0, false);
                break;

            // --- TARGET 22: P G P ---
            case 3: // Mixer G P P -> Shoot P G P (Order: 1, 0, 2)
                SetSequence(1, false);
                break;
            case 4: // Mixer P G P -> Shoot P P G (Order: 0, 2, 1)
                SetSequence(2, false);
                break;
            case 5: // Mixer P P G -> Shoot P P G (Order: 1, 0, 2)
                SetSequence(1, false);
                break;

            // --- TARGET 23: P P G ---
            case 6: // Mixer G P P -> Shoot P P G (Order: 1, 2, 0)
                SetSequence(4, true);
                break;
            case 7: // Mixer P G P -> Shoot P G P (Order: 0, 1, 2)
                SetSequence(2, true);
                break;
            case 8: // Mixer P P G -> Shoot P G P (Order: 0, 2, 1)
                SetSequence(2, false);
                break;
        }
    }

    private void SetSequence(int startIdx, boolean reverse) {
        orderedStartIndex = startIdx;
        orderedReverse = reverse;
        // Also fill legacy slots for slot mapping in RemoveArtifact
        for (int i = 0; i < 3; i++) {
            int idx = reverse ? (startIdx - i) : (startIdx + i);
            orderedShootSlots[i] = slotAtIndex[idx];
        }
    }

    private int ColorCase(Color color1, Color color2) {
        if (color1 == Color.Green && color2 == Color.Purple)
            return 1; // Green → Purple → Purple
        else if (color1 == Color.Purple && color2 == Color.Green)
            return 2; // Purple → Green → Purple
        else if (color1 == Color.Purple && color2 == Color.Purple)
            return 3; // Purple → Purple → Green
        else
            return 0; // Invalid state
    }

    private double CalculateDirection(int pos1, int pos2) {
        if (pos1 == pos2)
            pos2 = mixer.GetColorPositionFurthest(Color.Purple);
        if ((pos1 == 0 && pos2 == 1) || (pos1 == 1 && pos2 == 2) || (pos1 == 2 && pos2 == 0))
            return offsetPositionMixer;
        else
            return -offsetPositionMixer;
    }

    private void ReadButtons() {
        if (Aruncare != null)
            Aruncare.readValue();
        if (ThrowGreen != null)
            ThrowGreen.readValue();
        if (ThrowPurple != null)
            ThrowPurple.readValue();
        if (TransferUp != null)
            TransferUp.readValue();
        if (TransferDown != null)
            TransferDown.readValue();
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
                double effectiveMultiplier = breakMultiplierNear;
                if (constDist > 2.9)
                    effectiveMultiplier = breakMultiplierFar;

                double brakePower = -breakingError * effectiveMultiplier;
                MotorAruncare.setVelocity(brakePower);
                return;
            } else {
                isBraking = false;
            }
        }
        else if (isBraking && breakingError <= 5) {
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
            isShooting = true;
            unorderedSeqIndex = 0;
            totalShootTimer.reset();
            unorderedTimer.reset();

            int bc = mixer.GetArtifactCount();
            if (bc == 3) {
                MotorRidicareBila.setVelocity(TRANSFER_VELOCITY);
                unorderedCaseSwitch = 1; // Instant 3-ball sweep
                runtime.reset();
            } else {
                unorderedCaseSwitch = 100; // 1 or 2 balls spin-up delay
            }
        }

        if (!isShooting)
            return;

        double baselinePos = initialPosMixer + 4 * offsetPositionMixer;
        double baselineEnc = 4050 + 1357.5;

        switch (unorderedCaseSwitch) {
            case 100: // Spin up wait for 1/2 balls
                if (unorderedTimer.seconds() > 0.2) {
                    MotorRidicareBila.setVelocity(TRANSFER_VELOCITY);
                    int count = mixer.GetArtifactCount();
                    if (count == 1) {
                        unorderedCaseSwitch = 10;
                    } else {
                        // 2 balls: "make it think it has 3"
                        unorderedCaseSwitch = 1;
                    }
                    runtime.reset();
                    unorderedTimer.reset();
                }
                break;

            case 10: // 1-ball special push
                mixer.SetPosition(mixer.GetCurrentPosition() + offsetPositionMixer);
                targetUnorderedEncoder = 2 * 1357.5; // Exactly 1 offset in ticks
                unorderedCaseSwitch = 11;
                unorderedTimer.reset();
                break;

            case 11: // Wait for 1-ball pos
                int curEnc = (int) mixer.GetEncoderPosition();
                if (Math.abs(curEnc - (int) targetUnorderedEncoder) < 200 || unorderedTimer.seconds() > 0.3) {
                    pauseTimer.reset();
                    unorderedCaseSwitch = 12;
                }
                break;

            case 12: // Remove and finish 1-ball
                if (pauseTimer.seconds() > 0.05) {
                    mixer.RemoveArtifact(0);
                    mixer.RemoveArtifact(1);
                    mixer.RemoveArtifact(2);
                    unorderedCaseSwitch = 4;
                    runtime.reset();
                    unorderedTimer.reset();
                }
                break;

            case 1: // Standard sweep logic (Used for 3-balls or "forced" 2-balls)
                if (unorderedSeqIndex < 3) {
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

            case 2: // Wait for sweep pos
                int currentEnc = (int) mixer.GetEncoderPosition();
                if (Math.abs(currentEnc - (int) targetUnorderedEncoder) < 200 || runtime.seconds() > 0.17) {
                    pauseTimer.reset();
                    unorderedCaseSwitch = 3;
                }
                break;

            case 3: // Clear slot and loop
                if (pauseTimer.seconds() > 0.03) {
                    mixer.RemoveArtifact(unorderedSeqIndex);
                    unorderedSeqIndex++;
                    unorderedCaseSwitch = 1;
                }
                break;

            case 4: // Final reset
                mixer.ResetServoPosition();
                if (runtime.seconds() > 0.4) {
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

        if (!isShooting)
            return;

        switch (orderedCaseSwitch) {
            case 1:
                if (orderedSeqIndex < 3) {
                    targetOrderedEncoder = GetOrderedEnc(orderedSeqIndex);
                    mixer.SetPosition(GetOrderedPos(orderedSeqIndex));
                    orderedCaseSwitch = 2;
                    orderedTimer.reset();
                } else {
                    mixer.SetPosition(0.01);
                    orderedCaseSwitch = 4;
                    orderedTimer.reset();
                }
                break;

            case 2:
                int curEnc = (int) mixer.GetEncoderPosition();
                if (Math.abs(curEnc - (int) targetOrderedEncoder) < 200 || orderedTimer.seconds() > 0.2) {
                    pauseTimer.reset();
                    orderedCaseSwitch = 3;
                }
                break;

            case 3:
                if (pauseTimer.seconds() > 0.05) {
                    mixer.RemoveArtifact(orderedShootSlots[orderedSeqIndex]);
                    orderedSeqIndex++;
                    orderedCaseSwitch = 1;
                }
                break;

            case 4:
                mixer.ResetServoPosition();
                if (orderedTimer.seconds() > 0.4) {
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
                currentShootingPosition = mixer.GetColorPositionClosest(Color.Green);
            else if (shootType == ShootingState.Purple && mixer.GetCountPurple() > 0)
                currentShootingPosition = mixer.GetColorPositionClosest(Color.Purple);
        }
        ShootColor();
    }

    public void StopBackMotor() {
        MotorRidicareBila.setVelocity(0);
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
        prepareCalculated = false;
        mixer.ResetServoPosition();
        shootingAllowed = false;
        isShooting = false;
        shootType = ShootingState.None;
        arrangedIndex = 0;
        autoShooting = false;
    }

    public boolean HasArrangedBalls() {
        return mixer.GetCountGreen() == 1 && mixer.GetCountPurple() == 3;
    }

    public boolean CanShootArranged() {
        return HasArrangedBalls() && limelight.GetID() != 0;
    }

    private void UpdateDistanceAndHood() {
        if (isLong) {
            constDist = 3.36; // Fixed 336cm for auto long
        } else {
            double limeDist = limelight.GetDistanceToTargetPython();
            if (limeDist > 0)
                constDist = limeDist;
        }
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
        telemetry.Log("shootType", shootType.toString());
        telemetry.Log("shootAllowed", shootingAllowed);
        telemetry.Log("isShooting", isShooting);
        telemetry.Log("ordCase", orderedCaseSwitch);
        telemetry.Log("ordSeq", orderedSeqIndex);
        telemetry.Log("ordStart", orderedStartIndex);
        telemetry.Log("ordRev", orderedReverse);
        telemetry.Log("prepCalc", prepareCalculated);
        telemetry.Log("curEnc", (int) mixer.GetEncoderPosition());
        telemetry.Log("prepTgtEnc", (int) targetPrepareEncoder);
        telemetry.Log("curShootPos", currentShootingPosition);
        int curIdx = orderedReverse ? orderedStartIndex - orderedSeqIndex : orderedStartIndex + orderedSeqIndex;
        if (curIdx >= 0 && curIdx < 5)
            telemetry.Log("curIdx", curIdx);
        telemetry.Log("vel tgt", (int) targetShootingVelocity);
        telemetry.Log("dist cm", (int) distCm);
        String modeStr = "AUTO";
        if (manualTransferMode == 1)
            modeStr = "FORWARD";
        else if (manualTransferMode == 2)
            modeStr = "REVERSE";
        telemetry.Log("Transfer Mode", modeStr);
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