package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Singura sursa de pozitie a robotului (goBILDA Pinpoint), folosita de turela, shooter si drivetrain.
 *
 * Coordonate: aceleasi ca Pedro Pathing (teren 0..144 inch, heading 0 = +X, CCW pozitiv).
 * Pedro 2.x scrie si citeste Pinpoint-ul direct in coordonate Pedro, deci pozitia lasata de
 * autonomie e valida si in TeleOp (cat timp robotul nu e oprit/resetat intre ele).
 *
 * Doua moduri:
 *  - TeleOp: RobotPinpoint configureaza Pinpoint-ul si apeleaza pinpoint.update() in Run().
 *  - Autonomie: follower-ul Pedro configureaza si face update; RobotPinpoint doar citeste
 *    (Run() trebuie apelat DUPA follower.update()).
 *
 * Tot aici se calculeaza geometria spre goal (pozitia axei turelei, distanta, unghiul),
 * ca turela si shooterul sa foloseasca exact aceleasi valori in aceeasi bucla.
 */
@Configurable
public class RobotPinpoint implements Subsystem {
    private GoBildaPinpointDriver pinpoint;
    private final TelemetryCustom telemetry;

    private final GamepadEx gamepad1;
    private final GamepadEx gamepad2;
    private final boolean isBlue;
    /** true = TeleOp (noi facem update); false = autonomie (follower-ul Pedro face update). */
    private final boolean ownsUpdate;

    private boolean lastTriggerState = false;

    // ===================== BLOCARE POZITIE CAND ROBOTUL STA (doar TeleOp) =====================
    // Cand soferul nu da comenzi de mers si robotul sta, pozitia se "ingheata": vibratiile
    // (intake, shooter) nu mai pot muta X / Y / heading. Abaterile mici se rescriu in Pinpoint.
    // Daca abaterea e mare (alt robot v-a impins), pozitia noua e acceptata.
    public static boolean POSE_LOCK_ACTIVA = true;
    public static double LOCK_DELAY_S = 0.4;           // cat sta fara comenzi inainte de blocare
    public static double LOCK_MAX_SPEED_IN_S = 8.0;    // peste asta robotul inca se misca -> nu blocam
    public static double LOCK_MAX_TURN_DEG_S = 20.0;
    public static double LOCK_MAX_DRIFT_IN = 3.0;      // peste asta = impins -> se accepta pozitia noua
    public static double LOCK_MAX_DRIFT_DEG = 4.0;
    public static double LOCK_CORRECT_IN = 0.3;        // de la ce abatere se rescrie pozitia in Pinpoint
    public static double LOCK_CORRECT_DEG = 0.3;
    public static double LOCK_CORRECT_PERIOD_S = 0.5;  // cel mult o rescriere la atata timp
    public static double STICK_DEADBAND = 0.05;        // joystick / trigger sub asta = fara comanda
    // ==========================================================================================

    private boolean locked = false;
    private double anchorX = 0, anchorY = 0, anchorH = 0;
    private double lockDriftIn = 0, lockDriftDeg = 0;
    private int lockCorrections = 0, lockPushes = 0;
    private final ElapsedTime stillTimer = new ElapsedTime();
    private final ElapsedTime correctTimer = new ElapsedTime();
    // valorile brute din Pinpoint (inainte de blocare), pentru telemetrie / teste
    private double rawX = 0, rawY = 0, rawHeading = 0;

    // ===================== GEOMETRIE =====================
    // Axa de rotatie a turelei fata de centrul robotului (centrul de tracking al Pinpoint),
    // in sistemul robotului: +X = inainte (forward), +Y = stanga.
    // Turela e in SPATE fata de centru -> valoare negativa.
    public static double TURRET_OFFSET_X_MM = -100.897; // + = spre fata robotului
    public static double TURRET_OFFSET_Y_MM = 0.0;      // + = spre stanga robotului

    // Punctul spre care se uita turela si fata de care se masoara distanta
    // (coordonate de teren Pedro, inch)
    public static double TARGET_BLUE_X = 7;
    public static double TARGET_BLUE_Y = 135;
    public static double TARGET_RED_X = 135;
    public static double TARGET_RED_Y = 144 - 7;
    // =====================================================

    // Pozitia curenta a robotului
    private double x = 0;
    private double y = 0;
    private double heading = 0; // grade

    // Geometrie calculata la fiecare Run()
    private double turretX = 0;
    private double turretY = 0;
    private double distanceToGoalIn = 0;
    private double angleToGoalDeg = 0; // fata de fata robotului, + = stanga

    // Viteze (coordonate de teren, ca pozitia): inch/s si rad/s (+ = CCW)
    private double velX = 0;
    private double velY = 0;
    private double headingVelRad = 0;

    // Pozitia la care se reseteaza (Right Trigger pe gamepad 2 in TeleOp)
    private double initialX = 0;
    private double initialY = 0;
    private double initialHeading = 0;

    /** TeleOp: configureaza Pinpoint-ul si face update in Run(). */
    public RobotPinpoint(TelemetryCustom telemetry, GamepadEx ct1, GamepadEx ct2, boolean isBlue) {
        this.telemetry = telemetry;
        this.gamepad1 = ct1;
        this.gamepad2 = ct2;
        this.isBlue = isBlue;
        this.ownsUpdate = true;
    }

    /** Autonomie: Pinpoint-ul e configurat si actualizat de follower-ul Pedro. */
    public RobotPinpoint(TelemetryCustom telemetry, boolean isBlue) {
        this.telemetry = telemetry;
        this.gamepad1 = null;
        this.gamepad2 = null;
        this.isBlue = isBlue;
        this.ownsUpdate = false;
    }

    public void LinkComponents(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConstants.hardwareMapName);
    }

    public void Initialize(HardwareMap hardwareMap) {
        LinkComponents(hardwareMap);

        if (ownsUpdate) {
            ConfigurePinpoint();
            // NU resetam pozitia - pastram pozitia din Auto!
            pinpoint.update();
        }
        ReadPosition();

        telemetry.Log("RobotPinpoint", ownsUpdate
                ? "Initialized - Position preserved from Auto"
                : "Initialized - updated by Pedro follower");
    }

    /**
     * Aceleasi setari ca in Constants (Pedro). Pedro apeleaza
     * setOffsets(forwardPodY, strafePodX), deci si noi la fel - altfel TeleOp si Auto
     * ar avea offseturi diferite si pozitia ar deriva cand robotul se roteste.
     */
    private void ConfigurePinpoint() {
        pinpoint.setOffsets(
                Constants.localizerConstants.forwardPodY,
                Constants.localizerConstants.strafePodX,
                Constants.localizerConstants.distanceUnit);
        pinpoint.setEncoderResolution(Constants.localizerConstants.encoderResolution);
        pinpoint.setEncoderDirections(
                Constants.localizerConstants.forwardEncoderDirection,
                Constants.localizerConstants.strafeEncoderDirection);
    }

    public void Run() {
        if (ownsUpdate) {
            pinpoint.update();
        }
        ReadPosition();
        if (ownsUpdate && UpdatePoseLock()) {
            UpdateGoalGeometry();
        }

        // Reset (Right Trigger pe gamepad 2)
        if (gamepad2 != null) {
            boolean currentTriggerState = gamepad2.gamepad.right_trigger > 0.5;
            if (currentTriggerState && !lastTriggerState) {
                ResetPosition();
            }
            lastTriggerState = currentTriggerState;
        }
    }

    /** Citeste ultima pozitie (fara I2C, update() a fost deja facut) si recalculeaza geometria. */
    private void ReadPosition() {
        Pose2D pose = pinpoint.getPosition();
        x = pose.getX(DistanceUnit.INCH);
        y = pose.getY(DistanceUnit.INCH);
        heading = pose.getHeading(AngleUnit.DEGREES);
        rawX = x;
        rawY = y;
        rawHeading = heading;
        velX = pinpoint.getVelX(DistanceUnit.INCH);
        velY = pinpoint.getVelY(DistanceUnit.INCH);
        headingVelRad = pinpoint.getHeadingVelocity(AngleUnit.RADIANS.getUnnormalized());
        UpdateGoalGeometry();
    }

    /**
     * Blocarea pozitiei cand robotul sta. Returneaza true daca pozitia folosita e cea blocata
     * (atunci x, y, heading = ancora si vitezele = 0).
     */
    private boolean UpdatePoseLock() {
        if (!POSE_LOCK_ACTIVA || gamepad1 == null) {
            locked = false;
            stillTimer.reset();
            return false;
        }

        boolean driverIdle = Math.abs(gamepad1.getLeftX()) < STICK_DEADBAND
                && Math.abs(gamepad1.getLeftY()) < STICK_DEADBAND
                && gamepad1.gamepad.left_trigger < STICK_DEADBAND
                && gamepad1.gamepad.right_trigger < STICK_DEADBAND;
        if (!driverIdle) {
            locked = false;
            stillTimer.reset();
            return false;
        }

        if (!locked) {
            boolean slow = Math.hypot(velX, velY) < LOCK_MAX_SPEED_IN_S
                    && Math.abs(Math.toDegrees(headingVelRad)) < LOCK_MAX_TURN_DEG_S;
            if (!slow) {
                stillTimer.reset();
                return false;
            }
            if (stillTimer.seconds() < LOCK_DELAY_S)
                return false;
            locked = true;
            anchorX = x;
            anchorY = y;
            anchorH = heading;
            correctTimer.reset();
        }

        lockDriftIn = Math.hypot(x - anchorX, y - anchorY);
        lockDriftDeg = NormalizeDeg(heading - anchorH);

        if (lockDriftIn > LOCK_MAX_DRIFT_IN || Math.abs(lockDriftDeg) > LOCK_MAX_DRIFT_DEG) {
            // abatere mare = robotul a fost impins -> acceptam pozitia noua ca ancora
            anchorX = x;
            anchorY = y;
            anchorH = heading;
            lockPushes++;
            return false;
        }

        if ((lockDriftIn > LOCK_CORRECT_IN || Math.abs(lockDriftDeg) > LOCK_CORRECT_DEG)
                && correctTimer.seconds() >= LOCK_CORRECT_PERIOD_S) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, anchorX, anchorY, AngleUnit.DEGREES, anchorH));
            correctTimer.reset();
            lockCorrections++;
        }

        x = anchorX;
        y = anchorY;
        heading = anchorH;
        velX = 0;
        velY = 0;
        headingVelRad = 0;
        return true;
    }

    public boolean IsPoseLocked() { return locked; }

    private void UpdateGoalGeometry() {
        double h = Math.toRadians(heading);
        double cos = Math.cos(h), sin = Math.sin(h);

        // Offsetul turelei e fix in sistemul robotului; pe teren se roteste odata cu robotul,
        // deci il rotim cu heading-ul inainte sa-l adunam la pozitie.
        double offX = TURRET_OFFSET_X_MM / 25.4;
        double offY = TURRET_OFFSET_Y_MM / 25.4;
        turretX = x + offX * cos - offY * sin;
        turretY = y + offX * sin + offY * cos;

        double dx = GetTargetX() - turretX;
        double dy = GetTargetY() - turretY;
        distanceToGoalIn = Math.hypot(dx, dy);
        angleToGoalDeg = NormalizeDeg(Math.toDegrees(Math.atan2(dy, dx)) - heading);
    }

    /**
     * Reset la pozitia initiala. Robotul trebuie sa fie fix in acel loc cand apesi,
     * pentru ca se recalibreaza si IMU-ul.
     */
    public void ResetPosition() {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, initialX, initialY, AngleUnit.DEGREES, initialHeading));
        pinpoint.recalibrateIMU();
        locked = false;
        stillTimer.reset();

        x = initialX;
        y = initialY;
        heading = initialHeading;
        UpdateGoalGeometry();

        if (gamepad1 != null) {
            gamepad1.gamepad.rumble(300);
        }
        if (gamepad2 != null) {
            gamepad2.gamepad.rumble(450);
        }

        telemetry.Log("Pinpoint Reset", String.format("X:%.1f\" Y:%.1f\" H:%.1f°", initialX, initialY, initialHeading));
    }

    /** Pozitia folosita de ResetPosition(). */
    public void SetInitialPosition(double x, double y, double headingDegrees) {
        this.initialX = x;
        this.initialY = y;
        this.initialHeading = headingDegrees;
    }

    /** Seteaza pozitia curenta (relocalizare). */
    public void SetPosition(double x, double y, double headingDegrees) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, headingDegrees));
        locked = false;
        stillTimer.reset();
        this.x = x;
        this.y = y;
        this.heading = headingDegrees;
        UpdateGoalGeometry();
    }

    private static double NormalizeDeg(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    // ---------------- Getters ----------------

    public double GetX() { return x; }

    public double GetY() { return y; }

    /** Grade. */
    public double GetHeading() { return heading; }

    public double GetHeadingRadians() { return Math.toRadians(heading); }

    public boolean IsBlue() { return isBlue; }

    /** Punctul tinta al aliantei (inch, coordonate Pedro). */
    public double GetTargetX() { return isBlue ? TARGET_BLUE_X : TARGET_RED_X; }

    public double GetTargetY() { return isBlue ? TARGET_BLUE_Y : TARGET_RED_Y; }

    public double GetTurretX() { return turretX; }

    public double GetTurretY() { return turretY; }

    /** Viteza robotului pe teren (inch/s), aceeasi pe care o foloseste Pedro. */
    public double GetVelX() { return velX; }

    public double GetVelY() { return velY; }

    /** Viteza de rotatie a robotului (rad/s, + = CCW / stanga). */
    public double GetHeadingVelocityRad() { return headingVelRad; }

    /**
     * Viteza axei turelei pe teren (inch/s). Turela e in spatele centrului, deci cand
     * robotul se roteste, axa ei se misca pe un cerc: v_turela = v_centru + omega x r.
     */
    public double GetTurretVelX() { return velX - headingVelRad * (turretY - y); }

    public double GetTurretVelY() { return velY + headingVelRad * (turretX - x); }

    /** Distanta axa turelei -> punctul tinta, inch. */
    public double GetDistanceToGoalInches() { return distanceToGoalIn; }

    /** Distanta axa turelei -> punctul tinta, metri (unitatea folosita de Shooter). */
    public double GetDistanceToGoalMeters() { return distanceToGoalIn * 0.0254; }

    /** Distanta axa turelei -> punctul tinta, cm (unitatea tabelului din ShooterDistance). */
    public double GetDistanceToGoalCm() { return distanceToGoalIn * 2.54; }

    /** Unghiul spre punctul tinta fata de fata robotului, vazut din axa turelei. + = stanga. */
    public double GetAngleToGoalDeg() { return angleToGoalDeg; }

    public GoBildaPinpointDriver GetPinpoint() { return pinpoint; }

    public GoBildaPinpointDriver.DeviceStatus GetDeviceStatus() { return pinpoint.getDeviceStatus(); }

    public void UpdateTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addLine("=== PINPOINT ===");
        telemetry.addData("  Status", pinpoint.getDeviceStatus());
        telemetry.addData("  X", "%.2f\"", x);
        telemetry.addData("  Y", "%.2f\"", y);
        telemetry.addData("  Heading", "%.1f°", heading);
        telemetry.addData("  Turela pe teren", "X %.1f\"  Y %.1f\"", turretX, turretY);
        telemetry.addData("  Dist goal", "%.1f\" (%.0f cm)", distanceToGoalIn, distanceToGoalIn * 2.54);
        telemetry.addData("  Unghi goal", "%.1f°", angleToGoalDeg);
        telemetry.addData("  Viteza", "vx %.1f  vy %.1f in/s  w %.0f°/s", velX, velY, Math.toDegrees(headingVelRad));
        telemetry.addData("  Blocare pozitie", !POSE_LOCK_ACTIVA ? "OPRITA" : (locked
                ? String.format("DA (abatere %.2f\" %.2f°, corectii %d)", lockDriftIn, lockDriftDeg, lockCorrections)
                : "nu"));
        telemetry.addData("  Pinpoint brut", "X %.2f  Y %.2f  H %.2f°  (impingeri %d)", rawX, rawY, rawHeading, lockPushes);
    }
}
