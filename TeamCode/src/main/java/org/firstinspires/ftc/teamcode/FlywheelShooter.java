package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Shooter cu flywheel: doua motoare care se rotesc in sensuri opuse.
//
// Controale:
//  DPAD_UP / DPAD_DOWN     - viteza +/- 5
//  DPAD_RIGHT / DPAD_LEFT  - viteza +/- 2
//  RIGHT_BUMPER            - porneste / opreste shooterul
//  B                       - opreste shooterul
//  X                       - alege constanta (P, I, D, F)
//  Y / A                   - creste / scade constanta aleasa
//  LEFT_BUMPER             - schimba marimea pasului (10, 1, 0.1, 0.01, 0.001)
//
// Constantele public static se pot schimba si LIVE din Panels (tab Configurables).
// Schimbarile din Panels si din controller se pierd la restart: scrie aici valorile bune.
@Configurable
public class FlywheelShooter implements Subsystem {
    private DcMotorEx MotorShooter;
    private DcMotorEx MotorShooter2;
    private VoltageSensor battery = null;
    private final GamepadEx ct;
    private final Telemetry telemetry;

    ButtonReader VelUp, VelDown, VelUpFine, VelDownFine, Select, ParamUp, ParamDown, StepChange, Stop, Toggle;

    public static double P = 10.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 13.0;

    // Cat de aproape (in ticks/s) trebuie sa fie viteza ca sa zicem ca e gata de tras
    public static double Toleranta = 40;

    // Compensare de baterie: cand bateria scade, F creste putin ca viteza sa ramana la fel
    public static boolean USE_VOLTAGE_COMP = true;
    public static double V_REF = 12.5;

    // Viteza aleasa din D-pad; roata o atinge doar cand shooterul e pornit
    public double VitezaSetata = 1000;
    public boolean Pornit = false;
    // Viteza care chiar e ceruta motoarelor acum (0 cand e oprit)
    public double TargetVelocity = 0;

    public final ShotTable Table = new ShotTable();

    private final String[] numeParam = {"P", "I", "D", "F"};
    private int paramAles = 0;
    private final double[] pasi = {10, 1, 0.1, 0.01, 0.001};
    private int pasIndex = 1;

    private final ElapsedTime voltageTimer = new ElapsedTime();
    private double voltajFiltrat = V_REF;
    private double lastAppliedF = -1;

    public FlywheelShooter(GamepadEx ct, Telemetry telemetry) {
        this.ct = ct;
        this.telemetry = telemetry;
    }

    public void LinkComponents(HardwareMap hwMap) {
        MotorShooter = hwMap.get(DcMotorEx.class, "MotorShooter");
        MotorShooter2 = hwMap.get(DcMotorEx.class, "MotorShooter2");

        if (hwMap.voltageSensor.iterator().hasNext())
            battery = hwMap.voltageSensor.iterator().next();
    }

    public void Initialize(HardwareMap hwMap) {
        LinkComponents(hwMap);

        VelUp = new ButtonReader(ct, GamepadKeys.Button.DPAD_UP);
        VelDown = new ButtonReader(ct, GamepadKeys.Button.DPAD_DOWN);
        VelUpFine = new ButtonReader(ct, GamepadKeys.Button.DPAD_RIGHT);
        VelDownFine = new ButtonReader(ct, GamepadKeys.Button.DPAD_LEFT);
        Select = new ButtonReader(ct, GamepadKeys.Button.X);
        ParamUp = new ButtonReader(ct, GamepadKeys.Button.Y);
        ParamDown = new ButtonReader(ct, GamepadKeys.Button.A);
        StepChange = new ButtonReader(ct, GamepadKeys.Button.LEFT_BUMPER);
        Stop = new ButtonReader(ct, GamepadKeys.Button.B);
        Toggle = new ButtonReader(ct, GamepadKeys.Button.RIGHT_BUMPER);

        MotorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorShooter.setDirection(DcMotor.Direction.FORWARD);

        // Al doilea motor sta pe partea opusa, deci se invarte invers
        MotorShooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorShooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorShooter2.setDirection(DcMotor.Direction.REVERSE);

        voltajFiltrat = GetBatteryVoltage();
        ApplyPIDF();
        SetMotorsVelocity(0);
    }

    // ---------------- PIDF si baterie ----------------

    private double GetBatteryVoltage() {
        if (battery == null)
            return V_REF;
        double v = battery.getVoltage();
        return v > 1.0 ? v : V_REF;
    }

    private double CompensatedF() {
        if (!USE_VOLTAGE_COMP)
            return F;
        double scale = Math.pow(V_REF / voltajFiltrat, 1.1);
        return F * Range.clip(scale, 0.85, 1.15);
    }

    // Trimite P, I, D, F la motoare, doar daca F compensat s-a schimbat
    private void UpdatePIDF() {
        double f = CompensatedF();
        if (Math.abs(f - lastAppliedF) < 0.05)
            return;

        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, f);
        MotorShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        MotorShooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        lastAppliedF = f;
    }

    // Forteaza retrimiterea constantelor (cand s-a schimbat P, I, D sau F)
    public void ApplyPIDF() {
        lastAppliedF = -1;
        UpdatePIDF();
    }

    // Citirea bateriei se face rar, ca sa nu incetineasca bucla
    private void UpdateVoltage() {
        if (voltageTimer.milliseconds() < 200)
            return;
        voltageTimer.reset();
        voltajFiltrat += 0.3 * (GetBatteryVoltage() - voltajFiltrat);
        UpdatePIDF();
    }

    // ---------------- Viteza ----------------

    private void SetMotorsVelocity(double velocity) {
        MotorShooter.setVelocity(velocity);
        MotorShooter2.setVelocity(velocity);
    }

    private void Apply() {
        TargetVelocity = Pornit ? VitezaSetata : 0;
        SetMotorsVelocity(TargetVelocity);
    }

    public void SetTargetVelocity(double velocity) {
        VitezaSetata = Math.max(0, velocity);
        Apply();
    }

    public void Start() {
        Pornit = true;
        Apply();
    }

    public void Stop() {
        Pornit = false;
        Apply();
    }

    public double GetVelocity() {
        return MotorShooter.getVelocity();
    }

    public double GetVelocity2() {
        return MotorShooter2.getVelocity();
    }

    // Gata de tras doar cand ambele motoare sunt aproape de viteza tinta
    public boolean IsReady() {
        return TargetVelocity > 0
                && Math.abs(TargetVelocity - GetVelocity()) < Toleranta
                && Math.abs(TargetVelocity - GetVelocity2()) < Toleranta;
    }

    // ---------------- Tabel ----------------

    // Pune viteza dupa tabel, pentru distanta data. false daca tabelul e gol.
    public boolean ApplyDistance(double distanceCm) {
        ShotTable.Shot shot = Table.Get(distanceCm);
        if (shot == null)
            return false;
        SetTargetVelocity(shot.velocity);
        return true;
    }

    // ---------------- Reglaj din controller ----------------

    private double GetParam() {
        switch (paramAles) {
            case 0: return P;
            case 1: return I;
            case 2: return D;
            default: return F;
        }
    }

    private void SetParam(double value) {
        value = Math.max(0, value);
        switch (paramAles) {
            case 0: P = value; break;
            case 1: I = value; break;
            case 2: D = value; break;
            default: F = value; break;
        }
        ApplyPIDF();
    }

    public void Controls() {
        VelUp.readValue();
        VelDown.readValue();
        VelUpFine.readValue();
        VelDownFine.readValue();
        Select.readValue();
        ParamUp.readValue();
        ParamDown.readValue();
        StepChange.readValue();
        Stop.readValue();
        Toggle.readValue();

        if (VelUp.wasJustPressed()) SetTargetVelocity(VitezaSetata + 5);
        if (VelDown.wasJustPressed()) SetTargetVelocity(VitezaSetata - 5);
        if (VelUpFine.wasJustPressed()) SetTargetVelocity(VitezaSetata + 2);
        if (VelDownFine.wasJustPressed()) SetTargetVelocity(VitezaSetata - 2);
        if (Stop.wasJustPressed()) Stop();
        if (Toggle.wasJustPressed()) {
            if (Pornit) Stop();
            else Start();
        }

        if (Select.wasJustPressed()) paramAles = (paramAles + 1) % numeParam.length;
        if (StepChange.wasJustPressed()) pasIndex = (pasIndex + 1) % pasi.length;
        if (ParamUp.wasJustPressed()) SetParam(GetParam() + pasi[pasIndex]);
        if (ParamDown.wasJustPressed()) SetParam(GetParam() - pasi[pasIndex]);
    }

    // ---------------- Telemetrie ----------------

    public void ShowTelemetry() {
        double eroare1 = TargetVelocity - GetVelocity();
        double eroare2 = TargetVelocity - GetVelocity2();

        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Pornit", Pornit);
        telemetry.addData("Gata de tras", IsReady());
        telemetry.addData("Viteza setata (ticks/s)", "%.0f", VitezaSetata);
        telemetry.addData("Viteza tinta (ticks/s)", "%.0f", TargetVelocity);
        telemetry.addData("Motor 1: real / eroare", "%.0f / %.0f", GetVelocity(), eroare1);
        telemetry.addData("Motor 2: real / eroare", "%.0f / %.0f", GetVelocity2(), eroare2);
        if (TargetVelocity > 0) {
            telemetry.addData("Eroare medie %", "%.1f%%", 100.0 * (eroare1 + eroare2) / 2.0 / TargetVelocity);
        }
        telemetry.addLine("=== CONSTANTE ===");
        telemetry.addData("Aleasa (X)", numeParam[paramAles]);
        telemetry.addData("Pas (LB)", pasi[pasIndex]);
        telemetry.addData("P / I / D", "%.3f / %.3f / %.3f", P, I, D);
        telemetry.addData("F (baza / compensat)", "%.3f / %.3f", F, CompensatedF());
        telemetry.addData("Baterie", "%.2f V", voltajFiltrat);
    }

    public void Run() {
        Controls();
        UpdateVoltage();
        SetMotorsVelocity(TargetVelocity);
        ShowTelemetry();
    }
}
