package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Controale:
//  DPAD_UP / DPAD_DOWN     - viteza tinta +/- 5
//  DPAD_RIGHT / DPAD_LEFT  - viteza tinta +/- 2
//  X                       - alege ce constanta reglezi (P, I, D, F)
//  Y / A                   - creste / scade constanta aleasa
//  RIGHT_BUMPER            - porneste / opreste shooterul
//  B                       - opreste shooterul
public class Shooter implements Subsystem {
    private DcMotorEx MotorShooter;
    private DcMotorEx MotorShooter2;
    private final GamepadEx ct;
    private final Telemetry telemetry;

    ButtonReader VelUp, VelDown, VelUpFine, VelDownFine, Select, ParamUp, ParamDown, Stop, Toggle;

    // Viteza aleasa din D-pad; roata o atinge doar cand shooterul e pornit
    public double VitezaSetata = 1000;
    public boolean Pornit = false;

    // Valorile astea le gasesti reglandu-le din controller, apoi le scrii aici
    public static double P = 10.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 13.0;

    // Viteza dorita, in ticks/secunda
    public double TargetVelocity = 0;
    // Cat de aproape (in ticks/s) trebuie sa fie viteza ca sa zicem ca e gata de tras
    public static double Toleranta = 40;

    private final String[] numeParam = {"P", "I", "D", "F"};
    private int paramAles = 0;

    public Shooter(GamepadEx ct, Telemetry telemetry) {
        this.ct = ct;
        this.telemetry = telemetry;
    }

    public void LinkComponents(HardwareMap hwMap) {
        MotorShooter = hwMap.get(DcMotorEx.class, "MotorShooter");
        MotorShooter2 = hwMap.get(DcMotorEx.class, "MotorShooter2");
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
        Stop = new ButtonReader(ct, GamepadKeys.Button.B);
        Toggle = new ButtonReader(ct, GamepadKeys.Button.RIGHT_BUMPER);

        MotorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorShooter.setDirection(DcMotor.Direction.FORWARD);

        // Al doilea motor sta pe partea opusa, deci se invarte invers
        MotorShooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorShooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorShooter2.setDirection(DcMotor.Direction.REVERSE);

        ApplyPIDF();
        SetMotorsVelocity(0);
    }

    public void ApplyPIDF() {
        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);
        MotorShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        MotorShooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

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

    public void Stop() {
        Pornit = false;
        Apply();
    }

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

    // F si P merg cu pasi mari, I si D cu pasi mici
    private double Pas() {
        return (paramAles == 0 || paramAles == 3) ? 0.5 : 0.05;
    }

    public void Controls() {
        VelUp.readValue();
        VelDown.readValue();
        VelUpFine.readValue();
        VelDownFine.readValue();
        Select.readValue();
        ParamUp.readValue();
        ParamDown.readValue();
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
        if (ParamUp.wasJustPressed()) SetParam(GetParam() + Pas());
        if (ParamDown.wasJustPressed()) SetParam(GetParam() - Pas());
    }

    public void ShowTelemetry() {
        telemetry.addData("Shooter pornit", Pornit);
        telemetry.addData("Shooter viteza setata", VitezaSetata);
        telemetry.addData("Shooter viteza tinta", TargetVelocity);
        telemetry.addData("Shooter viteza reala motor 1", GetVelocity());
        telemetry.addData("Shooter viteza reala motor 2", GetVelocity2());
        telemetry.addData("Shooter eroare motor 1", TargetVelocity - GetVelocity());
        telemetry.addData("Shooter eroare motor 2", TargetVelocity - GetVelocity2());
        telemetry.addData("Shooter gata de tras", IsReady());
        telemetry.addData("Shooter constanta aleasa", numeParam[paramAles]);
        telemetry.addData("Shooter P", P);
        telemetry.addData("Shooter I", I);
        telemetry.addData("Shooter D", D);
        telemetry.addData("Shooter F", F);
    }

    public void Run() {
        Controls();
        SetMotorsVelocity(TargetVelocity);
        ShowTelemetry();
    }
}
