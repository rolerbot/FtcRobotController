package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "TestIntakeShooter", group = "TeleOp")
public class TestInstakeshooter extends OpMode {
    private DcMotorEx MotorAruncare, MotorIntake, MotorRidicareBila;
    TelemetryCustom myLogger;

    private final double shooterP = 0.15;
    private final double shooterI = 0.0;
    private final double shooterD = 8.0;
    private final double shooterF = 14.0;

    private final double TRANSFER_P = 10.0;
    private final double TRANSFER_I = 3.0;
    private final double TRANSFER_D = 0.0;
    private final double TRANSFER_F = 12.0;
    private final double TRANSFER_VELOCITY = 2800;

    @Override
    public void init() {
        myLogger = new TelemetryCustom(telemetry);
        MotorAruncare = hardwareMap.get(DcMotorEx.class, "MotorAruncare");
        MotorIntake = hardwareMap.get(DcMotorEx.class, "MotorIN");
        MotorRidicareBila = hardwareMap.get(DcMotorEx.class, "MotorRidicareBila");

        MotorAruncare.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorAruncare.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorAruncare.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients shooterPIDF = new PIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
        MotorAruncare.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        MotorRidicareBila.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorRidicareBila.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorRidicareBila.setDirection(DcMotorSimple.Direction.FORWARD);

        PIDFCoefficients transferPIDF = new PIDFCoefficients(TRANSFER_P, TRANSFER_I, TRANSFER_D, TRANSFER_F);
        MotorRidicareBila.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, transferPIDF);

        MotorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        telemetry.addData("Shooter Target", 1580);
        telemetry.addData("Shooter Current", MotorAruncare.getVelocity());
        telemetry.addData("Transfer Target", TRANSFER_VELOCITY);
        telemetry.addData("Transfer Current", MotorRidicareBila.getVelocity());
        telemetry.update();

        MotorIntake.setPower(1);
        MotorAruncare.setVelocity(1580);
        MotorRidicareBila.setVelocity(TRANSFER_VELOCITY);
    }
}
