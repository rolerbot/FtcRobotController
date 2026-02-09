package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

@TeleOp
public class TurretOPTutorial extends OpMode
{
    private TurretMechanismTutorial turret;
    private LimeLight limeLight;

    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    int stepIndex = 2;

    @Override
    public void init()
    {
        limeLight = new LimeLight(true, false);
        turret = new TurretMechanismTutorial(limeLight);

    }

    @Override
    public void start()
    {
        limeLight.Initialize(hardwareMap);
        turret.Initialize(hardwareMap);
    }

    @Override
    public void loop()
    {
        limeLight.Run();
        turret.Run();

        if(gamepad1.bWasPressed()) stepIndex = (stepIndex + 1) % stepSizes.length;

        if(gamepad1.dpadLeftWasPressed()) turret.SetKp(turret.GetKp() - stepSizes[stepIndex]);
        if(gamepad1.dpadRightWasPressed()) turret.SetKp(turret.GetKp() + stepSizes[stepIndex]);

        if(gamepad1.dpadUpWasPressed()) turret.SetKd(turret.GetKd() + stepSizes[stepIndex]);
        if(gamepad1.dpadDownWasPressed()) turret.SetKd(turret.GetKd() - stepSizes[stepIndex]);

        telemetry.addData("Step Index", stepIndex);
        telemetry.addData("Step Size", stepSizes[stepIndex]);
        telemetry.addData("Kp", turret.GetKp());
        telemetry.addData("Kd", turret.GetKd());
        telemetry.update();
    }
}
