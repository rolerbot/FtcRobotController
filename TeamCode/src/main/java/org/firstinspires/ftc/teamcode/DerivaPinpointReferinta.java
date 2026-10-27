package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Nimic pornit: deriva ar trebui sa fie ~0. Vezi DerivaPinpointTest. */
@TeleOp(name = "Deriva - Referinta (nimic pornit)", group = "Teste deriva")
public class DerivaPinpointReferinta extends DerivaPinpointTest {
    @Override protected double ShooterVelocity() { return 0; }
    @Override protected boolean RunMechanisms() { return false; }
    @Override protected String TestName() { return "Deriva - Referinta (nimic pornit)"; }
}
