package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Shooter la viteza minima din tabel (120 cm) + intake + transfer. Vezi DerivaPinpointTest. */
@TeleOp(name = "Deriva - Shooter MIC (1110)", group = "Teste deriva")
public class DerivaPinpointMic extends DerivaPinpointTest {
    @Override protected double ShooterVelocity() { return 1110; }
    @Override protected boolean RunMechanisms() { return true; }
    @Override protected String TestName() { return "Deriva - Shooter MIC (1110)"; }
}
