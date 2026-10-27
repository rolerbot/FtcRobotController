package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Shooter la viteza maxima din tabel (375 cm) + intake + transfer. Vezi DerivaPinpointTest. */
@TeleOp(name = "Deriva - Shooter MARE (1570)", group = "Teste deriva")
public class DerivaPinpointMare extends DerivaPinpointTest {
    @Override protected double ShooterVelocity() { return 1570; }
    @Override protected boolean RunMechanisms() { return true; }
    @Override protected String TestName() { return "Deriva - Shooter MARE (1570)"; }
}
