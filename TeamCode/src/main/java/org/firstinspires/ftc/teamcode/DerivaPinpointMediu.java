package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Shooter la viteza medie din tabel (240 cm) + intake + transfer. Vezi DerivaPinpointTest. */
@TeleOp(name = "Deriva - Shooter MEDIU (1270)", group = "Teste deriva")
public class DerivaPinpointMediu extends DerivaPinpointTest {
    @Override protected double ShooterVelocity() { return 1270; }
    @Override protected boolean RunMechanisms() { return true; }
    @Override protected String TestName() { return "Deriva - Shooter MEDIU (1270)"; }
}
