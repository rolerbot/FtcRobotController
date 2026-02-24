package org.firstinspires.ftc.teamcode.PedroAutoTurret;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.TurretPositionControl;

@Autonomous(name = "AInitOnly", group = "Autonomous")
@Configurable // Panels
public class AInitOnly extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;

    // Robot subsystems
    private TelemetryCustom telemetryCustom;
    private Intake intake;
    private LimeLight limeLight;
    private Shooter shooter;
    private Mixer mixer;
    private TurretPositionControl turret;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryCustom = new TelemetryCustom(telemetry);

        intake = new Intake(telemetryCustom);
        intake.Initialize(hardwareMap);

        // Initialize LimeLight for Blue alliance
        limeLight = new LimeLight(true, true);
        limeLight.Initialize(hardwareMap);

        mixer = new Mixer(telemetryCustom, intake);
        mixer.Initialize(hardwareMap);

        shooter = new Shooter(telemetryCustom, mixer, limeLight, false);
        shooter.Initialize(hardwareMap);

        turret = new TurretPositionControl(limeLight, shooter);
        turret.Initialize(hardwareMap, true); // Reset encoder at start (Right barrier = 0)
        turret.setTargetTicks(0); // Blue behavior: Stay at 0

        mixer.Reset(); // Ensure mixer is at home and encoder is zero

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0)); // Neutral starting pose for init-only

        panelsTelemetry.debug("Status", "Initialized - Blue Alliance (A)");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        follower.update();
        shooter.Run();
        limeLight.Run();
        turret.Run();

        panelsTelemetry.debug("Mode", "Only Init (Blue)");
        panelsTelemetry.debug("Turret Angle", turret.getCurrentAngle());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        if (shooter != null)
            shooter.StopShooterMotors();
    }
}
