
package org.firstinspires.ftc.teamcode.PedroAutoTurret;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;


@Autonomous(name = "AL15CazBun", group = "Autonomous")
@Configurable // Panels
public class AL15CazBun extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private int nextPhase = 0; // Next phase in the autonomous sequence
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(57, 9, Math.toRadians(180)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {

        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain ShootingPos;
        public PathChain FirstStackPos;
        public PathChain PickUpStack;
        public PathChain PickUpHumanBalls1;
        public PathChain GoBackHuman;
        public PathChain PickUpHumanBalls2;
        public PathChain Park;

        //Order of paths:
        /*

         ShootingPos; 0
         PickUpHumanBalls1; 3
         GoBackHuman; 4
         PickUpHumanBalls2; 5
         ShootingPos; 0
         Park;  6
         */

        public Paths(Follower follower) {
            ShootingPos = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57.000, 9.000),

                                    new Pose(48.000, 11.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            FirstStackPos = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 11.000),

                                    new Pose(41.745, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            PickUpStack = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(41.745, 36.000),

                                    new Pose(11.211, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            PickUpHumanBalls1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.000, 11.000),
                                    new Pose(28.341, 9.247),
                                    new Pose(11.500, 12.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            GoBackHuman = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(11.500, 12.000),

                                    new Pose(28.500, 11.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            PickUpHumanBalls2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(28.500, 11.000),
                                    new Pose(15.800, 9.000),
                                    new Pose(10.000, 9.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Park = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 11.000),

                                    new Pose(35.000, 12.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        nextPhase++;
    }

    public void autonomousPathUpdate() {
        switch (pathState)
        {
            // Shooting Position or Park
            case 0:
                follower.setMaxPower(0.7);
                if (!follower.isBusy()) {

                    // First time shooting
                    if(nextPhase == 0) {follower.followPath(paths.ShootingPos); setPathState(1);}

                    // Shoot again for at least 2 times
                    else if(nextPhase < 3) {follower.followPath(paths.ShootingPos); setPathState(4);}

                    // After shooting at least 2 times, go park
                    else {follower.followPath(paths.Park); setPathState(6);}
                }
                break;

            // First  Stack Position
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.FirstStackPos);
                    setPathState(2);
                }
                break;

            // Pick  Up Stack
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PickUpStack);
                    setPathState(3);
                }
                break;

            // Shooting Pose
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootingPos);
                    setPathState(0);
                    nextPhase++;
                }
                break;

            //   Go Back From Human Player
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GoBackHuman);
                    setPathState(5);
                }
                break;

            // Pick Up Human Balls2
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PickUpHumanBalls2);
                    setPathState(3);
                }
                break;

            //After Park
            case 6:
                if (!follower.isBusy()) {
                    setPathState(7);
                }
                break;
        }
    }

}
    