package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test Mixer Positions", group = "Test")
public class TextMixerPositions extends OpMode {

    private Servo mixerServo;
    private DcMotorEx mixerEncoder;

    private final double offsetPositionMixer = 0.09153;
    private final double initialPosMixer = 0.0827 + 2 * offsetPositionMixer;

    private final double[] artPozOrdered = {
            initialPosMixer - offsetPositionMixer,      // second ball
            initialPosMixer + offsetPositionMixer,      // first ball
            initialPosMixer + 3 * offsetPositionMixer,  // idx2
            initialPosMixer + 5 * offsetPositionMixer,  // idx3
            initialPosMixer + 7 * offsetPositionMixer   // idx4
    };

    private int currentIdx = 0;
    private boolean lastA = false;
    private boolean lastB = false;

    @Override
    public void init() {
        mixerServo = hardwareMap.get(Servo.class, "ServoMixer1"); // change name if needed
        mixerEncoder = hardwareMap.get(DcMotorEx.class, "MotorIN"); // change name if needed

        mixerServo.setPosition(initialPosMixer);
        telemetry.addData("Status", "Initialized at initialPosMixer");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean pressedA = gamepad1.a;
        boolean pressedB = gamepad1.b;

        // A = next position
        if (pressedA && !lastA) {
            currentIdx = (currentIdx + 1) % 5;
            mixerServo.setPosition(artPozOrdered[currentIdx]);
        }

        // B = previous position
        if (pressedB && !lastB) {
            currentIdx = (currentIdx + 4) % 5;
            mixerServo.setPosition(artPozOrdered[currentIdx]);
        }

        lastA = pressedA;
        lastB = pressedB;

        telemetry.addData("Current idx", currentIdx);
        telemetry.addData("Servo pos", artPozOrdered[currentIdx]);
        telemetry.addData("Encoder", mixerEncoder.getCurrentPosition());
        telemetry.addData("", "A = next, B = prev");
        telemetry.addData("idx0", String.format("%.5f", artPozOrdered[0]));
        telemetry.addData("idx1", String.format("%.5f", artPozOrdered[1]));
        telemetry.addData("idx2", String.format("%.5f", artPozOrdered[2]));
        telemetry.addData("idx3", String.format("%.5f", artPozOrdered[3]));
        telemetry.addData("idx4", String.format("%.5f", artPozOrdered[4]));
        telemetry.update();
    }
}