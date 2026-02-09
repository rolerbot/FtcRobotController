package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class TurretMechanismTutorial implements Subsystem
{
    private DcMotorEx MotorTurela = null;
    private double kp = 0.0001;
    private double kd = 0.000;
    private LimeLight limeLight;
    private double lastError = 0;
    private double angleTolerance = 0.2;// Tolerance in degrees
    private final double MAX_POWER = 0.5; // Maximum power to prevent overshooting
    private double power = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public TurretMechanismTutorial(LimeLight limeLight)
    {
        this.limeLight = limeLight;
    }

    public void LinkComponents(HardwareMap hwMap)
    {
        MotorTurela = hwMap.get(DcMotorEx.class, "MotorTurela");
    }

    public void Initialize(HardwareMap hwMap)
    {
        MotorTurela.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        MotorTurela.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        MotorTurela.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void ResetTimer()
    {
        timer.reset();
    }

    public void Update()
    {
        double deltaTime = timer.seconds();
        ResetTimer();

        // Obține poziția robotului de la LimeLight (MT2 cu IMU fusion)
        double robotX = limeLight.GetRobotX();
        double robotY = limeLight.GetRobotY();
        double robotHeading = limeLight.GetRobotHeading(); // În radiani

        // Poziția targetului (coșul) - de la LimeLight
        double targetX = limeLight.GetTargetX();
        double targetY = limeLight.GetTargetY();

        // Calculează unghiul necesar pentru a privi spre target
        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        double angleToTarget = Math.atan2(deltaY, deltaX); // În radiani

        // Error = diferența dintre unghiul actual al robotului și unghiul dorit
        double error = angleToTarget - robotHeading;

        // Normalizează error-ul la [-PI, PI]
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;

        // Convertește error-ul în grade pentru controlul motorului
        double errorDegrees = Math.toDegrees(error);

        // PD Control
        double pTerm = kp * errorDegrees;

        double dTerm = 0;
        if(deltaTime > 0)
        {
            double derivative = (errorDegrees - lastError) / deltaTime;
            // Limitează derivata pentru a preveni spike-uri
            derivative = Range.clip(derivative, -100, 100);
            dTerm = kd * derivative;
        }

        if(Math.abs(errorDegrees) < angleTolerance)
        {
            power = 0;
            MotorTurela.setPower(0);
            lastError = 0; // Resetează pentru a preveni windup
        }
        else
        {
            power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);

            // Safety check cu encoder
            int currentPosition = MotorTurela.getCurrentPosition();
            int maxPosition = 1000;
            int minPosition = -1000;

            if((power > 0 && currentPosition >= maxPosition) ||
                    (power < 0 && currentPosition <= minPosition))
            {
                power = 0;
            }

            MotorTurela.setPower(power);
            lastError = errorDegrees;
        }
    }

    public void SetKp(double newkp) { kp = newkp; }
    public double GetKp() { return kp; }
    public void SetKd(double newkd) { kd = newkd; }
    public double GetKd() { return kd; }

    public double GetCurrentError() { return lastError; }
    public double GetCurrentPower() { return power; }

    public void Run() { Update(); }
}