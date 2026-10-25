package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Ce trebuie sa ofere o turela ca s-o poata folosi Shooter-ul, FieldDrawing si TeleOp-ul.
 * Fiecare varianta de turela (TurretPinpointPIDControl, TurretMotionCompensatedPID,
 * TurretBallisticShotPID) e o clasa completa, independenta, care implementeaza interfata asta.
 * Daca una nu mai e folosita, se poate sterge fara sa afecteze celelalte.
 */
public interface TurretShot extends Subsystem {

    /** Unghiul real al turelei acum (grade, + = stanga, 0 = TICKS_AT_CENTER). */
    double getCurrentAngle();

    /** Unghiul la care vrea codul sa stea turela (grade, + = stanga). */
    double getAimAngle();

    /** Unde priveste turela la TICKS_AT_CENTER fata de fata robotului (grade). */
    double getZeroHeadingDeg();

    /** Distanta folosita de shooter (metri). */
    double GetShotDistanceMeters();

    /** true = turela da direct viteza flywheel si pozitia hood (fara tabelul ShooterDistance). */
    boolean HasDirectShot();

    /** Viteza flywheel (tick/s), doar daca HasDirectShot(). */
    double GetFlywheelVelocity();

    /** Pozitia servo hood, doar daca HasDirectShot(). */
    double GetHoodServoPosition();

    void UpdateTelemetry(Telemetry telemetry);
}
