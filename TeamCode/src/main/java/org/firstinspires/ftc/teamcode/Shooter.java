package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.LinkedList;
import java.util.Queue;

public class Shooter implements Subsystem{
    private ButtonReader Aruncare;
    private ButtonReader ThrowGreen, ThrowPurple;
    private final GamepadEx ct1;
    private Husky husky;
    private final double initialPosition = 0; // 0 si 0.032 - final
    private final double finalPosition = 0.37;
    private ElapsedTime runtime = new ElapsedTime();
    boolean isShooting = false;
    boolean shooterPrepare = false;
    private final Mixer mixer;
    private final Intake intake;
    private final TelemetryCustom telemetry;
    private boolean preparingLaunch = false;
    public Servo ServoRidicare = null;
    /// Motor Aruncare
    public DcMotorEx MotorAruncare1 = null;
    public DcMotorEx MotorAruncare2 = null;
    private double motorPower = 0.6;
    private ArtifactArrangement shootingOrder = ArtifactArrangement.None;
    private double[] artPoz = {0.0206 + 0.3834 + 0.3834 / 2, 0.0206 + 0.3834 / 2, 0.0206 + 2 * 0.3834 + 0.3834 / 2}; //pregatire, +0.03834 / 2 -pentru aruncare
    private boolean shootingAllowed = false;
    private ElapsedTime launchTime = new ElapsedTime();
    ButtonReader Aruncare2;
    private boolean launchtest = false;
    Queue<Color> artifactBuffer = new LinkedList<Color>();
    private boolean enqueued = false;

    private void PozTester()
    {
        Aruncare2.readValue();
        if(Aruncare2.wasJustPressed())
        {
            intake.SetMotorPower(0.5);
            launchtest = true;
            launchTime.reset();
        }
        if(launchtest && launchTime.seconds() > 1 && launchTime.seconds() < 2)
            mixer.SetPozition(artPoz[0]);
        else if(launchtest &&  launchTime.seconds() > 2 && launchTime.seconds() < 3)
            mixer.SetPozition(artPoz[1]);
        else if(launchtest &&  launchTime.seconds() > 3 && launchTime.seconds() < 4)
        {
            mixer.SetPozition(artPoz[2]);
            launchtest = false;
        }
    }
    public Shooter(TelemetryCustom tl, Mixer mixer,Intake intk,Husky husky,GamepadEx ct1)
    {
        this.ct1 = ct1;
        this.mixer = mixer;
        this.intake = intk;
        this.telemetry = tl;
        this.husky = husky;
    }
     public void LinkComponents(HardwareMap hardwareMap)
     {
        Aruncare = new ButtonReader(ct1, GamepadKeys.Button.A);
        ThrowGreen = new ButtonReader(ct1, GamepadKeys.Button.DPAD_LEFT);
        ThrowPurple = new ButtonReader(ct1, GamepadKeys.Button.DPAD_RIGHT);
        Aruncare2 = new  ButtonReader(ct1, GamepadKeys.Button.LEFT_BUMPER);
        ServoRidicare = hardwareMap.get(Servo.class, "ServoRidicare");
        MotorAruncare1 = hardwareMap.get(DcMotorEx.class, "MotorAruncare1");
        MotorAruncare2 = hardwareMap.get(DcMotorEx.class, "MotorAruncare2");
    }
    public void Initialize(HardwareMap hwMap)
    {
        LinkComponents(hwMap);
        MotorAruncare1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorAruncare1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorAruncare1.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorAruncare2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorAruncare2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorAruncare2.setDirection(DcMotorSimple.Direction.REVERSE);
        ServoRidicare.setDirection(Servo.Direction.REVERSE);
        ServoRidicare.setPosition(initialPosition);
    }
    public void Run()
    {
        Aruncare.readValue();

        if(Aruncare.wasJustPressed())
        {
            if (!IsAllowedToShoot()) // check to not toggle back while running
                ToggleAllowShooting();
            telemetry.Log("Toggle:", shootingAllowed);
            if (CanShootArranged()) // setup for ordered shooting
            {
                ArtifactArrangement targetArr = husky.GetTargetArrangement();
                telemetry.Log("Aranjament: ", targetArr.toString());
                setShootingOrder(targetArr);
            }
        }
        if (IsAllowedToShoot())
        {
            PrepareLaunch();
            Shooting();
        }
         PozTester();
    }
    private boolean IsAllowedToShoot(){
        return shootingAllowed;
    }
    private void ToggleAllowShooting(){
        if (shootingAllowed) shootingAllowed = false;
        else shootingAllowed = true;
    }
    private void setShootingOrder(ArtifactArrangement targetArr)
    {
        shootingOrder = targetArr;
    }
    private boolean IsValidOrder(ArtifactArrangement order)
    {
        if (order == ArtifactArrangement.None)
            return false;
        return true;
    }
    public void SetPositionLever(double position){
        ServoRidicare.setPosition(position);
    }

    private void ResetTimer(){
        runtime.reset();
    }
    public void PowerShooterMotors(double power)
    {
        MotorAruncare1.setPower(power);
        MotorAruncare2.setPower(power);
    }

    private void StopShooterMotors(){
        preparingLaunch = false; // EVIDENT
        PowerShooterMotors(0);
    }

    public boolean GetIsShooting() {return isShooting;}

    public boolean IsNotShooting() {return !isShooting;}
    private void Shooting() // arunca mignile in ordine, sau fara daca nu gaseste una
    {
//        telemetry.Log("I am shooting! IsValidOrder, IsNotShooting, ShootingOrder", String.format("%b %b %s", IsValidOrder(shootingOrder), IsNotShooting(), shootingOrder.toString()));
        if (!IsValidOrder(shootingOrder) && !enqueued)
            ShootUnordered();
        else if (IsValidOrder(shootingOrder) && !enqueued)
        {
            EnqueueOrder(shootingOrder);
            enqueued = true;
            telemetry.Log("Enqueued balls!", "");
        }
        else if (enqueued){
            boolean isDone = ShootOrdered(artifactBuffer.peek());
            if (isDone)
            {
                telemetry.Log("trying to shoot object", artifactBuffer.peek().toString());
                artifactBuffer.remove();
                if (artifactBuffer.isEmpty())
                {
                    telemetry.Log("artifactBuffer is empty!", "");
                    isShooting = false;
                    enqueued = false;
                }
            }
        }
        else if (IsNotShooting())
        {
            shootingAllowed = false;
            telemetry.Log("Disallowed shooting, can press button again!", "");
        } /// else do nothing
    }
/**
 * ShootingOrdered: Purple
 * 22:50:19.414, 13.911324231| I am shooting! IsValidOrder, IsNotShooting, ShootingOrder: false false None
 * 22:50:19.415, 13.911828523| trying to shoot object: Purple
 * 22:50:19.415, 13.912262232| I am shooting! IsValidOrder, IsNotShooting, ShootingOrder: false false None
 * 22:50:19.416, 13.912689232| trying to shoot object: Green
 * 22:50:19.416, 13.913200815| I am shooting! IsValidOrder, IsNotShooting, ShootingOrder: false false None
 * 22:50:19.417, 13.913474398| trying to shoot object: Purple
 * 22:50:19.417, 13.913965857| I am shooting! IsValidOrder, IsNotShooting, ShootingOrder: false false None
 * 22:50:19.417, 13.914266857| trying to shoot object: Purple
 * 22:50:19.418, 13.914748982| I am shooting! IsValidOrder, IsNotShooting, ShootingOrder: false false None
 * 22:50:19.418, 13.915129024| trying to shoot object: Purple
 * 22:50:19.419, 13.915594232| I am shooting! IsValidOrder, IsNotShooting, ShootingOrder: false false None
 * 22:50:19.419, 13.915873065| trying to shoot object: Green
 * */
    private boolean ShootOrdered(Color color) // arunca in ordinea data de husky
    {
        /// should not change while executing for a single artifact
        int position = mixer.GetColorPosition(color);
        double servoPosition = artPoz[position];
        if (IsLaunchPrepared() && !isShooting && !mixer.IsEmpty())
        {
            isShooting = true;
            telemetry.Log("ShootingOrdered", color.toString());
            PowerShooterMotors(this.motorPower);
            ResetTimer();
            mixer.SetPozition(servoPosition); // muta mixerul pentru pozitia corecta
            return false;
        }
        if (isShooting && runtime.seconds() > 0.8 && runtime.seconds() <= 1.2) // trage
        {
            SetPositionLever(finalPosition);
            return false;
        }
        else if (isShooting && runtime.seconds() > 1.2 && runtime.seconds() <= 1.4) // coboara
        {
            SetPositionLever(initialPosition);
            return false;
        }
        else if (isShooting && runtime.seconds() > 1.4 && runtime.seconds() < 1.6) //se roteste
        {
            isShooting = false;
            mixer.RemoveArtifact(position);
            if (mixer.IsEmpty())
            {
                StopShooterMotors();
                preparingLaunch = false;
                if (intake.IsForward())
                    intake.SetPowerMax();
                mixer.ResetServoPosition();
                shooterPrepare = false;
            }
            ResetTimer();
            return true;
        }
        telemetry.Log("wtf?", "");
        return false;
    }
    private void ShootUnordered()
    {
        ///  preparingLaunch, isShooting, !mixer.IsEmpty(): true true false
//        telemetry.Log("preparingLaunch, isShooting, !mixer.IsEmpty()", String.format("%b %b %b", preparingLaunch, isShooting, mixer.IsEmpty()));
        if (preparingLaunch && IsNotShooting() && !mixer.IsEmpty())
        {
            isShooting = true;
            telemetry.Log("ShootingUnordered", "");
            PowerShooterMotors(this.motorPower);
            ResetTimer();
            mixer.NextPosition(); // pregateste sa traga
        }
        if (isShooting && runtime.seconds() > 0.7 && runtime.seconds() <= 1) // trage
        {
            telemetry.Log("Setting lever up", "");
            SetPositionLever(finalPosition);
        } else if (isShooting && runtime.seconds() > 1 && runtime.seconds() <= 1.25) // coboara
            SetPositionLever(initialPosition);
        else if (isShooting && runtime.seconds() > 1.25 && runtime.seconds() < 1.5)
        {
            isShooting = false;
            mixer.RemoveArtifact();
            if (mixer.IsEmpty())
            {
                StopShooterMotors();
                preparingLaunch = false;
                if (intake.IsForward())
                    intake.SetPowerMax();
                mixer.ResetServoPosition();
                telemetry.Log("Poz Servos", mixer.GetServoPosition());
                shooterPrepare = false;
            }
            else mixer.NextPosition();
            ResetTimer();
        }
    }
    private boolean CanShootArranged() //verifica daca poate trage in ordinea data de husky
    {
        if (mixer.IsEmpty())
            return false;
        return mixer.GetCountGreen() == 1 && mixer.GetCountPurple() == 2 && husky.GetID() != 0;
    }
    private void EnqueueOrder(ArtifactArrangement arr) // arunca in ordinea data de husky
    {
        /// adds ball order to buffer (queue)
        switch (arr){
            case None:
                return;
            case GPP:
            {
                ShootColor(Color.Green);
                ShootColor(Color.Purple);
                ShootColor(Color.Purple);
            }
            case PGP:
            {
                ShootColor(Color.Purple);
                ShootColor(Color.Green);
                ShootColor(Color.Purple);
            }
            case PPG:
            {
                ShootColor(Color.Purple);
                ShootColor(Color.Purple);
                ShootColor(Color.Green);
            }
        }
        setShootingOrder(ArtifactArrangement.None); // reset order after shooting
    }
    private void ShootColor(Color color){
        artifactBuffer.add(color);
    }

    private void PrepareLaunch()
    {
        if(!mixer.IsEmpty() && !shooterPrepare && !preparingLaunch)
        {
            preparingLaunch = true;
            PowerShooterMotors(0.45);
        }
    }
    private boolean IsLaunchPrepared()
    {
        return preparingLaunch;
    }
}
