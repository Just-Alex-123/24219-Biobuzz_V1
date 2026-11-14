package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

public class AutonomousSubsystem {

    // --- Hardware Declarations ---
    private DcMotorEx Nshooter;
    private DcMotorEx Pshooter;
    private DcMotor intake;
    private DcMotor transfer;

    private Servo flower;
    private CRServo T1;   // Continuous Rotation Servo
    private CRServo T2;   // Continuous Rotation Servo
    private Servo flap;

    public AutonomousSubsystem(HardwareMap hardwareMap) {
        // Map Motors
        Nshooter = hardwareMap.get(DcMotorEx.class, "Nshooter");
        Pshooter = hardwareMap.get(DcMotorEx.class, "Pshooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");

        // Map Servos
        flower = hardwareMap.get(Servo.class, "flower");
        T1 = hardwareMap.get(CRServo.class, "T1");
        T2 = hardwareMap.get(CRServo.class, "T2");
        flap = hardwareMap.get(Servo.class, "flap");

        // Configure Motor Behaviors
        Nshooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Pshooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse directions if necessary
        Pshooter.setDirection(DcMotor.Direction.REVERSE);
        Nshooter.setDirection(DcMotor.Direction.REVERSE);

        // --- Initialize Independent PIDF Coefficients to 0.0 ---
        PIDFCoefficients nPIDF = new PIDFCoefficients(0.0, 0.0, 0.0, 0.0);
        PIDFCoefficients pPIDF = new PIDFCoefficients(0.0, 0.0, 0.0, 0.0);

        Nshooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, nPIDF);
        Pshooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pPIDF);
    }
    // --- Shooter Methods ---
    public void setShooterVelocities(double nVelocity, double pVelocity) {
        Nshooter.setVelocity(nVelocity);
        Pshooter.setVelocity(pVelocity);
    }
    public void stopShooters() {
        Nshooter.setPower(0.0);
        Pshooter.setPower(0.0);
    }

    // --- Intake & Transfer Methods ---
    public void runIntake(double power) {
        intake.setPower(power);
    }

    public void runTransfer(double power) {
        transfer.setPower(power);
    }

    public void stopIntakeSystem() {
        intake.setPower(0.0);
        transfer.setPower(0.0);
    }

    // --- Servo & Turret Methods ---
    public void setFlowerPosition(double position) {
        flower.setPosition(position);
    }

    // Since T1 and T2 are CRServos, they take power (-1.0 to 1.0) instead of position angles
    public void setTurretPower(double power1, double power2) {
        T1.setPower(power1);
        T2.setPower(power2);
    }

    public void stopTurrets() {
        T1.setPower(0.0);
        T2.setPower(0.0);
    }

    public void setFlapPosition(double position) {
        flap.setPosition(position);
    }
}