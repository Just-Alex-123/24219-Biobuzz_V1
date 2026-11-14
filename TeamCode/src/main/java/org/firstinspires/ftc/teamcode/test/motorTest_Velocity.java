package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class motorTest_Velocity extends LinearOpMode {

    private DcMotorEx motor;
//    private DcMotorEx fwR;

//     setVelocity() takes TICKS PER SECOND (not RPM)
    private double targetVelocity = 600;
    private static final double STEP = 50;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
//        fwR = hardwareMap.get(DcMotorEx.class, "fwR");

        motor.setDirection(DcMotorSimple.Direction.FORWARD);
//        fwR.setDirection(DcMotorSimple.Direction.REVERSE);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        fwR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // FIX 1: velocity control only works in RUN_USING_ENCODER
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        fwR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        fwR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean xWasPressed = false;
        boolean yWasPressed = false;

        waitForStart();

        while (opModeIsActive()) {

            // FIX 2: adjust target on a button press (edge), no sleep(), works while spinning
            if (gamepad1.x && !xWasPressed) targetVelocity += STEP;
            if (gamepad1.y && !yWasPressed) targetVelocity -= STEP;
            xWasPressed = gamepad1.x;
            yWasPressed = gamepad1.y;
            targetVelocity = Math.max(0, targetVelocity);

            // FIX 3: motors are always told what to do, so x/y can't leave them running
            if (gamepad1.left_bumper) {
                motor.setVelocity(targetVelocity);
//                fwR.setVelocity(targetVelocity);
            } else {
                motor.setVelocity(0);
//                fwR.setVelocity(0);
            }

            telemetry.addData("target velocity", targetVelocity);
            telemetry.addData("actual velocity left", motor.getVelocity());
//            telemetry.addData("actual velocity right", fwR.getVelocity());
            telemetry.addData("left power / position", "%.2f / %d", motor.getPower(), motor.getCurrentPosition());
//            telemetry.addData("right power / position", "%.2f / %d", fwR.getPower(), fwR.getCurrentPosition());
            telemetry.update();
        }
    }
}