package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Teleop_Exercise extends LinearOpMode {

    // sets variables for drive motors
    private DcMotor driveFL, driveFR, driveBL, driveBR;

    // creates variables for inputs from controllers
    private double forwardBackward, leftRight, rotate, strafeRight, strafeLeft;

    @Override
    public void runOpMode() {

        // calls initialize class to do init routine
        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // runs until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // sets values of variables for gamepad1 (start+a) inputs
            forwardBackward = -gamepad1.left_stick_y;
//            strafeRight = gamepad1.right_trigger;
//            strafeLeft = gamepad1.left_trigger;
            leftRight = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;

/*
            // mini-program on driving forward and backward using left stick
            driveFL.setPower(forwardBackward);
            driveFR.setPower(forwardBackward);
            driveBL.setPower(forwardBackward);
            driveBR.setPower(forwardBackward);

            // mini program for strafing left and right using left stick
            driveFL.setPower(leftRight);
            driveFR.setPower(-leftRight);
            driveBL.setPower(-leftRight);
            driveBR.setPower(leftRight);
*/
            // mini-program on driving and strafing using left stick
            driveFL.setPower(1);
            driveFR.setPower(1);
            driveBL.setPower(1);
            driveBR.setPower(1);
/*
            // mini-program on rotating using right stick
            driveFL.setPower(rotate);
            driveFR.setPower(-rotate);
            driveBL.setPower(rotate);
            driveBR.setPower(-rotate);

          // mini-program on strafing left and right using left and right triggers
            driveFL.setPower(strafeRight-strafeLeft);
            driveFR.setPower(strafeLeft-strafeRight);
            driveBL.setPower(strafeLeft-strafeRight);
            driveBR.setPower(strafeRight-strafeLeft);
*/



            // calls addTelemetry class to update telemetry
            addTelemetry();

        }
    }

    // initialization class
    private void initialize() {

        // maps drive motor variables to hardware configuration names
        driveFL = hardwareMap.get(DcMotor.class, "motorTestFL");
        driveFR = hardwareMap.get(DcMotor.class, "motorTestFR");
        driveBL = hardwareMap.get(DcMotor.class, "motorTestBL");
        driveBR = hardwareMap.get(DcMotor.class, "motorTestBR");

        // sets right motors to reverse direction so they're going the right way
        driveFR.setDirection(DcMotor.Direction.REVERSE);
        driveBR.setDirection(DcMotor.Direction.REVERSE);

        // sets drive motor zero power behavior to brake
        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialization Complete");
        telemetry.update();
    }

    // class to add and update telemetry
    private void addTelemetry() {
        telemetry.addData("Forward Backward Power", forwardBackward);
//      telemetry.addData("Strafe Power", leftRight);
        telemetry.addData("Rotate Power", rotate);
        telemetry.addData("Front Left Drive Motor Power", driveFL.getPower());
        telemetry.addData("Front Right Drive Motor Power", driveFR.getPower());
        telemetry.addData("Back Left Drive Motor Power", driveBL.getPower());
        telemetry.addData("Back Right Drive Motor Power", driveBR.getPower());
        telemetry.update();

        telemetry.addData("Status", "FAILED TO LOAD" );
        telemetry.update();
    }
}