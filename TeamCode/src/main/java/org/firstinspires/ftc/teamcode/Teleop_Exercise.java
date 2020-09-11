package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Teleop_Exercise extends LinearOpMode {

    // sets variables for drive motors
    private DcMotor driveFL, driveFR, driveBL, driveBR;

    // creates variables for drive inputs from controllers
    private double forwardBackward, leftRight, rotate;

    @Override
    public void runOpMode() {

        // calls initialize class to do init routine
        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // runs until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // sets values of variables for gamepad1 (start+a) inputs
            forwardBackward = -gamepad1.left_stick_y;  // uses left stick to move forward and backward
            leftRight = gamepad1.left_stick_x;  // uses left stick to strafe left and right
//          leftRight = gamepad1.right_trigger - gamepad1.left_trigger    // uses triggers to strafe left and right
            rotate = gamepad1.right_stick_x;  // uses right stick to rotate

/*
            // drives forward and backward
            driveFL.setPower(forwardBackward);
            driveFR.setPower(forwardBackward);
            driveBL.setPower(forwardBackward);
            driveBR.setPower(forwardBackward);

            // strafes left and right
            driveFL.setPower(leftRight);
            driveFR.setPower(-leftRight);
            driveBL.setPower(-leftRight);
            driveBR.setPower(leftRight);
*/
           // drives forward/backward and strafes left/right
            driveFL.setPower(forwardBackward + leftRight);
            driveFR.setPower(forwardBackward - leftRight);
            driveBL.setPower(forwardBackward - leftRight);
            driveBR.setPower(forwardBackward + leftRight);
/*
            // rotates
            driveFL.setPower(rotate);
            driveFR.setPower(-rotate);
            driveBL.setPower(rotate);
            driveBR.setPower(-rotate);

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