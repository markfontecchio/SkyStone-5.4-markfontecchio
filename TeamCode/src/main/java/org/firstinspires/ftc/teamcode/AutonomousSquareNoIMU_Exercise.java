package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Autonomous Square No IMU Exercise")

public class AutonomousSquareNoIMU_Exercise extends LinearOpMode {

    // sets variables for drive motors
    private DcMotor driveFL, driveFR, driveBL, driveBR;

    @Override
    public void runOpMode() {

        // calls initialize class to do init routine
        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)

        if (opModeIsActive())
        {
            // autonomous code goes here


            composeTelemetry();
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
    private void composeTelemetry() {
        telemetry.addData("Front Left Drive Motor Power", driveFL.getPower());
        telemetry.addData("Front Right Drive Motor Power", driveFR.getPower());
        telemetry.addData("Back Left Drive Motor Power", driveBL.getPower());
        telemetry.addData("Back Right Drive Motor Power", driveBR.getPower());
        telemetry.update();
    }
}