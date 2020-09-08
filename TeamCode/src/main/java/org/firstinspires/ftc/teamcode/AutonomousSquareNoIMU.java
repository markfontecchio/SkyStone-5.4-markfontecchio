package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous Square No IMU")

public class AutonomousSquareNoIMU extends LinearOpMode {

    // sets variables for drive motors
    private DcMotor driveFL, driveFR, driveBL, driveBR;

    @Override
    public void runOpMode() {

        // initialization code starts here

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

        // initialization code ends here


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)

        if (opModeIsActive()) {
            for(double i=0;i<4;i++)
            {
                driveFL.setPower(0.5);
                driveFR.setPower(0.5);
                driveBL.setPower(0.5);
                driveBR.setPower(0.5);

                sleep(1000);

                driveFL.setPower(-0.5);
                driveFR.setPower(0.5);
                driveBL.setPower(-0.5);
                driveBR.setPower(0.5);

                sleep(1400);

            }

            driveFL.setPower(0.0);
            driveFR.setPower(0.0);
            driveBL.setPower(0.0);
            driveBR.setPower(0.0);

            // telemetry
            telemetry.addData("Front Left Drive Motor Power", driveFL.getPower());
            telemetry.addData("Front Right Drive Motor Power", driveFR.getPower());
            telemetry.addData("Back Left Drive Motor Power", driveBL.getPower());
            telemetry.addData("Back Right Drive Motor Power", driveBR.getPower());
            telemetry.update();

            telemetry.addData("Status", "FAILED TO LOAD" );
            telemetry.update();

        }
    }
}