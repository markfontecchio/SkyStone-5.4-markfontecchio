package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous Square Encoder")

public class AutonomousEncoder extends LinearOpMode {
    //RLJ1 Test
    //Maahish
    //AK
    //AS
    // sets variables for drive motors
    private DcMotor driveFL, driveFR, driveBL, driveBR;

    @Override
    public void runOpMode() {

        // initialization code starts here

        // maps drive motor variables to hardware configuration names
        //RLJ
        //Matthew
        //NP
        driveFL = hardwareMap.get(DcMotor.class, "motorTestFL");
        driveFR = hardwareMap.get(DcMotor.class, "motorTestFR");
        driveBL = hardwareMap.get(DcMotor.class, "motorTestBL");
        driveBR = hardwareMap.get(DcMotor.class, "motorTestBR");// sets right motors to reverse direction so they're going the right way
        driveFR.setDirection(DcMotor.Direction.REVERSE);
        driveBR.setDirection(DcMotor.Direction.REVERSE);

        // sets drive motor zero power behavior to brake
        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        telemetry.addData("Status", "Initialization Complete");
        telemetry.update();

        double ticksperinch = 800 / (3.5 * Math.PI);

        // initialization code ends here


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)

        if (opModeIsActive()) {


        }
    }

    public void encoderDrive(double speed, int leftInches, int rightInches) {
        // Put run blocks here.
        //Set the positino of the 4 motors
        driveFL.setTargetPosition(leftInches * 90);
        driveFR.setTargetPosition(rightInches * 90);
        driveBL.setTargetPosition(leftInches * 90);
        driveBR.setTargetPosition(rightInches * 90);

        //Stop and reset all the motor encoders
        driveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set to use encoder
        driveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set motor powers
        driveFL.setPower(speed);
        driveFR.setPower(speed);
        driveBL.setPower(speed);
        driveBR.setPower(speed);

        //Run to position
        driveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //while (opModeIsActive() && (driveFL.isBusy() || driveBL.isBusy() || driveFR.isBusy() || driveBR.isBusy
        while (opModeIsActive() && (driveFL.isBusy() || driveFR.isBusy() || driveBL.isBusy() || driveBR.isBusy() )) {
            // Put loop blocks here.

            //left inches is negative
            if ((leftInches<0) && (driveFL.getCurrentPosition()>(driveFL.getTargetPosition()+100))){
                driveBL.setPower(-speed);

            }else if ((leftInches<0) && (driveFL.getCurrentPosition()<(driveFL.getTargetPosition()+100))) {
                driveBL.setPower(0);
            }

            //left inches is positive
            if ((leftInches>0) && (driveFL.getCurrentPosition()<(driveFL.getTargetPosition()-100))){
                driveBL.setPower(speed);

            }else if ((leftInches>0) && (driveFL.getCurrentPosition()>(driveFL.getTargetPosition()-100))) {
                driveBL.setPower(0);
            }


            //right inches is negative
            if ((rightInches<0) && (driveFR.getCurrentPosition()>(driveFR.getTargetPosition()+100))){
                driveBR.setPower(-speed);

            }else if ((rightInches<0) && (driveFR.getCurrentPosition()<(driveFR.getTargetPosition()+100))) {
                driveBR.setPower(0);
            }

            //right inches is positive
            if ((rightInches>0) && (driveFR.getCurrentPosition()<(driveFR.getTargetPosition()-100))){
                driveBR.setPower(speed);

            }else if ((rightInches>0) && (driveFR.getCurrentPosition()>(driveFR.getTargetPosition()-100))) {
                driveBR.setPower(0);
            }

            telemetry.addData("left 1 CP", driveFL.getCurrentPosition());
            telemetry.addData("left 2 power", driveBL.getPower());
            telemetry.addData("right 1 CP", driveFR.getCurrentPosition());
            telemetry.addData("right 2 power", driveBR.getPower());

            telemetry.addData("left 1 TP", driveFL.getTargetPosition());
            //telemetry.addData("left 2", driveBL.getTargetPosition());
            telemetry.addData("right 1 TP", driveFR.getTargetPosition());
            //telemetry.addData("right 2", driveBR.getTargetPosition());
            telemetry.update();
        }
        //Stop motors
        driveFL.setPower(0);
        driveBL.setPower(0);
        driveFR.setPower(0);
        driveBR.setPower(0);

        //get out of encoder mode
        driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

}