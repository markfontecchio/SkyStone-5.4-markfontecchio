package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="Autonomous Maze")

public class AutonomousMaze extends LinearOpMode {

    private DcMotor driveFL, driveFR, driveBL, driveBR;
    private BNO055IMU imu;
    private double wheelDiameterInches = 4;
    private double encoderTicksPerRevolution = 28;
    private double wheelInchesPerRevolution = wheelDiameterInches * Math.PI;  // aka wheel circumference
    private double encoderTicksPerInch = encoderTicksPerRevolution / wheelInchesPerRevolution;

    @Override
    public void runOpMode() {

        // call function to initialize the robot
        initialize();

        //wait for user to press start
        waitForStart();

        if (opModeIsActive()) {

            // drive forward 24"
            // turn left 90 degrees
            // turnLeft(90);
            // drive forward 20"
            // driveForward(20);
            // turn right 90 degrees
            // turnRight(90);
            // drive forward 36"
            // driveForward(36);

        }


    }

/*    private void driveForward(int inches, double power) {
        driveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveFL.setTargetPosition(inches * encoderTicksPerInch);

        driveBR.setPower(power);

        while (opModeIsActive() && driveFL.isBusy()){
            telemetry.addData("Status", "Driving");
            telemetry.update();
            idle();
        }

        driveFL.setPower(0);

        telemetry.addData("Status", "Stopped");
        telemetry.update();
        sleep(250);

    }

 */

    private void initialize() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // map our variables to robot components+
        driveFL = hardwareMap.get(DcMotor.class, "motorTestFL");
        driveFR = hardwareMap.get(DcMotor.class, "motorTestFR");
        driveBL = hardwareMap.get(DcMotor.class, "motorTestBL");
        driveBR = hardwareMap.get(DcMotor.class, "motorTestBR");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        driveFR.setDirection(DcMotor.Direction.REVERSE);
        driveBR.setDirection(DcMotor.Direction.REVERSE);

        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

        telemetry.addData("Status", "Initialization Complete");
        telemetry.update();
    }


}
