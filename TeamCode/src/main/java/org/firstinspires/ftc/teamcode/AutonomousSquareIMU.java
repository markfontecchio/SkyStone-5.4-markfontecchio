package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous Square IMU")

public class AutonomousSquareIMU extends LinearOpMode {

    // sets variables for drive motors, IMU, etc.
    DcMotor driveFL, driveFR, driveBL, driveBR;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle=0, localAngle=0, power = .50, correction;

    // called when the initialization button is  pressed
    @Override
    public void runOpMode() throws InterruptedException {
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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // maps imu variables to hardware configuration name and initializes imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        telemetry.addData("Status", "Initialization Complete");
        telemetry.update();

        // initialization code ends here

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)

        if (opModeIsActive()) {
//            for(double i=0;i<4;i++) {
//                driveFL.setPower(0.5);
//                driveFR.setPower(0.5);
//                driveBL.setPower(0.5);
//                driveBR.setPower(0.5);

                telemetry.addData("lastAngles before rotate 1", lastAngles);
                localAngle += globalAngle;
                telemetry.addData("local angle before rotate",localAngle);
                telemetry.addData("global angle before rotate",globalAngle);
                telemetry.update();
                sleep(4000);

                rotate(85, power);
                telemetry.addData("lastAngles after rotate 1", lastAngles);
                localAngle += globalAngle;
                telemetry.addData("local angle after rotate 1", localAngle);
                telemetry.addData("global angle before rotate",globalAngle);
                telemetry.update();
                sleep(4000);

                rotate(85, power);
                telemetry.addData("lastAngles after rotate 2", lastAngles);
                localAngle += globalAngle;
                telemetry.addData("localangle after rotate 2", localAngle);
                telemetry.addData("global angle before rotate",globalAngle);
                telemetry.update();
                sleep(4000);

                rotate(-localAngle, power);
                telemetry.addData("lastAngles after rotate 3", lastAngles);
                localAngle += globalAngle;
                telemetry.addData("localangle after rotate 3", localAngle);
                telemetry.addData("global angle before rotate",globalAngle);
                telemetry.update();
                sleep(4000);

//            }

        }

    }

    private void rotate(double degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        driveBL.setPower(leftPower);
        driveFL.setPower(leftPower);
        driveFR.setPower(rightPower);
        driveBR.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // for right turn we have to get off zero first
            while (opModeIsActive() && getAngle() == 0) {}
            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn
             while (opModeIsActive() && getAngle() < degrees) {}


        // turn the motors off.
        driveFL.setPower(0);
        driveBL.setPower(0);
        driveFR.setPower(0);
        driveBR.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
//        resetAngle();
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
}