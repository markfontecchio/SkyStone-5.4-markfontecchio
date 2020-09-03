package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class banana1 extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor motorTestFL;
    private DcMotor motorTestFR;
    private DcMotor motorTestBL;
    private DcMotor motorTestBR;

    // private DigitalChannel digitalTouch;
    // private DistanceSensor sensorColorRange;
    private Servo servoTest;
    @Override
    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        motorTestFL = hardwareMap.get(DcMotor.class, "motorTestFL");
        motorTestFR = hardwareMap.get(DcMotor.class, "motorTestFR");
        motorTestBL = hardwareMap.get(DcMotor.class, "motorTestBL");
        motorTestBR = hardwareMap.get(DcMotor.class, "motorTestBR");

        // reverses motor direction for right motors so they're going the right way
        motorTestFR.setDirection(DcMotor.Direction.REVERSE);
        motorTestBR.setDirection(DcMotor.Direction.REVERSE);

        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        servoTest = hardwareMap.get(Servo.class, "servoTest");
        telemetry.addData("Status", "Initialized ETHEAN");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        //double tgtPower=0;
        //double tgtPower2 = 0;
        double forwardBackward, leftRight;

        while (opModeIsActive()) {
            forwardBackward = -gamepad1.left_stick_y;
            leftRight = gamepad1.left_stick_x;

            motorTestFL.setPower(forwardBackward + leftRight);
            motorTestFR.setPower(forwardBackward - leftRight);
            motorTestBL.setPower(forwardBackward - leftRight);
            motorTestBR.setPower(forwardBackward + leftRight);



/*            tgtPower = -this.gamepad1.left_stick_x;
            tgtPower2 = this.gamepad1.right_stick_y;
            //motorTest.setPower(tgtPower);
            motorTestFL.setPower(tgtPower);
            motorTestFR.setPower(-tgtPower);
            motorTestBL.setPower(tgtPower);
            motorTestBR.setPower(-tgtPower);

            motorTestFL.setPower(tgtPower2);
            motorTestFR.setPower(tgtPower2);
            motorTestBL.setPower(tgtPower2);
            motorTestBR.setPower(tgtPower2);





            // check to see if we need to move the servo.
            if(gamepad1.y) {
                // move to 0 degrees.
                servoTest.setPosition(0);
            } else if (gamepad1.x || gamepad1.b) {
                // move to 90 degrees.
                servoTest.setPosition(0.5);
            } else if (gamepad1.a) {
                // move to 180 degrees.
                servoTest.setPosition(1);
            }

            /// move side to side
            if (gamepad1.x) {
                motorTestFL.setPower(-1);
                motorTestFR.setPower(-1);
                motorTestBL.setPower(1);
                motorTestBR.setPower(1);
            }

            if(gamepad1.b) {
                motorTestFL.setPower(1);
                motorTestFR.setPower(1);
                motorTestBL.setPower(-1);
                motorTestBR.setPower(-1);
            }


            if(gamepad1.y) {
                motorTestFL.setPower(1);
                motorTestFR.setPower(-1);
                motorTestBL.setPower(1);
                motorTestBR.setPower(-1);
            }
            if(gamepad1.a) {
                motorTestFL.setPower(-1);
                motorTestFR.setPower(1);
                motorTestBL.setPower(-1);
                motorTestBR.setPower(1);

            }
            if(gamepad1 .right_trigger >.1) {
                motorTestFL.setPower(1);
                motorTestFR.setPower(-1);
                motorTestBL.setPower(1);
                motorTestBR.setPower(-1);
            }


            if(gamepad1.left_trigger >.1) {
                motorTestFL.setPower(-1);
                motorTestFR.setPower(1);
                motorTestBL.setPower(-1);
                motorTestBR.setPower(1);
            }

            if(gamepad1.left_bumper) {
                motorTestFL.setPower(-1);
                motorTestFR.setPower(-1);
                motorTestBL.setPower(1);
                motorTestBR.setPower(1);
            }

            if(gamepad1.right_bumper) {
                motorTestFL.setPower(1);
                motorTestFR.setPower(1);
                motorTestBL.setPower(-1);
                motorTestBR.setPower(-1);
            }*/











            telemetry.addData("Servo Position", servoTest.getPosition());
            telemetry.addData("Forward Backward Power", forwardBackward);
            //telemetry.addData("Motor Power", motorTest.getPower());
            telemetry.addData("Motor Power FL", motorTestFL.getPower());
            telemetry.addData("Motor Power FR", motorTestFR.getPower());
            telemetry.addData("Motor Power BL", motorTestBL.getPower());
            telemetry.addData("Motor Power BR", motorTestBR.getPower());

            telemetry.addData("Status", "FAILED TO LOAD" );
            telemetry.update();

        }
    }
}
//}