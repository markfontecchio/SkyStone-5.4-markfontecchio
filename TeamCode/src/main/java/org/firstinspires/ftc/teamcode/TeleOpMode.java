package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// indicates this is a teleop mode
@TeleOp

public class TeleOpMode extends LinearOpMode {

    // creates variables for all motors
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    // creates variables for gamepad1 control movements
    private double drive; // forward/backward
    private double strafe; // left/right
    private double turn; // rotate clockwise/counterclockwise

    @Override
    public void runOpMode(){

        // initialization code begins

        // initializes the hardware variables to correspond with robot configuration file
        leftFront = hardwareMap.get(DcMotor.class,"leftFront");
        leftBack = hardwareMap.get(DcMotor.class,"leftBack");
        rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        rightBack = hardwareMap.get(DcMotor.class,"rightBack");

        // set proper direction for all motors
        // you may need to switch the left and right directions depending on robot build
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // set all motors to zero power
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        // initialization code ends

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                // Sets values for gamepad1's driving control and variables. Controls:
                //      Up/down on left stick = forward/backward
                //      Left/right on left stick = strafe left/right
                //      Left/right on right stick = rotate counterclockwise/clockwise
                // drive = forward/backward, strafe = left/right, turn = clockwise/counterclockwise
                drive = -gamepad1.left_stick_y; // y direction must be reversed due to gamepad settings
                strafe = gamepad1.left_stick_x;
                turn = gamepad1.right_stick_x;

                // sets the proper power for each drive motor based on gamepad1 controls
                leftFront.setPower(drive + strafe + turn);
                leftBack.setPower(drive - strafe + turn);
                rightFront.setPower(drive - strafe - turn);
                rightBack.setPower(drive + strafe - turn);

                // sets values for gamepad2's controls for grabbing and stacking
                // gamepad2 code to go here

            }
        }
    }
}
