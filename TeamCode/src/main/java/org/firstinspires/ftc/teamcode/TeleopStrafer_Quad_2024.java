/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Modified by michaudc 2017
Additional modifications by michaudc 2021
Modifications for CENTERSTAGE by michaudc 2023
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MaristBot2024: Teleop Strafer 2024", group="Training")
//@Disabled
public class TeleopStrafer_Quad_2024 extends OpMode {

    /* Declare OpMode members. */
    MaristBaseRobot2024_Quad robot   = new MaristBaseRobot2024_Quad(); // use the class created to define a Robot's hardware
                                                         // could also use HardwarePushbotMatrix class.
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    
    int armPos = 0;
    private double ARM_SPEED = 0.7;
    private double DRIVE_SPEED = 0.5;

    private double ClIMB_POWER = 0.8;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Robot Ready");    //

        // Set to Run without Encoder for Tele Operated
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        robot.leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armPos = robot.leftArm.getCurrentPosition();
        

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        armPos = 50; // Lifts Arm for Driving
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double leftX;
        double leftY;
        double rightX;

        // Strafer Mode
        /*
        leftX = gamepad1.left_stick_x * DRIVE_SPEED;
        leftY = gamepad1.left_stick_y * DRIVE_SPEED;
        rightX = gamepad1.right_stick_x * DRIVE_SPEED;
        
        double leftRearPower = leftY + leftX - rightX;
        double leftFrontPower = leftY - leftX - rightX;
        double rightRearPower = leftY - leftX + rightX;
        double rightFrontPower = leftY + leftX + rightX;
        
        robot.leftFront.setPower(leftFrontPower);
        robot.leftRear.setPower(leftRearPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightRear.setPower(rightRearPower);
        */


        // Strafer Mode
        leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y;
        rightX = gamepad1.right_stick_x ;
        //robot.driveFieldCentric(leftX, leftY, rightX, DRIVE_SPEED);
        robot.driveStrafer(leftX, leftY, rightX);

        
        // Use gamepad left & right Bumpers to open and close the claws
        // This assumes two Servos in opposite position
        if (gamepad1.right_bumper) {         // CLOSE
            robot.leftHand.setPosition(0);   // You will need to adjust these numbers 
            robot.rightHand.setPosition(1);  // You will need to adjust these numbers
        }
        else if (gamepad1.left_bumper) {    // OPEN
            robot.leftHand.setPosition(1);  // You will need to adjust these numbers 
            robot.rightHand.setPosition(0); // You will need to adjust these numbers 
        }
            
        // Use dpad buttons to move right arm motor
        
        if (gamepad1.dpad_up)
            robot.rightArm.setPower(ClIMB_POWER);
        else if (gamepad1.dpad_down)
            robot.rightArm.setPower(-ClIMB_POWER);
        else
            robot.rightArm.setPower(0.0);

        // Control Arm with Right and Left Triggers
        double armMotorPower = gamepad1.right_trigger - gamepad1.left_trigger;
        // Limit Power to -0.4 to 0.4
        if (armMotorPower > 0.1) {
            ARM_SPEED = 0.7;
            armPos += gamepad1.right_trigger * 10;
        }

        if (armMotorPower < -0.1) {
            ARM_SPEED = 0.7;
            armPos -= gamepad1.left_trigger * 10;
        }

        // Boundary Check - Adjust these numbers as needed
        if (armPos < 0) {
            armPos = 0;
        }

        if (armPos > 600) {
            armPos = 600;
        }

        // Presets: Adjust as Needed
        if (gamepad1.y)  {
            ARM_SPEED = 0.3;
            armPos = 500; // Arm Up
        }

        if (gamepad1.a) {
            ARM_SPEED = 0.3;
            armPos = 50; // Arm in Position for Driving / Down
        }

        //robot.leftArm.setPower(armMotorPower);
        robot.leftArm.setTargetPosition(armPos);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftArm.setPower(ARM_SPEED);

        

        // Send telemetry message to signify robot running;
        telemetry.addData("claw",  "Offset = %.2f", clawOffset);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}


