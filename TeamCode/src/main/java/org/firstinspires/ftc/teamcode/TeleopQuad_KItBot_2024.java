/*
Starter Code for Quad Training Robot 2024
Modified by michaudc 2017, 2023, 2024
Arm Hold Codes revised with help from Mason Moore 2024
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="MaristBot2024: KitBot INTO THE DEEP", group="Training")
//@Disabled
public class TeleopQuad_KitBot_2024 extends OpMode {

    // Create instance of MaristBaseRobot2024
    MaristBaseRobot2024_Quad robot   = new MaristBaseRobot2024_Quad();
    
    double SPEED_CONTROL = 1;
    
    private int armPos = 0;
    private int sliderPos = 0;
    
    private double ARM_SPEED = 0.8;
    private double SLIDER_SPEED = 0.8;

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
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        robot.rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderPos = robot.rightArm.getCurrentPosition();
        
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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        
        // Driver Control Code
        double leftY = gamepad1.left_stick_y * SPEED_CONTROL;
        double leftX = gamepad1.left_stick_x * SPEED_CONTROL ;
        double rightX = gamepad1.right_stick_x *SPEED_CONTROL;
        
        robot.driveStrafer(leftX, leftY, rightX);
        
        // Left Arm Code
        double deltaArmPos = gamepad1.left_trigger-gamepad1.right_trigger;
        
        if (Math.abs(deltaArmPos) > 0.1) { // Arm is moving
            robot.leftArm.setMode(DcMotor.RUN_WITHOUT_ENCODER);
            robot.leftArm.setPower(deltaArmPos);
        }
        else { // Arm is holding Position
            armPos = robot.leftArm.getCurrentPosition();
            robot.leftArm.setTargetPosition(armPos);
            robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftArm.setPower(ARM_SPEED);
        }

        
        // Right Arm (Slider) Code
        if (gamepad1.y) { // Arm Up
            robot.rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightArm.setPower(-0.8);
        }
        else if (gamepad1.a) { // Arm Down
            robot.rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightArm.setPower(0.8);
        }
        else {
            sliderPos = robot.rightArm.getCurrentPosition();
            robot.rightArm.setTargetPosition(sliderPos);
            robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightArm.setPower(SLIDER_SPEED);
        }
        
        // Left Hand (Grasper) Code
        if (gamepad1.right_bumper) { // Close
            robot.leftHand.setPosition(0.15);
        }
        if (gamepad1.left_bumper) { // Open
            robot.leftHand.setPosition(0.75);
        }
        
        telemetry.addData("ArmPos", armPos);
        telemetry.addData("SliderPos", sliderPos);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    
}