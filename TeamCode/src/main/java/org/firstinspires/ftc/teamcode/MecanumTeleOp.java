package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="MecanumTeleOp", group="FreightFrenzy")
public class MecanumTeleOp extends LinearOpMode {

    FrenzyHardwareMap robot = new FrenzyHardwareMap();

    @Override
    public void runOpMode() throws InterruptedException {

        //import the hardware map
        robot.init(hardwareMap, telemetry);

        //set the drivetrain slowdown
        double slowdown = 1.0;

        //set the controller stick values
        double controller1lstickx = 0.0;
        double controller1lsticky = 0.0;
        double controller1rstickx = 0.0;

        // init motor and add intake for arm
        robot.motorArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        int armCurrentPos = 0;
        int armSetPos = 0;
        int armMaxPos = 1200;

        //set arm levels
        int armLevel0 = 0;   // (a)
        int armLevel1 = 250; // (x)
        int armLevel2 = 600; // (y)
        int armLevel3 = 900; // (b)

        double armPower = 0.7;
        double armPowerLevels = 0.7;
        boolean gamepad2DPadDown = false; // track the state of the dpad

        //intake power
        double intakePower = 0.5;

        //carousel power
        double carouselPower = 0;

        //arm limit switch toggle to reset to 0
        boolean armLimitPressed;
        boolean armLimitSwitchFlag = true;

        //booleans for slow mode
        boolean bpressed = false;
        boolean slowdownflag = false;


        //init loop
        while (! isStarted()) {
            telemetry.addData("Arm Encoder", robot.motorArm.getCurrentPosition());
            telemetry.addData("Say GEX GEX GEX", "Hello Driver");
            telemetry.addData("Arm Limit Switch", robot.armLimitSwitch.isPressed());
            telemetry.update();
        }


        waitForStart();

        if (isStopRequested()) return;

        // Set the Arm position and motor mode once before the while loop
        robot.motorArm.setTargetPosition(0);
        robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive()) {

            /* Arm, Gamepad 2
             * Button A - Level 0, floor
             * Button B - Level 1, bottom level
             * Button C - Level 2, middle level
             * Button D - Level 3, top level
             * Rightstick Y - nudge up and down in small incremenets
             */
            armCurrentPos = robot.motorArm.getCurrentPosition();
            armLimitPressed = robot.armLimitSwitch.isPressed();
            gamepad2DPadDown = gamepad2.dpad_down; // track state of the dpad button

            // Magnetic Limit Switch
            if (armLimitPressed && !armLimitSwitchFlag && !gamepad2DPadDown){
                // Reset to 0 if the magnetic limit switch is pressed and our flag is set to false
                armSetPos = 0;
                robot.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLimitSwitchFlag = true;
                armPower = 0;// Added arm power 0

            }else if( armCurrentPos > 15 && !armLimitPressed) {
                // when arm is up set the limit switch flag to false to allow reset on next limit switch press
                armLimitSwitchFlag = false;
            }

            // Arm Up and Arm Down in small increments, >= 0.5 helps prevent issues with 0 value
            if (gamepad2.right_stick_y <= -0.5){
                armSetPos = armSetPos + 10;
                armPower = armPowerLevels;
            } else if (gamepad2.right_stick_y >= 0.5 && (!armLimitPressed && !armLimitSwitchFlag)) {
                armSetPos = armSetPos - 10;
            } else if(gamepad2DPadDown == true){
                armSetPos = armSetPos - 2;
                armPower = armPowerLevels;
            }


            //set arm positions to gamepad.2 buttons
            if (gamepad2.a) {
                armSetPos = armLevel0;
                armPower = armPowerLevels;
            }else if (gamepad2.x) {
                armSetPos = armLevel1;
                armPower = armPowerLevels;
            }else if (gamepad2.y) {
                armSetPos = armLevel2;
                armPower = armPowerLevels;
            }else if (gamepad2.b) {
                armSetPos = armLevel3;
                armPower = armPowerLevels;
            }

            //set arm power
            robot.motorArm.setTargetPosition(armSetPos);
            robot.motorArm.setPower(armPower); // needs to be slow otherwise is jerky


            /* Intake, Gamepad 2
             * Right trigger in at faster speed
             * Left Trigger out at a slower speed
             */
            if (gamepad2.right_trigger > 0.5){
                intakePower = -0.75;
            }
            else if (gamepad2.left_trigger > 0.5){
                intakePower = 0.35;
            }
            else if (gamepad2.left_trigger <= 0.5 && gamepad2.right_trigger <= 0.5){
                intakePower = 0;
            }
            //set power intake
            robot.motorIntake.setPower(intakePower);


            /* Carousel, Gamepad 2
             * Right bumper
             * Left Bumper
             */
            if (gamepad2.right_bumper == true && !gamepad2.left_bumper) {
                carouselPower = 0.5;
            }
            else if (gamepad2.left_bumper == true && !gamepad2.right_bumper) {
                carouselPower = -0.5;
            }
            else{
                carouselPower = 0;
            }
            //set power carousel
            robot.motorCarousel.setPower(carouselPower);


            /* Drivetrain Mecanum, Gamepad 1
             * Leftstick controls forward/back and strafing
             * Rightstick controls rotation
             * B button press toggles slowdown, cuts speed in half
             * Custom deadzone created to account for joystick drift
             */
            // Deadzone to correct drift
            controller1lstickx = gamepad1.left_stick_x;
            controller1lsticky = gamepad1.left_stick_y;
            controller1rstickx = gamepad1.right_stick_x;
            if(controller1lstickx < 0.3 && controller1lstickx > -0.3)
                controller1lstickx = 0.0;
            if(controller1lsticky < 0.3 && controller1lsticky > -0.3)
                controller1lsticky = 0.0;
            if(controller1rstickx < 0.3 && controller1rstickx > -0.3)
                controller1rstickx = 0.0;

            // set slowdown
            if(gamepad1.b == true && bpressed == false){
                if(slowdownflag){
                    slowdown = 1;
                    slowdownflag = false;
                }
                else if(slowdownflag == false){
                    slowdown = 0.5;
                    slowdownflag = true;
                }
            }
            bpressed = gamepad1.b;

            //Add slowdown
            controller1lstickx = controller1lstickx * slowdown;
            controller1lsticky = controller1lsticky * slowdown;
            controller1rstickx = controller1rstickx * slowdown;

            double y = controller1lsticky;
            double x = -controller1lstickx * 1.1; // * 1.1 Counteract imperfect strafing
            double rx = -controller1rstickx;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = ((y + x + rx) / denominator)*slowdown;
            double backLeftPower = ((y - x + rx) / denominator)*slowdown;
            double frontRightPower = ((y - x - rx) / denominator)*slowdown;
            double backRightPower = ((y + x - rx) / denominator)*slowdown;

            robot.motorFrontLeft.setPower(frontLeftPower);
            robot.motorBackLeft.setPower(backLeftPower);
            robot.motorFrontRight.setPower(frontRightPower);
            robot.motorBackRight.setPower(backRightPower);

            telemetry.addData("Left Stick X", controller1lstickx);
            telemetry.addData("Left Stick Y", controller1lsticky);
            telemetry.addData("Left Stick X input", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y input", gamepad1.left_stick_y);
            telemetry.addData("Left Stick Y", controller1lsticky);
            telemetry.addData("Arm Current pos", armCurrentPos);
            telemetry.addData("Arm Set pos", armSetPos);
            telemetry.addData("Arm LimitSwitch", robot.armLimitSwitch.isPressed());
            telemetry.addData("Arm LimitSwitchFlag", armLimitSwitchFlag);
            telemetry.addData("Right Y Stick",gamepad1.right_stick_y);
            telemetry.addData("Left Y Stick",gamepad1.left_stick_y);
            telemetry.addData("Slowdown:",slowdownflag);
            telemetry.update();
        }
    }
}