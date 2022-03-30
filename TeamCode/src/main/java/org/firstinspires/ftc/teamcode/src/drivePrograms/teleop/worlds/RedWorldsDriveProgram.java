package org.firstinspires.ftc.teamcode.src.drivePrograms.teleop.worlds;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.src.utills.MiscUtils;
import org.firstinspires.ftc.teamcode.src.utills.enums.FreightFrenzyGameObject;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.TeleOpTemplate;

@TeleOp(name = "🟥Red Worlds Drive Program🟥")
public class RedWorldsDriveProgram extends TeleOpTemplate {
    public static BlinkinPattern defaultColor = BlinkinPattern.RED;
    private BlinkinPattern currentPattern;
    private boolean x_depressed = true;
    private boolean tapeMeasureCtrl = false;

    public void opModeMain() throws InterruptedException {
        this.initAll();

        // Fancy way of saying leds.setPattern(RedWorldsDriveProgram.defaultColor)
        // But, it will also allow it to change to blue for the BlueWorldsDriveProgram
        currentPattern = (BlinkinPattern) MiscUtils.getStaticMemberFromObject(this, "defaultColor");
        leds.setPattern(currentPattern);

        slide.autoMode();

        telemetry.addData("Initialization", "finished");
        telemetry.update();

        System.gc();
        waitForStart();

        final ElapsedTime t = new ElapsedTime();

        while (opModeIsActive() && !isStopRequested()) {
            //Declan's controls
            {
                driveTrain.gamepadControl(gamepad1, gamepad2);
                //Carousel Spinner
                spinner.gamepadControl(gamepad1, gamepad2);
            }


            //Eli's controls
            {
                if (!gamepad2.x) {
                    x_depressed = true;
                }
                if (gamepad2.x && x_depressed) {
                    tapeMeasureCtrl = !tapeMeasureCtrl;
                    turret.halt();
                    slide.halt();
                    intake.halt();
                    outtake.halt();
                    x_depressed = false;
                }

                if (tapeMeasureCtrl) {
                    turret.gamepadControl(gamepad1, gamepad2);

                } else {

                    //Handles Linear Slide Control
                    slide.gamepadControl(gamepad1, gamepad2);

                    //Intake Controls
                    BlinkinPattern proposedPattern = FreightFrenzyGameObject.getLEDColorFromItem(outtake.gamepadControl(gamepad1, gamepad2));
                    if (proposedPattern != null){
                        leds.setPattern(proposedPattern);
                    }else {
                        leds.setPattern((BlinkinPattern) MiscUtils.getStaticMemberFromObject(this, "defaultColor"));
                    }

                    intake.gamepadControl(gamepad1, gamepad2);

                    if (t.seconds() > 1) {
                        RobotLog.d("I exist and am running");
                        t.reset();
                    }

                    Thread.yield();
                }

            }

        }
    }
}


