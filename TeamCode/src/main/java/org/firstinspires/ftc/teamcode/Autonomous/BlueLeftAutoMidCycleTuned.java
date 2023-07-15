package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Components.Lift.LiftConstants.LIFT_MED_JUNCTION;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;


@Config
@Autonomous(name = "BlueLeftAutoMidCycleTuned")
//temp auto logs
//13.24 E NOBOOST NOROTATED

public class BlueLeftAutoMidCycleTuned extends LinearOpMode {

    public static double dummyP = 3;

    public static double dropX = 28.5, dropY = 21, dropA = toRadians(320), dropET = toRadians(150);

//    public static double pickupX1 = -46, pickupY1 = 10, pickupA1 = toRadians(180), pickupET1 = toRadians(180);
public static double pickupX2 = 63, pickupY2 = 11.5, pickupA2 = toRadians(0), pickupET2 = toRadians(0);

    double[] stackPos = {440, 330, 245, 100, 0};

    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this,false);
        BlueLeftMid autoRunner = new BlueLeftMid(false,this, robot);
        sleep(500);
        autoRunner.init();
        abort:
        while ((getRuntime() < 27 && (!robot.queuer.isFullfilled() || robot.queuer.isFirstLoop()))&&!isStopRequested()) {
            autoRunner.preload();
            for (int i = 0; i < 5; i++) {
                if (!autoRunner.pick(i)) {
                    break abort;
                }
//                if(i==3){
//                    robot.roadrun.update();
//                }
                autoRunner.drop(i);
            }
            autoRunner.update();
        }
        robot.done();
        robot.queuer.reset();
        robot.done();
        while ((getRuntime() < 29.8 && (!robot.queuer.isFullfilled() || robot.queuer.isFirstLoop()))&&!isStopRequested()) {
            autoRunner.park();
            autoRunner.update();
        }
        stop();
    }
}