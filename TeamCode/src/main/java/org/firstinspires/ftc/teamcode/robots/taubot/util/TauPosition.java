package org.firstinspires.ftc.teamcode.robots.taubot.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.io.Serializable;
import java.time.LocalTime;

public class TauPosition implements Serializable {
    private static final long serialVersionUID = 1234L;
    private double chassisX;
    private double chassisY;
    private double chassisHeading;
    private double turretHeading;
    private long timestamp;
    public TauPosition() {
        chassisX = 0;
        chassisY = 0;
        chassisHeading = 0;
        turretHeading=0;
        timestamp = System.currentTimeMillis();
    }

    public TauPosition(Pose2d chassisPose, double turretHeading) {
        this.chassisX = chassisPose.getX();
        this.chassisY = chassisPose.getY();
        this.chassisHeading = chassisPose.getHeading();
        this.turretHeading= turretHeading;
        timestamp = System.currentTimeMillis();
    }


    public void setPose(Pose2d pose){
        this.chassisX = pose.getX();
        this.chassisY = pose.getY();
        this.chassisHeading = pose.getHeading();
    }
    public void setTurretHeading(double heading) {
        turretHeading = heading;
    }
    public void updateTime() { timestamp = System.currentTimeMillis(); }
    public Pose2d getPose(){
        return new Pose2d(chassisX, chassisY, chassisHeading);
    }
    public double getTurretHeading() { return turretHeading; }
    public long getTimestamp() { return timestamp; }
}