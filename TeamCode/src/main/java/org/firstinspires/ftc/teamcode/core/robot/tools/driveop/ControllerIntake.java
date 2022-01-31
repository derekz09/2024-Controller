package org.firstinspires.ftc.teamcode.core.robot.tools.driveop;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoIntake;
import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoLift;
import org.firstinspires.ftc.teamcode.opmodes.util.MyToggleButtonReader;

/**
 * intake, not an extension of ControllerToggleableTool
 */
public class ControllerIntake extends AutoIntake {
    private final GamepadEx toolGamepad;
    private final MyToggleButtonReader reader;

    public ControllerIntake(@NonNull HardwareMap map, GamepadEx toolGamepad) {
        super(map);
        this.reader = new MyToggleButtonReader(toolGamepad, GamepadKeys.Button.X);
        this.toolGamepad = toolGamepad;
    }

    public void update(AutoLift.Positions liftPosition) {
        reader.readValue();
        if (noObject() && liftPosition == AutoLift.Positions.INTAKING) {
            if (reader.getState()) {
                if (toolGamepad.getButton(GamepadKeys.Button.Y)) {
                    this.forward();
                } else {
                    this.backward();
                }
            } else {
                this.stop();
            }
        } else {
            if (toolGamepad.getButton(GamepadKeys.Button.Y)) {
                this.forward();
            } else if (toolGamepad.getButton(GamepadKeys.Button.A)) {
                this.backward();
            } else {
                this.stop();
            }
            reader.currToggleState = false;
        }
    }
}
