package org.firstinspires.ftc.teamcode.Extra;//package org.firstinspires.ftc.teamcode.Extra;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.arcrobotics.ftclib.command.CommandBase;
//
//public class CommandAction implements Action {
//    private CommandBase command;
//    private boolean initialized = false;
//    private boolean finished = false;
//
//    public CommandAction(CommandBase command){
//        this.command = command;
//    }
//
//    @Override
//    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//        if(!initialized) {
//            command.initialize();
//            initialized = true;
//        } else if (command.isFinished()) {
//            command.end(false);
//            finished = true;
//        } else {
//            command.execute();
//        }
//        return ! finished;
//    }
//}
