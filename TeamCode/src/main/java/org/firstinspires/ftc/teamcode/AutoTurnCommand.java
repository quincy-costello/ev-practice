package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.CommandBase;

public class AutoTurnCommand extends CommandBase {
    private MecanumDriveSubsystem subsystem;

    public AutoTurnCommand(MecanumDriveSubsystem subsystem) {
        this.subsystem = subsystem;
    }


}
