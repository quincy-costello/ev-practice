package org.firstinspires.ftc.teamcode.commandbasetraining.subsystems.drivetrain.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.commandbasetraining.subsystems.drivetrain.DriveTrain;

public class DriveForward extends CommandBase {

    private final DriveTrain driveTrain; // subsystem

    private final double targetY;
    private final double dirRad;

    public DriveForward(DriveTrain driveTrain, double dirRad, double targetY) {
        this.driveTrain = driveTrain;
        this.targetY = targetY;

        this.dirRad = dirRad;

        addRequirements(driveTrain);
    }

    public void execute() {
        driveTrain.translate(dirRad);
    }

    public boolean isFinished() {
        return targetY >= driveTrain.getY();
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stopMotors();
    }
}