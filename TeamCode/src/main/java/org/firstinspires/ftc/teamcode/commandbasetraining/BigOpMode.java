package org.firstinspires.ftc.teamcode.commandbasetraining;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.AutoTurnCommand;
import org.firstinspires.ftc.teamcode.commandbasetraining.subsystems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.commandbasetraining.subsystems.drivetrain.commands.DriveForward;

@TeleOp(name="BIg", group="Linear OpMode")
public class BigOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {

        DriveTrain driveTrain = new DriveTrain(hardwareMap);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                    new DriveForward(driveTrain, 0.5 * Math.PI, 18), // drive forward 18 inches
                    new DriveForward(driveTrain, -0.5 * Math.PI, 18) // drive backward 18 inches
                )
        );

        waitForStart();

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            // updates and telemtry
        }
    }
}
