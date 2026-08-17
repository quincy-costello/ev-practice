package org.firstinspires.ftc.teamcode.commandbasetraining;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.firstinspires.ftc.teamcode.commandbasetraining.subsystems.drivetrain.DriveTrain;

@TeleOp
public class TheRobotGoesVroom extends LinearOpMode {

    @Override
    public void runOpMode() {
        DriveTrain dt = new DriveTrain(hardwareMap);

        DcMotor intakeMotor = hardwareMap.dcMotor.get("intake_motor");

        dt.setDefaultCommand(
                new RunCommand(
                        () -> {
                            dt.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                        },
                        dt
                )
        );

        waitForStart();

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            intakeMotor.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

            if (gamepad1.aWasPressed()) {
                dt.resetYaw();
            }
        }
    }

}
