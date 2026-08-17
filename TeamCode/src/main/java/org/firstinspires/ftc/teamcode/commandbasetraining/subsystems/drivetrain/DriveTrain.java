package org.firstinspires.ftc.teamcode.commandbasetraining.subsystems.drivetrain;

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

public class DriveTrain extends SubsystemBase {

    private final DcMotor frontLeftDrive;
    private final DcMotor backLeftDrive;
    private final DcMotor frontRightDrive;
    private final DcMotor backRightDrive;

    private final IMU imu;

    private final double ticksPerRevolution = 360.0; // each tick is one degree
    private final double wheelCircumfrence = 3.77953 * Math.PI; // stored in inches. wheel diameter is 9.6 cm

    private double prevFrontLeftPos = 0.0;
    private double prevBackLeftPos = 0.0;
    private double prevFrontRightPos = 0.0;
    private double prevBackRightPos = 0.0;

    private double x = 0.0;
    private double y = 0.0;
    private double yaw = 0.0;

    public DriveTrain(HardwareMap hardwareMap) {
        frontLeftDrive = hardwareMap.dcMotor.get("front_left_drive");
        backLeftDrive = hardwareMap.dcMotor.get("back_left_drive");
        frontRightDrive = hardwareMap.dcMotor.get("front_right_drive");
        backRightDrive = hardwareMap.dcMotor.get("back_right_drive");

        // Setting directions of drive motors
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Resetting encoders
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    public void periodic() {
        double frontLeftTicks = frontLeftDrive.getCurrentPosition();
        double backLeftTicks = backLeftDrive.getCurrentPosition();
        double frontRightTicks = frontRightDrive.getCurrentPosition();
        double backRightTicks = backRightDrive.getCurrentPosition();

        double frontLeftRevolutions = frontLeftTicks/ticksPerRevolution;
        double backLeftRevolutions = backLeftTicks/ticksPerRevolution;
        double frontRightRevolutions = frontRightTicks/ticksPerRevolution;
        double backRightRevolutions = backRightTicks/ticksPerRevolution;

        double frontLeftPos = frontLeftRevolutions * wheelCircumfrence;
        double backLeftPos = backLeftRevolutions * wheelCircumfrence;
        double frontRightPos = frontRightRevolutions * wheelCircumfrence;
        double backRightPos = backRightRevolutions * wheelCircumfrence;

        double frontLeftDisplacement = frontLeftPos - prevFrontLeftPos;
        double backLeftDisplacement = backLeftPos - prevBackLeftPos;
        double frontRightDisplacement = frontRightPos - prevFrontRightPos;
        double backRightDisplacement = backRightPos - prevBackRightPos;

        double dyLocal = (frontLeftDisplacement + backLeftDisplacement + frontRightDisplacement + backRightDisplacement)/4;
        double dxLocal = (frontLeftDisplacement - backLeftDisplacement - frontRightDisplacement + backRightDisplacement)/4;

        yaw = imu.getRobotYawPitchRollAngles().getYaw() * (Math.PI/180.0);

        x += (dxLocal * Math.cos(yaw)) - (dyLocal * Math.sin(yaw));
        y += (dxLocal * Math.sin(yaw)) + (dyLocal * Math.cos(yaw));

        prevFrontLeftPos = frontLeftPos;
        prevBackLeftPos = backLeftPos;
        prevFrontRightPos = frontRightPos;
        prevBackRightPos = backRightPos;
    }

    public void translate(double dir) {
        double y = Math.sin(dir);
        double x = Math.cos(dir);

        double denominator = Math.max(Math.abs(y) + Math.abs(x), 1) * 6 ;

        double frontLeftPower = (y + x) / denominator;
        double backLeftPower = (y - x) / denominator;
        double frontRightPower = (y - x) / denominator;
        double backRightPower = (y + x) / denominator;

        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);
    }
    public void translate(double y, double x) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x), 1) * 3;

        double frontLeftPower = (y + x) / denominator;
        double backLeftPower = (y - x) / denominator;
        double frontRightPower = (y - x) / denominator;
        double backRightPower = (y + x) / denominator;

        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);
    }

    public void drive(double y, double x, double rot) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rot), 1);

        double frontLeftPower = (y + x + rot) / denominator;
        double backLeftPower = (y - x + rot) / denominator;
        double frontRightPower = (y - x - rot) / denominator;
        double backRightPower = (y + x - rot) / denominator;

        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public void stopMotors() {
        frontLeftDrive.setPower(0.0);
        backLeftDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
        backRightDrive.setPower(0.0);
    }

    public double getY() { return y; }
    public double getX() { return x; }
    public double getYaw() { return yaw; }
}
