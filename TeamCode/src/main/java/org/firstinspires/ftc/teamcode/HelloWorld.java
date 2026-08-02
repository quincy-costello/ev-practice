package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
public class HelloWorld extends LinearOpMode {
    // change

    // Declaring runtime variable
    private ElapsedTime runtime = new ElapsedTime();

    // Declaring motor variables
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    // IMU
    private IMU imu;

    //private double frontLeftPower;
    //private double backLeftPower;
    //private double frontRightPower;
    //private double backRightPower;

    private double ticksPerRevolution = 360.0; // each tick is one degree
    private double wheelCircumfrence = 3.77953 * Math.PI; // stored in inches. wheel diameter is 9.6 cm

    @Override
    public void runOpMode() {
        // Initializing motor variables
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

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        // Starting
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        double prevFrontLeftPos = 0.0;
        double prevBackLeftPos = 0.0;
        double prevFrontRightPos = 0.0;
        double prevBackRightPos = 0.0;

        double x = 0.0;
        double y = 0.0;
        while (opModeIsActive()) {

            // ENCODER INTRO

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

            double yaw = imu.getRobotYawPitchRollAngles().getYaw() * (Math.PI/180.0);

            x += (dxLocal * Math.cos(yaw)) - (dyLocal * Math.sin(yaw));
            y += (dxLocal * Math.sin(yaw)) + (dyLocal * Math.cos(yaw));

            telemetry.addData("x", x);
            telemetry.addData("y", y);

            telemetry.addData("yaw", yaw);

            if (y < 18) {
                translate(0.5 * Math.PI);
            }
            else {
                stopMotors();
            }

            telemetry.update();

            prevFrontLeftPos = frontLeftPos;
            prevBackLeftPos = backLeftPos;
            prevFrontRightPos = frontRightPos;
            prevBackRightPos = backRightPos;

            /*
            // Translate in a square:
            telemetry.addData("time elapsed", runtime.seconds());
            telemetry.addData("(int) time elapsed", (int) runtime.seconds());
            telemetry.addData("(int) time elapsed % 4", ((int) runtime.seconds()) % 4);

            translate(((((int)runtime.seconds()) % 4) * (pi/2)) + (pi/2));
            */

            /*
            // REMOTE CONTROL W JOYSTICKS:

            double y = -gamepad1.left_stick_y; // y stick is reversed
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1) * 3;

            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftDrive.setPower(frontLeftPower);
            backLeftDrive.setPower(backLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backRightDrive.setPower(backRightPower);

            telemetry.update();
            */

            /*
            // INDIVIDUAL MOTOR DEBUG TEST:
            if (gamepad1.x) {
                frontLeftDrive.setPower(power);
            }
            else {
                frontLeftDrive.setPower(0.0);
            }

            if (gamepad1.a) {
                backLeftDrive.setPower(power);
            }
            else {
                backLeftDrive.setPower(0.0);
            }

            if (gamepad1.y) {
                frontRightDrive.setPower(power);
            }
            else {
                frontRightDrive.setPower(0.0);
            }

            if (gamepad1.b) {
                backRightDrive.setPower(power);
            }
            else {
                backRightDrive.setPower(0.0);
            }
            */

        }
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

    public void stopMotors() {
        frontLeftDrive.setPower(0.0);
        backLeftDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
        backRightDrive.setPower(0.0);
    }
}