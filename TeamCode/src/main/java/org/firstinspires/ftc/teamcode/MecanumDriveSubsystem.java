package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class MecanumDriveSubsystem extends SubsystemBase {
    // Declaring runtime variable
    private ElapsedTime runtime = new ElapsedTime();

    // Declaring motor variables
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    // IMU
    private IMU imu;

    private double ticksPerRevolution = 360.0; // each tick is one degree
    private double wheelCircumfrence = 3.77953 * Math.PI; // stored in inches. wheel diameter is 9.6 cm

    public MecanumDriveSubsystem(final HardwareMap hardwareMap) {
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