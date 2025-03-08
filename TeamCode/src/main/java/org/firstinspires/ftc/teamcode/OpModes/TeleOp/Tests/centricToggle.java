package org.firstinspires.ftc.teamcode.OpModes.TeleOp.Tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@TeleOp
public class centricToggle extends LinearOpMode {
    double tgtPower = 0;
    double botHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);



        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);
        imu.resetYaw();
        waitForStart();
        if (isStopRequested()) return;

        double centricClickCount = 0;
        double frontLeftPower = 0;
        double backLeftPower = 0;
        double frontRightPower = 0;
        double backRightPower = 0;

        while (opModeIsActive()) {

            if (imu.getRobotYawPitchRollAngles().getAcquisitionTime() == 0) {
                imu.initialize(parameters);
            }

            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            tgtPower = -this.gamepad1.left_stick_y;
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1;  // Counteract imperfect strafing

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            if (currentGamepad1.y && !previousGamepad1.y) {
                if (centricClickCount % 2 == 1) {
                    centricClickCount = centricClickCount + 1;
                }else if (centricClickCount % 2 == 0) {
                    centricClickCount = centricClickCount + 1;
                }
            }

                // field Centric.
                if (centricClickCount % 2 == 1) {
                    frontLeftPower = (rotY + rotX + rx) / denominator;
                    backLeftPower = (rotY - rotX + rx) / denominator;
                    frontRightPower = (rotY - rotX - rx) / denominator;
                    backRightPower = (rotY + rotX - rx) / denominator;
                    // robot Centric.
                }  else if (centricClickCount % 2 == 0) {
                    frontLeftPower = (y + x + rx) / denominator;
                    backLeftPower = (y - x + rx) / denominator;
                    frontRightPower = (y - x - rx) / denominator;
                    backRightPower = (y + x - rx) / denominator;
                }

            else if (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0 && gamepad1.right_stick_x == 0) {
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);

            }

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addData("frontLeftPower", frontLeftMotor.getPower());
            telemetry.addData("backLeftPower", backLeftMotor.getPower());
            telemetry.addData("frontRightPower", frontRightMotor.getPower());
            telemetry.addData("backRightPower", backRightMotor.getPower());
            telemetry.addData("frontLeftVariable", frontLeftPower);
            telemetry.addData("backLeftVariable", backLeftPower);
            telemetry.addData("frontRightVariable", frontRightPower);
            telemetry.addData("backRightVariable", backRightPower);
            if (centricClickCount % 2 == 0) {
                telemetry.addData("Centric", "robot");
            } else {
                telemetry.addData("Centric", "field");
            }
            telemetry.addData("botHeading",   botHeading);
            telemetry.update();

        }
    }
}
