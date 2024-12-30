package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@TeleOp
public class FieldCentric extends LinearOpMode {
    double tgtPower = 0;
    DigitalChannel digitalTouch;
    double botHeading = 0;
    DistanceSensor dsensor;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();


        DcMotor frontLift = hardwareMap.dcMotor.get("frontLift");
        DcMotor backLift = hardwareMap.dcMotor.get("backLift");
        Servo servoArm = hardwareMap.servo.get("ServoArm");
        Servo servoClaw = hardwareMap.servo.get("ServoClaw");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        // Declare our motors
        // Make sure your ID's match your configuration

        ColorRangeSensor dsensor = hardwareMap.get(ColorRangeSensor.class, "dsensor");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");

        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        telemetry.addData("DigitalTouchSensorExample", "Press start to continue...");


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        frontLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
        backLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        backLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
        double value = dsensor.getDistance(DistanceUnit.INCH);

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();

        waitForStart();
        double closed = 0.15;
        double open = 0;
        double clawclickcount = 0;
        if (value > 5 && value <= 10) {
        }
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (imu.getRobotYawPitchRollAngles().getAcquisitionTime() == 0) {
                imu.initialize(parameters);
            }

            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            tgtPower = -this.gamepad1.left_stick_y;

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            int position = backLift.getCurrentPosition();
            double revolutions = position;

            double angle = revolutions * 360;
            double angleNormalized = angle % 360;

            if (gamepad1.a) {
                frontLift.setPower(-0.5);
                backLift.setPower(0.5);
            } else if (backLift.getCurrentPosition() == 10000) {
                frontLift.setPower(0);
                backLift.setPower(0);
            } else if (frontLift.getCurrentPosition() == 10000) {
                frontLift.setPower(0);
                backLift.setPower(0);
            }

            if (gamepad1.left_bumper) {
                frontLift.setPower(-0.5);
                backLift.setPower(0.5);
            } else if (backLift.getCurrentPosition() == 900) {
                frontLift.setPower(0);
                backLift.setPower(0);
            } else if (frontLift.getCurrentPosition() == 900) {
                frontLift.setPower(0);
                backLift.setPower(0);
            }


            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.start) {
                imu.resetYaw();
            }


            if (gamepad1.dpad_up) {
                // move to 0 degrees.
                servoArm.setPosition(0);
            } else if (gamepad1.dpad_down) {
                // move to 90 degrees.
                servoArm.setPosition(0.5);
            }
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
            if (currentGamepad1.b && !previousGamepad1.b) {
                if (clawclickcount % 2 == 1) {
                    servoClaw.setPosition(open);
                    clawclickcount = clawclickcount + 1;

                } else if (clawclickcount % 2 == 0) {
                    servoClaw.setPosition(closed);
                    clawclickcount = clawclickcount + 1;
                }


                if (gamepad1.right_bumper) {
                    frontLift.setPower(-1);
                    backLift.setPower(1);
                } else {
                    frontLift.setPower(0);
                    backLift.setPower(0);
                }

                telemetry.addData("yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
                telemetry.addData("Encoder Position", position);
                telemetry.addData("Encoder Revolutions", revolutions);
                telemetry.addData("Encoder Angle (Degrees)", angle);
                telemetry.addData("Encoder Angle - Normalized (Degrees)", angleNormalized);
                telemetry.addData("Distance Sensor", dsensor.getDistance(DistanceUnit.INCH));
                telemetry.addData("front Left Motor", frontLeftMotor.getPower());
                telemetry.addData("front Right Motor", frontRightMotor.getPower());
                telemetry.addData("back Left Motor", backLeftMotor.getPower());
                telemetry.addData("back Right Motor", backRightMotor.getPower());
                if (digitalTouch.getState() == false) {
                    telemetry.addData("Button", "PRESSED");
                } else {
                    telemetry.addData("Button", "NOT PRESSED");
                }
                if (servoClaw.getPosition() == 0) {
                    telemetry.addData("Claw", "Open");
                } else {
                    telemetry.addData("Claw", "closed");
                }
                telemetry.addData("Claw Position", servoClaw.getPosition());
                telemetry.addData("Arm Position", servoArm.getPosition());
                telemetry.addData("Claw Position", servoClaw.getPosition());
                telemetry.addData("Claw Click Count", clawclickcount);

                telemetry.update();
            }
        }
    }

}