package org.firstinspires.ftc.teamcode.OpModes.TeleOp;



import android.app.Activity;
import android.graphics.Color;
import android.util.Size;
import android.view.View;
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;



import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.List;

@TeleOp
public class FieldCentric extends LinearOpMode {
    double tgtPower = 0;

    View relativeLayout;
    DigitalChannel digitalTouch;
    double botHeading = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor frontLift = hardwareMap.dcMotor.get("frontLift");
        DcMotor backLift = hardwareMap.dcMotor.get("backLift");
        Servo servoArm = hardwareMap.servo.get("ServoArm");
        Servo servoClaw = hardwareMap.servo.get("ServoClaw");
        IMU imu = hardwareMap.get(IMU.class, "imu");

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

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
        double rest = 0.5;
        double out = 0.2;
        double armclickcount = 0;

        int upperBasket = 10000;
        int lowerBasket = 8000;
        int upperBar = 9000;
        int lowerBar = 7000;
        int scoreRange = 200;
        double distance = 2.5;

       /* ArrayList<Integer> integerList = new ArrayList<>();
        integerList.add(upperBasket);
        integerList.add(upperBar);
        integerList.add(lowerBasket);
        integerList.add(lowerBar);
        integerList.add(scoreRange);*/


        if (isStopRequested()) return;
        PredominantColorProcessor colorRangeSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorRangeSensor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "dsensor"))
                .build();
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

             /*if (gamepad1.dpad_up) {
                 frontLift.setTargetPosition();
                 frontLeftMotor.setPower(1);
                 frontLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             }  while (frontLift.isBusy()) {
             }*/
// down
            if (gamepad1.left_trigger > 0) {
                frontLift.setPower(gamepad1.left_trigger);
                backLift.setPower(-gamepad1.left_trigger);
            } else if (backLift.getCurrentPosition() == upperBasket || frontLift.getCurrentPosition() == upperBasket) {
                frontLift.setPower(0);
                backLift.setPower(0);
            } else if (backLift.getCurrentPosition() == upperBar || frontLift.getCurrentPosition() == upperBar) {
                frontLift.setPower(0);
                backLift.setPower(0);
            } else if (backLift.getCurrentPosition() == lowerBasket || frontLift.getCurrentPosition() == lowerBasket) {
                frontLift.setPower(0);
                backLift.setPower(0);
            } else if (backLift.getCurrentPosition() == lowerBar || frontLift.getCurrentPosition() == lowerBar) {
                frontLift.setPower(0);
                backLift.setPower(0);
            } else if (backLift.getCurrentPosition() == scoreRange || frontLift.getCurrentPosition() == scoreRange) {
                frontLift.setPower(0);
                backLift.setPower(0);
            }


// up
                if (gamepad1.right_trigger > 0) {
                    frontLift.setPower(-gamepad1.right_trigger);
                    backLift.setPower(gamepad1.right_trigger);
                }


                // This button choice was made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.
                if (gamepad1.start) {
                    imu.resetYaw();
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
                }
                if (currentGamepad1.x && !previousGamepad1.x) {
                    if (armclickcount % 2 == 1) {
                        servoArm.setPosition(out);
                        armclickcount = armclickcount + 1;

                    } else if (armclickcount % 2 == 0) {
                        servoArm.setPosition(rest);
                        armclickcount = armclickcount + 1;
                    }
                }

                if (dsensor.getDistance(DistanceUnit.INCH) < distance) {

                }
                telemetry.addData("DS preview on/off", "3 dots, Camera Stream\n");
                PredominantColorProcessor.Result result = colorRangeSensor.getAnalysis();
                telemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result.rgb), Color.green(result.rgb), Color.blue(result.rgb)));
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
                if (!digitalTouch.getState()) {
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
                telemetry.addData("right Trigger", gamepad1.right_trigger);
                telemetry.addData("left Trigger", gamepad1.left_trigger);

                telemetry.update();


        }

    }
}