package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import android.view.View;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp
public class FieldCentric extends LinearOpMode {
    View relativeLayout;
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
        Servo servoClawGrab = hardwareMap.servo.get("ServoClawGrab");
        Servo servoClawRotate = hardwareMap.servo.get("ServoClawRotate");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        // Increases the speed of gathering sensor data
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        // Declare our motors
        // Make sure your ID's match your configuration

        /* ColorRangeSensor dsensor = hardwareMap.get(ColorRangeSensor.class, "dsensor");*/
        /*digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");

        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        telemetry.addData("DigitalTouchSensorExample", "Press start to continue...");*/


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLift.setDirection(DcMotorSimple.Direction.REVERSE);
        backLift.setDirection(DcMotorSimple.Direction.FORWARD);
        /*frontLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/ // Reset the motor encoder
        frontLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
        /*backLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/ // Reset the motor encoder
        backLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
        backLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();
        waitForStart();

        double botAdjustment = 45;
        double rotation = .5;
        double rotateClickCount = 1;
        final double closed = .5;
        final double open = 0;
        double clawclickcount = 0;
        final double rest = 0.73;
        final double out = 0.4;
        double armclickcount = 0;
        final int upperBasket = 1295;
        final int upperSpecimenBar = 540;
        final int robotSlowDown = 600;
        final int specimenScoringMacro = 775;
        int scoreMacroLock = 0;
        final int wall = 108;
        final int liftSlowDown = 500;
        final int armLockOut = 300;
        double driveTrainSpeed = 1;
        double driveTrainClickCount = 0;
        double centricClickCount = 0;
        double frontLeftPower = 0;
        double backLeftPower = 0;
        double frontRightPower = 0;
        double backRightPower = 0;

        if (isStopRequested()) return;
        servoClawRotate.setPosition(.5);
        servoArm.setPosition(.7);
       /* PredominantColorProcessor colorRangeSensor = new PredominantColorProcessor.Builder()
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
                .build();*/
         while (opModeIsActive()) {

            if (imu.getRobotYawPitchRollAngles().getAcquisitionTime() == 0) {
                imu.initialize(parameters);
            }

            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - botAdjustment;

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (currentGamepad1.a && !previousGamepad1.a) {
                if (driveTrainClickCount % 2 == 1 ) {
                    driveTrainClickCount = driveTrainClickCount + 1;

                }else if (driveTrainClickCount % 2 == 0) {
                    driveTrainClickCount = driveTrainClickCount + 1;
                }
            }
            if (driveTrainClickCount % 2 == 1 || servoArm.getPosition() == out) {
                driveTrainSpeed = .4;
            } else if (driveTrainClickCount % 2 == 0) {
                driveTrainSpeed = 1;
            }
            /* if(servoArm.getPosition() == out) {
                 driveTrainClickCount = 1;
             }*/



            if (currentGamepad1.y && !previousGamepad1.y) {
                // field Centric.
                if (centricClickCount % 2 == 1) {
                    centricClickCount = centricClickCount + 1;
                    // robot Centric.
                } else if (centricClickCount % 2 == 0) {
                    centricClickCount = centricClickCount + 1;
                }
            }
            // robot Centric.
            if (centricClickCount % 2 == 1) {
                frontLeftPower = (y + x + rx) / denominator;
                backLeftPower = (y - x + rx) / denominator;
                frontRightPower = (y - x - rx) / denominator;
                backRightPower = (y + x - rx) / denominator;
                // field Centric
            } else if (centricClickCount % 2 == 0) {
                frontLeftPower = (rotY + rotX + rx) / denominator;
                backLeftPower = (rotY - rotX + rx) / denominator;
                frontRightPower = (rotY - rotX - rx) / denominator;
                backRightPower = (rotY + rotX - rx) / denominator;
            }

            frontLeftMotor.setPower(frontLeftPower * driveTrainSpeed);
            backLeftMotor.setPower(backLeftPower * driveTrainSpeed);
            frontRightMotor.setPower(frontRightPower * driveTrainSpeed);
            backRightMotor.setPower(backRightPower * driveTrainSpeed);

            //down
            if (gamepad2.left_trigger > 0 && frontLift.getCurrentPosition() >= liftSlowDown && gamepad2.right_trigger == 0) {
                frontLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
                backLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
                frontLift.setPower(-gamepad2.left_trigger);
                backLift.setPower(-gamepad2.left_trigger);
            }

            if (gamepad2.left_trigger > 0 && frontLift.getCurrentPosition() < 5) {
                frontLift.setPower(0);
                backLift.setPower(0);
            }

            if (gamepad2.left_trigger > 0 && frontLift.getCurrentPosition() < liftSlowDown && frontLift.getCurrentPosition() > 5 && gamepad2.right_trigger == 0) {
                frontLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
                backLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
                frontLift.setPower(.5 * -gamepad2.left_trigger);
                backLift.setPower(.5 * -gamepad2.left_trigger);
            }

            // up
            if (gamepad2.right_trigger > 0 && Math.abs(frontLift.getCurrentPosition()) < upperBasket && gamepad2.left_trigger == 0) {
                scoreMacroLock = 1;
                frontLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
                backLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
                frontLift.setPower(gamepad2.right_trigger);
                backLift.setPower(gamepad2.right_trigger);
            }

            if (frontLift.getCurrentPosition() >= upperBasket && gamepad2.right_trigger > 0) {
                frontLift.setPower(0);
                backLift.setPower(0);
            }

            if (gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0 && frontLift.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER && !gamepad2.dpad_right) {
                frontLift.setPower(0);
                backLift.setPower(0);
            }

            if (gamepad2.right_trigger > 0 && gamepad2.left_trigger > 0) {
                frontLift.setPower(0);
                backLift.setPower(0);
            }
            // Dpad Up takes the lift to upper basket
            if (gamepad2.dpad_up && gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0 && servoArm.getPosition() == rest) {
                scoreMacroLock = 0;
                frontLift.setTargetPosition(upperBasket);
                backLift.setTargetPosition(upperBasket);
                frontLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLift.setPower(1);
                backLift.setPower(1);
            }

            // Dpad Right takes lift to the upper specimen bar
            if (gamepad2.dpad_left && gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0 && servoArm.getPosition() == rest) {
                frontLift.setTargetPosition(upperSpecimenBar);
                backLift.setTargetPosition(upperSpecimenBar);
                frontLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLift.setPower(1);
                backLift.setPower(1);
                scoreMacroLock = 1;
            }

            // Dpad Down takes the lift to the human player wall
            if (gamepad2.dpad_down && gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0) {
                scoreMacroLock = 0;
                frontLift.setTargetPosition(wall);
                backLift.setTargetPosition(wall);
                frontLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLift.setPower(1);
                backLift.setPower(1);
                armclickcount = 0;
            }


            if (gamepad2.dpad_right && frontLift.getCurrentPosition() <= specimenScoringMacro) {
                frontLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontLift.setPower(1);
                backLift.setPower(1);
            } else if(gamepad2.dpad_right && frontLift.getCurrentPosition() > specimenScoringMacro) {
                frontLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontLift.setPower(0);
                 backLift.setPower(0);
            }


            if (Math.abs(frontLift.getCurrentPosition()) > robotSlowDown && driveTrainClickCount % 2 == 0) {
                driveTrainClickCount = driveTrainClickCount + 1;
            } else if(frontLift.getMode() == DcMotor.RunMode.RUN_TO_POSITION && frontLift.getCurrentPosition() < liftSlowDown) {
                driveTrainClickCount = 0;
            }
            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.start) {
                botAdjustment = 0;
                imu.resetYaw();
            }
            if (currentGamepad2.b && !previousGamepad2.b) {
                if (clawclickcount % 2 == 1) {
                    clawclickcount = clawclickcount + 1;
                } else if (clawclickcount % 2 == 0) {
                    clawclickcount = clawclickcount + 1;
                }
            }
            if (clawclickcount % 2 == 1) {
                servoClawGrab.setPosition(closed);
            } else if (clawclickcount % 2 == 0) {
                servoClawGrab.setPosition(open);
            }

            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper && rotation > .2) {
                    rotation = rotation - .1;
            }


            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper && rotation < .8) {
                    rotation = rotation + .1;
            }

            if (servoArm.getPosition() == rest){
                rotation = .5;
            }

            servoClawRotate.setPosition(rotation);

            if (frontLift.getCurrentPosition() < 600 && frontLift.getCurrentPosition() > 500){
                driveTrainClickCount = 0;
            }

            /*if(gamepad2.y) {
                backLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }*/

            if (currentGamepad2.x && !previousGamepad2.x) {
                if (armclickcount % 2 == 1 && Math.abs(frontLift.getCurrentPosition()) < armLockOut) {
                    armclickcount = armclickcount + 1;
                } else if (armclickcount % 2 == 0) {
                    armclickcount = armclickcount + 1;
                }
            }

            if (armclickcount % 2 == 1 && frontLift.getCurrentPosition() < armLockOut) {
                servoArm.setPosition(out);
            }else if (armclickcount % 2 == 0) {
                servoArm.setPosition(rest);
            }

            /*if (dsensor.getDistance(DistanceUnit.INCH) < distance) {

            }
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream\n");
            PredominantColorProcessor.Result result = colorRangeSensor.getAnalysis();
            telemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result.rgb), Color.green(result.rgb), Color.blue(result.rgb)));*/
            telemetry.addData("yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            /*telemetry.addData("Distance Sensor", dsensor.getDistance(DistanceUnit.INCH));*/
            if (centricClickCount % 2 == 0) {
                telemetry.addData("Centric", "field");
            } else {
                telemetry.addData("Centric", "robot");
            }
            if (servoClawGrab.getPosition() == open) {
                telemetry.addData("Claw", "closed");
            } else {
                telemetry.addData("Claw", "open");
            }
            telemetry.addData("back Lift Pos", backLift.getCurrentPosition());
            telemetry.addData("front Lift Pos", frontLift.getCurrentPosition());
            telemetry.addData("Drive Train Speed", driveTrainSpeed);
            telemetry.addData("Claw Click Count", clawclickcount);
            telemetry.addData("arm Click Count", armclickcount);
            telemetry.addData("right Trigger", gamepad2.right_trigger);
            telemetry.addData("left Trigger", gamepad2.left_trigger);
            telemetry.addData("Left Odometry", frontRightMotor.getCurrentPosition());
            telemetry.addData("Right Odometry", frontLeftMotor.getCurrentPosition());
            telemetry.addData("Back Odometry", backRightMotor.getCurrentPosition());
            telemetry.addData("Claw Position", servoClawGrab.getPosition());
            telemetry.addData("Arm Position", servoArm.getPosition());
            telemetry.addData("Drive Train Click Count", driveTrainClickCount);
            telemetry.addData("Centric Click Count", centricClickCount);
            telemetry.addData("back Lift", backLift.getPower());
            telemetry.addData("front Lift", frontLift.getPower());
            if (frontLift.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                telemetry.addData("RunMode", "RUN_WITHOUT_ENCODER");
            } else {
                telemetry.addData("RunMode", "RUN_TO_POSITION");
            }
            telemetry.addData("front Left Motor", frontLeftMotor.getPower());
            telemetry.addData("front Right Motor", frontRightMotor.getPower());
            telemetry.addData("back Left Motor", backLeftMotor.getPower());
            telemetry.addData("back Right Motor", backRightMotor.getPower());
            telemetry.addData("scoreMacroLock", scoreMacroLock);
            telemetry.addData("Claw Rotate Position", servoClawRotate.getPosition());
            telemetry.addData("rotateClickCount", rotateClickCount);
            telemetry.addData("rotation", rotation);
            if (servoArm.getPosition() == .7) {
                telemetry.addData("arm", "Rest");
            } else if (servoArm.getPosition() == .4) {
                telemetry.addData("arm", "Out");
            }
            telemetry.addData("imu", botHeading);
            telemetry.update();
        }
    }
}




