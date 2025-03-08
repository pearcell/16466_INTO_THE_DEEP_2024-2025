package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class newClawTuningTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servoClawGrab = hardwareMap.servo.get("servoClawGrab");
        Servo servoClawRotate = hardwareMap.servo.get("servoClawRotate");
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        waitForStart();
        double rotation = .5;
        double rotateClickCount = 1;
        servoClawRotate.setPosition(rotation);
        final double open = 0;
        final double closed = 0.5;
        double clawclickcount = 0;
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
            if (currentGamepad2.b && !previousGamepad2.b) {
                if (clawclickcount % 2 == 1) {
                    clawclickcount = clawclickcount + 1;
                } else if (clawclickcount % 2 == 0) {
                    clawclickcount = clawclickcount + 1;
                }
            }
            if (clawclickcount % 2 == 1) {
                servoClawGrab.setPosition(open);

            } else if (clawclickcount % 2 == 0) {
                servoClawGrab.setPosition(closed);
            }
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper && rotation < 1) {
                if (rotateClickCount % 2 == 1) {
                   rotateClickCount = rotateClickCount + 1;
                    rotation = rotation + .1;
                } else if (clawclickcount % 2 == 0) {
                    rotateClickCount = rotateClickCount + 1;
                    rotation = rotation + .1;
                }
            }

            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper && rotation > 0) {
                if (rotateClickCount % 2 == 1) {
                    rotateClickCount = rotateClickCount - 1;
                    rotation = rotation - .1;
                } else if (clawclickcount % 2 == 0) {
                    rotateClickCount = rotateClickCount - 1;
                    rotation = rotation - .1;
                }
            }




            /*if(gamepad2.right_bumper && rotation >= 0) {
                rotation = rotation -.0025;
                servoClawRotate.setPosition(rotation);
            }
            if(gamepad2.left_bumper && rotation <= 1) {
                rotation = rotation +.0025;
                servoClawRotate.setPosition(rotation);
            }*/
            telemetry.addData("Claw Grab Position", servoClawGrab.getPosition());
            telemetry.addData("Claw Rotate Position", servoClawRotate.getPosition());
            telemetry.addData("rotateClickCount", rotateClickCount);
            telemetry.addData("rotation", rotation);
            telemetry.update();
        }
    }
}