package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class NewClawTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo servoClaw = hardwareMap.servo.get("claw");

        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        double closed = .5;
        double open = 0;
        double clawclickcount = 0;
        double test1 = .3;
        double test2 = .2;

        double swivelPosition = 0;

        waitForStart();

        while (opModeIsActive()) {

            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            if (gamepad2.b) {
                // opens claw
                servoClaw.setPosition(test1);
            }

            if (gamepad2.x) {
                //closes claw
                servoClaw.setPosition(test2);
            }

            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper && swivelPosition <= 1) {
                swivelPosition += .1;
            } else if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper && swivelPosition >= 0) {
                swivelPosition -= .1;
            }



            telemetry.addData("swivel Pos", swivelPosition);

            telemetry.addData("Claw Position", servoClaw.getPosition());
            telemetry.update();
        }
    }
}
