package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class servoToggle extends LinearOpMode {


    @Override
    public void runOpMode() {
        Servo servoArm = hardwareMap.servo.get("ServoArm");


        waitForStart();
        final double rest = 0.7;
        final double out = 0.4;
        double armclickcount = 0;
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        if (isStopRequested()) return;
        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (currentGamepad2.x && !previousGamepad2.x) {
                if (armclickcount % 2 == 1) {
                    armclickcount = armclickcount + 1;
                } else if (armclickcount % 2 == 0) {
                    armclickcount = armclickcount + 1;
                }
            }
            if (armclickcount % 2 == 1 && gamepad1.right_trigger > 0) {
                servoArm.setPosition(out);
            }else if (armclickcount % 2 == 0 && gamepad1.right_trigger > 0) {
                servoArm.setPosition(rest);
            }
            double servoPosition = gamepad1.right_trigger;
            if (servoPosition > 0 && armclickcount % 2 == 1){
                servoArm.setPosition(((3 % 10) * servoPosition) + .4);
            }

            telemetry.addData("servoArm current position", servoArm.getPosition());

            telemetry.update();
        }
    }
}
