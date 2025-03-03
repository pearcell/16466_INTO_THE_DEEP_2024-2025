package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Tests;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.FourBarClaw;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.HorizontalExtension;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.Swivel;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.VerticalLift;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.FourBarClaw;

@TeleOp
public class Auto_Test extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d beginPose = new Pose2d(15.2, 62.35, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        VerticalLift lift = new VerticalLift(hardwareMap);
        FourBarClaw claw = new FourBarClaw(hardwareMap);
        Swivel swivel = new Swivel(hardwareMap);
        HorizontalExtension intake = new HorizontalExtension(hardwareMap);

        //initialize trajectories
        //score preloaded sample
        TrajectoryActionBuilder scorePreload = drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(new Vector2d(57.9, 59.2), Math.toRadians(49.3)), Math.toRadians(45), new TranslationalVelConstraint(20));

        //sample 1 grab
        TrajectoryActionBuilder grab1 = scorePreload.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(48.5, 52.7), Math.toRadians(-90), new AngularVelConstraint(Math.toRadians(90)));

        //prebuild
        Action scorePreloadFinal = scorePreload.build();
        Action grab1Final = grab1.build();




        waitForStart();

        Actions.runBlocking(
                intake.retract()
        );

        Actions.runBlocking(swivel.turn(.5));

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                    scorePreloadFinal,
                    grab1Final
                )
        );
    }
}
