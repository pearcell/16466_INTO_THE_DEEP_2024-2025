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
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.HorizontalExtension;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.VerticalLift;

@TeleOp
public class Auto_Test extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d beginPose = new Pose2d(15.2, 62.35, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        VerticalLift lift = new VerticalLift(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        HorizontalExtension intake = new HorizontalExtension(hardwareMap);



        //score preloaded sample
        /* TrajectoryActionBuilder scorePreload = drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(new Vector2d(56.9, 58.2), Math.toRadians(49.3)), Math.toRadians(45), new TranslationalVelConstraint(20)); */
        Action scorePreload = drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(new Vector2d(56.9, 58.2), Math.toRadians(49.3)), Math.toRadians(45), new TranslationalVelConstraint(20))
                .build();

        Action grab1 = drive.actionBuilder(new Pose2d(new Vector2d(56.9, 58.2), Math.toRadians(49.3)))
                .strafeToLinearHeading(new Vector2d(47, 51.7), Math.toRadians(-90))
                .build();
        //sample 1 grab
       /* TrajectoryActionBuilder grab1 = scorePreload.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(47, 51.7), Math.toRadians(-90), new AngularVelConstraint(Math.toRadians(90))); */


        telemetry.update();
        waitForStart();

        Actions.runBlocking(
                intake.retract()
        );

        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                claw.grab(),
                                lift.SetSlidePos(1300),
                                new SleepAction(5),
                                lift.SetSlidePos(0)
                        ),
                        lift.move()
                )


        );
    }
}
