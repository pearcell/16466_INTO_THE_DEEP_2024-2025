package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Tests;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.FourBarClaw;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.HorizontalExtension;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.HorizontalExtensionOLD;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.Swivel;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.VerticalLift;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.Vision;

@TeleOp
public class Auto_Test extends LinearOpMode {

    //swivel variable based on cv
    public static double angleCVTest;

    @Override
    public void runOpMode() {

        Pose2d beginPose = new Pose2d(15.2, 62.35, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        VerticalLift lift = new VerticalLift(hardwareMap);
        FourBarClaw claw = new FourBarClaw(hardwareMap);

        HorizontalExtension intake = new HorizontalExtension(hardwareMap);

        Vision vision = new Vision(hardwareMap, drive);
        Swivel swivel = new Swivel(hardwareMap);

        TrajectoryActionBuilder forward = drive.actionBuilder(beginPose)
                .lineToY(52);

        TrajectoryActionBuilder scan1 = forward.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(25, 52), new TranslationalVelConstraint(3));

        TrajectoryActionBuilder scan2 = scan1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(25, 50))
                .strafeToConstantHeading(new Vector2d(15, 50), new TranslationalVelConstraint(3));


        Actions.runBlocking(claw.drop());

        Actions.runBlocking(swivel.turn(0.5));

        Actions.runBlocking(
                intake.retract(.7)
        );

        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                lift.SetSlidePos(200),
                                new SleepAction(1),
                                intake.retract(.3),
                                forward.build(),
                                new RaceAction(
                                        new SequentialAction(
                                                scan1.build(),
                                                scan2.build()
                                        ),
                                        new SequentialAction(
                                                new SleepAction(1),
                                                vision.scan()
                                        )
                                ),
                                vision.updatePose(),
                                swivel.turnCV(),
                                new SleepAction(1),
                                lift.SetSlidePos(20),
                                new SleepAction(1),
                                claw.grab(),
                                new SleepAction(.25),
                                lift.SetSlidePos(200),
                                new SleepAction(.25),
                                swivel.turn(.5),
                                intake.retract(.7)
                        ),
                        lift.move()
                )
        );
    }
}

/*
swivel.turnCV(),
                                intake.retract(.46),

 */