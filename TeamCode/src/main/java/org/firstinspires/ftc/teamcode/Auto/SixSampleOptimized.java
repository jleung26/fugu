package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.ExtendingOuttake;
import org.firstinspires.ftc.teamcode.Subsystem.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.VerticalSlides;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

import java.util.ArrayList;
import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "0+6 OPTIMIZED TEST", group = "A", preselectTeleOp = "Full TeleOp FINAL")
public class SixSampleOptimized extends OpMode {
    // declaring subsystems
    RobotHardware robotHardware = new RobotHardware();
    VerticalSlides verticalSlides = new VerticalSlides();
    HorizontalSlides horizontalSlides = new HorizontalSlides();
    ExtendingOuttake outtake = new ExtendingOuttake();
    Intake intake = new Intake();
    Follower follower;
    Timer pathTimer, actionTimer, opmodeTimer;

    // booleans
    // team color
    // spec/sample
    boolean redAlliance = true;
    Intake.IntakeChamberState COLOR_TO_REJECT;

    // Action stuff
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    // bulk cache reading
    private List<LynxModule> allHubs;

    // adds ability to graph telemetry values
    private MultipleTelemetry dashboardTelemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

    // loop time tracking
    private ElapsedTime elapsedtime;

    private int pathState;
    private int samplesGrabbedFromSub = 0;
    private int slidesDistanceTraveled = 0; // 1 for partial, 2 for mostly, 3 for full

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(6.25, 115, Math.toRadians(270));
    // X: bot against wall
    // Y: along closest edge of tile to bucket, just before covering the foam connecting teeth
    // back of bot towards bucket

    /** Bucket Scoring Pose */
    private final Pose score0Pose = new Pose(14.5, 134, Math.toRadians(340)); // TODO: tune

    private final Pose score1Pose = new Pose(14, 135, Math.toRadians(345)); // TODO: tune

    private final Pose score2Pose = new Pose(15.5, 137, Math.toRadians(0)); // TODO: tune

    private final Pose score3Pose = new Pose(15, 134, Math.toRadians(315)); //

    // Pick up spike marks
    private final Pose pickup1Pose = new Pose(18, 132.5, Math.toRadians(340)); // tuned

    private final Pose pickup2Pose = new Pose(17, 134.5, Math.toRadians(0)); // tuned

    private final Pose pickup3Pose = new Pose(25.5, 127.5, Math.toRadians(49)); // TODO: tune
                                            /// 24, 128, Math.toRadians(45), reliable pose, go back to if too many issues
    // to and from sub
    private final Pose scoreControlPose = new Pose(64, 113, Math.toRadians(999)/* heading unused*/);

    private final Pose subScorePose = new Pose(14, 132, Math.toRadians(320));

    private final Pose sub1Pose = new Pose(56.5, 98, Math.toRadians(270));

    private final Pose sub2Pose = new Pose(60, 98, Math.toRadians(270));

    private final Pose sub3Pose = new Pose(64, 98, Math.toRadians(270));

    private final Pose sub4Pose = new Pose(68, 98, Math.toRadians(270));

    private final Pose parkPose = new Pose(62, 98, Math.toRadians(90)); // tuned

    private final Pose parkControlPose = new Pose(64, 120, Math.toRadians(999)/* heading unused*/); // done

    // constants and thresholds
    private final double SUB_GRAB_TIMEOUT_1 = 0.75; // TODO
    private final double SUB_GRAB_TIMEOUT_2 = 1.5; // TODO
    private final double SUB_GRAB_TIMEOUT_3 = 3.5; // TODO
    private final double DEPOSIT_DELAY = 0.3; // delay to wait before follow next path after deposit sample Action
    private final double VERT_SLIDES_EXTENDED_THRESHOLD = 800;
    private final double SLIDES_STUCK_TIMEOUT = 3;
    private final double SPIKE_MARK_TIMEOUT = 1.5;
    private final double AT_SUB_Y_THRESHOLD = 99.5;
    private final double RUSH_SCORE_X_THRESHOLD = subScorePose.getX() + 3;
    private final double RUSH_SCORE_Y_THRESHOLD = subScorePose.getY() - 2;

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, scoreToSub1, scoreFrom1, sub1ToSub2, scoreToSub2, scoreFrom2, sub2ToSub3, scoreToSub3, scoreFrom3, scoreToSub4, sub3ToSub4, scoreFrom4, park;


    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(score0Pose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), score0Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score0Pose), new Point(pickup1Pose)))
                .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                .setLinearHeadingInterpolation(score0Pose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(score1Pose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), score1Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score1Pose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(score1Pose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(score2Pose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), score2Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score2Pose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(score2Pose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(score3Pose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), score3Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        scoreToSub1 = follower.pathBuilder() // to sub for first grab
                .addPath(new BezierCurve(new Point(score3Pose), /* Control Point */ new Point(parkControlPose), new Point(sub1Pose)))
                .setLinearHeadingInterpolation(score3Pose.getHeading(), sub1Pose.getHeading())
                .build();

        scoreFrom1 = follower.pathBuilder() // from sub1 pose to scoring
                .addPath(new BezierCurve(new Point(sub1Pose), /* Control Point */ new Point(scoreControlPose), new Point(subScorePose)))
                .setLinearHeadingInterpolation(sub1Pose.getHeading(), subScorePose.getHeading())
                .build();

        sub1ToSub2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sub1Pose), new Point(sub2Pose)))
                .setConstantHeadingInterpolation(sub2Pose.getHeading())
                .build();

        scoreToSub2 = follower.pathBuilder() // to sub for second grab
                .addPath(new BezierCurve(new Point(subScorePose), /* Control Point */ new Point(parkControlPose), new Point(sub2Pose)))
                .setLinearHeadingInterpolation(subScorePose.getHeading(), sub2Pose.getHeading())
                .build();

        scoreFrom2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sub2Pose), /* Control Point */ new Point(scoreControlPose), new Point(subScorePose)))
                .setLinearHeadingInterpolation(sub2Pose.getHeading(), subScorePose.getHeading())
                .build();

        // backup, not for 7th sample
        sub2ToSub3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sub2Pose), new Point(sub3Pose)))
                .setConstantHeadingInterpolation(sub3Pose.getHeading())
                .build();

        scoreToSub3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(subScorePose), /* Control Point */ new Point(parkControlPose), new Point(sub3Pose)))
                .setLinearHeadingInterpolation(subScorePose.getHeading(), sub3Pose.getHeading())
                .build();

        scoreFrom3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sub3Pose), /* Control Point */ new Point(scoreControlPose), new Point(subScorePose)))
                .setLinearHeadingInterpolation(sub3Pose.getHeading(), subScorePose.getHeading())
                .build();

        sub3ToSub4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sub3Pose), new Point(sub4Pose)))
                .setConstantHeadingInterpolation(sub4Pose.getHeading())
                .build();

        scoreToSub4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(subScorePose), /* Control Point */ new Point(parkControlPose), new Point(sub4Pose)))
                .setLinearHeadingInterpolation(subScorePose.getHeading(), sub4Pose.getHeading())
                .build();

        scoreFrom4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sub4Pose), /* Control Point */ new Point(scoreControlPose), new Point(subScorePose)))
                .setLinearHeadingInterpolation(sub4Pose.getHeading(), subScorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(subScorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)))
                .setLinearHeadingInterpolation(subScorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                allToScoreAction();
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy() && verticalSlides.getCurrentPos() > VERT_SLIDES_EXTENDED_THRESHOLD) {
                    depositSampleAction();
                    prepToIntakeAction();
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() >= DEPOSIT_DELAY) {
                    follower.followPath(grabPickup1,true);
                    runningActions.add(new InstantAction(() -> horizontalSlides.extend()));
                    setPathState(4);
                }
                break;
//            case 3:
//                if(!follower.isTurning()) {
//                    runningActions.add(new InstantAction(() -> horizontalSlides.extend()));
//                    setPathState(4);
//                }
//                break;
            case 4:
                // wait until sample picked up
                if (intake.chamberState != Intake.IntakeChamberState.EMPTY || pathTimer.getElapsedTimeSeconds() > SPIKE_MARK_TIMEOUT) {
                    // retract and prep for transfer
                    retractIntakeAction();
                    // drive up to scoring
                    follower.followPath(scorePickup1,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* ready to transfer */
                if(/*arm stuff*/horizontalSlides.slidesRetracted && intake.wristFlippedUp) {
                    transferAndPrepToScoreAction();
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()  && verticalSlides.getCurrentPos() > VERT_SLIDES_EXTENDED_THRESHOLD) {
                    depositSampleAction();
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTimeSeconds() >= DEPOSIT_DELAY) {
                    prepToIntakeAction();
                    follower.followPath(grabPickup2,true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    runningActions.add(new InstantAction(() -> horizontalSlides.extend()));
                    setPathState(9);
                }
                break;
            case 9:
                // wait until sample picked up
                if (intake.chamberState != Intake.IntakeChamberState.EMPTY || pathTimer.getElapsedTimeSeconds() > SPIKE_MARK_TIMEOUT) {
                    // retract and prep for transfer
                    retractIntakeAction();
                    follower.followPath(scorePickup2,true);
                    setPathState(10);
                }
                break;
            case 10:
                /* ready to transfer */
                if(/*arm stuff*/ horizontalSlides.slidesRetracted && intake.wristFlippedUp) {
                    transferAndPrepToScoreAction();
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy() && verticalSlides.getCurrentPos() > VERT_SLIDES_EXTENDED_THRESHOLD) {
                    depositSampleAction();
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds() >= DEPOSIT_DELAY) {
                    follower.followPath(grabPickup3,true);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    prepToIntakeAction();
                    runningActions.add(new InstantAction(() -> horizontalSlides.extend()));
                    setPathState(14);
                }
                break;
            case 14:
                // wait until sample picked up or times out
                if (intake.chamberState != Intake.IntakeChamberState.EMPTY || pathTimer.getElapsedTimeSeconds() > SPIKE_MARK_TIMEOUT) {
                    // retract and prep for transfer
                    retractIntakeAction();
                    follower.followPath(scorePickup3,true);
                    setPathState(15);
                }
                break;
            case 15:
                /* ready to transfer */
                if(/*arm stuff*/horizontalSlides.slidesRetracted && intake.wristFlippedUp) {
                    /* Transfer and prep to score*/
                    transferAndPrepToScoreAction();
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy() && verticalSlides.getCurrentPos() > VERT_SLIDES_EXTENDED_THRESHOLD) {
                    depositSampleAction();
                    setPathState(1001);
                }
                break;



            /// GRAB: bucket to sub 1, then intake
            case 1001:
                if (pathTimer.getElapsedTimeSeconds() >= DEPOSIT_DELAY) { // delay
                    follower.followPath(scoreToSub1,true);
                    setPathState(1002);
                }
                break;
            case 1002:
                if (follower.getPose().getY() < AT_SUB_Y_THRESHOLD) {
                    subIntakeAction();
                    slidesDistanceTraveled = 1;
                    setPathState(1003);
                }
                break;
            /// GRAB: bucket to sub 2, then intake
            case 2001:
                if (pathTimer.getElapsedTimeSeconds() >= DEPOSIT_DELAY) { // delay

                    follower.followPath(scoreToSub2,true);
                    setPathState(2002);
                }
                break;
            case 2002:
                if (follower.getPose().getY() < AT_SUB_Y_THRESHOLD) {
                    subIntakeAction();
                    slidesDistanceTraveled = 1;
                    setPathState(2003);
                }
                break;
            /// GRAB: bucket to sub 3, then intake
            case 3001:
                if (pathTimer.getElapsedTimeSeconds() >= DEPOSIT_DELAY) { // delay
                    follower.followPath(scoreToSub3,true);
                    setPathState(3002);
                }
                break;
            case 3002:
                if (follower.getPose().getY() < AT_SUB_Y_THRESHOLD) {
                    subIntakeAction();
                    slidesDistanceTraveled = 1;
                    setPathState(3003);
                }
                break;
            /// GRAB: bucket to sub 4, then intake
            case 4001:
                if (pathTimer.getElapsedTimeSeconds() >= DEPOSIT_DELAY) { // delay
                    follower.followPath(scoreToSub4,true);
                    setPathState(4002);
                }
                break;
            case 4002:
                if (follower.getPose().getY() < AT_SUB_Y_THRESHOLD) {
                    subIntakeAction();
                    slidesDistanceTraveled = 1;
                    setPathState(4003);
                }
                break;

            /// INTAKE: logic at sub 1 pose
            case 1003:
                if (intake.chamberState != COLOR_TO_REJECT && intake.chamberState != Intake.IntakeChamberState.EMPTY && intake.chamberState != Intake.IntakeChamberState.UNKNOWN) {
                    samplesGrabbedFromSub++;
                    retractIntakeAction();
                    setPathState(1004); // sub 1 to bucket
                    break;
                } else if (intake.chamberState == COLOR_TO_REJECT) {
                    // reject sample and try again at same pose
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> intake.flipUp()),
                            new SleepAction(0.2),
                            new InstantAction(() -> intake.reverse()),
                            new SleepAction(0.4),
                            new InstantAction(() -> intake.intake()),
                            new InstantAction(() -> intake.dropDown())
                    )); // stays in this case
                    break;
                }
                else if (pathTimer.getElapsedTimeSeconds() > SUB_GRAB_TIMEOUT_2 && slidesDistanceTraveled == 2 && intake.chamberState == Intake.IntakeChamberState.EMPTY) {
                    // gives up and moves on to next pose
                    slidesDistanceTraveled = -1;
                    retractIntakeAction();
                    follower.followPath(sub1ToSub2);
                    setPathState(2002);
                    break;
                }
                else if (pathTimer.getElapsedTimeSeconds() > SUB_GRAB_TIMEOUT_1 && slidesDistanceTraveled == 1 && intake.chamberState == Intake.IntakeChamberState.EMPTY) {
                    slidesDistanceTraveled = 2;
                    runningActions.add(new InstantAction(() -> horizontalSlides.extend()));
                    break;
                    // stays in this case
                }
                break;

            /// INTAKE: logic at sub 2 pose
            case 2003:
                if (intake.chamberState != COLOR_TO_REJECT && intake.chamberState != Intake.IntakeChamberState.EMPTY && intake.chamberState != Intake.IntakeChamberState.UNKNOWN) {
                    samplesGrabbedFromSub++;
                    retractIntakeAction();
                    setPathState(2004); // sub 1 to bucket
                    break;
                } else if (intake.chamberState == COLOR_TO_REJECT) {
                    // reject sample and try again at same pose
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> intake.flipUp()),
                            new SleepAction(0.2),
                            new InstantAction(() -> intake.reverse()),
                            new SleepAction(0.4),
                            new InstantAction(() -> intake.intake()),
                            new InstantAction(() -> intake.dropDown())
                    )); // stays in this case
                    break;
                }
                else if (pathTimer.getElapsedTimeSeconds() > SUB_GRAB_TIMEOUT_2  && slidesDistanceTraveled == 2 && intake.chamberState == Intake.IntakeChamberState.EMPTY) { // how tf does it even make it to this if statement?
                    // gives up and moves on to next pose
                    slidesDistanceTraveled = -1;
                    retractIntakeAction();
                    follower.followPath(sub2ToSub3);
                    setPathState(3002);
                    break;
                }
                else if (pathTimer.getElapsedTimeSeconds() > SUB_GRAB_TIMEOUT_1 && slidesDistanceTraveled == 1 && intake.chamberState == Intake.IntakeChamberState.EMPTY) {
                    slidesDistanceTraveled = 2;
                    runningActions.add(new InstantAction(() -> horizontalSlides.extend()));
                    break;
                    // stays in this case
                }
                break;

            /// INTAKE: logic at sub 3 pose
            case 3003:
                 if (intake.chamberState != COLOR_TO_REJECT && intake.chamberState != Intake.IntakeChamberState.EMPTY && intake.chamberState != Intake.IntakeChamberState.UNKNOWN) {
                    samplesGrabbedFromSub++;
                    retractIntakeAction();
                    setPathState(3004); // sub 1 to bucket
                    break;
                } else if (intake.chamberState == COLOR_TO_REJECT) {
                    // reject sample and try again at same pose
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> intake.flipUp()),
                            new SleepAction(0.2),
                            new InstantAction(() -> intake.reverse()),
                            new SleepAction(0.4),
                            new InstantAction(() -> intake.intake()),
                            new InstantAction(() -> intake.dropDown())
                    )); // stays in this case
                    break;
                }
                else if (pathTimer.getElapsedTimeSeconds() > SUB_GRAB_TIMEOUT_2 && slidesDistanceTraveled == 2 && intake.chamberState == Intake.IntakeChamberState.EMPTY) {
                    slidesDistanceTraveled = -1;
                    // gives up and moves on to next pose
                    retractIntakeAction();
                    follower.followPath(sub3ToSub4);
                    setPathState(4002);
                    break;
                }
                else if (pathTimer.getElapsedTimeSeconds() > SUB_GRAB_TIMEOUT_1 && slidesDistanceTraveled == 1 && intake.chamberState == Intake.IntakeChamberState.EMPTY) {
                    slidesDistanceTraveled = 2;
                    runningActions.add(new InstantAction(() -> horizontalSlides.extend()));
                    break;
                    // stays in this case
                }
                break;

            /// INTAKE: logic at sub 4 pose
            case 4003:
                 if (intake.chamberState != COLOR_TO_REJECT && intake.chamberState != Intake.IntakeChamberState.EMPTY && intake.chamberState != Intake.IntakeChamberState.UNKNOWN) {
                    samplesGrabbedFromSub++;
                    pathTimer.resetTimer();
                    retractIntakeAction();
                    setPathState(4004); // sub 1 to bucket
                    break;
                } else if (intake.chamberState == COLOR_TO_REJECT) {
                    // reject sample and try again at same pose
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> intake.flipUp()),
                            new SleepAction(0.2),
                            new InstantAction(() -> intake.reverse()),
                            new SleepAction(0.4),
                            new InstantAction(() -> intake.intake()),
                            new InstantAction(() -> intake.dropDown())
                    )); // stays in this case
                    break;
                }
                else if (pathTimer.getElapsedTimeSeconds() > SUB_GRAB_TIMEOUT_2 && slidesDistanceTraveled == 2 && intake.chamberState == Intake.IntakeChamberState.EMPTY) {
                    // how tf does it even make it to this case?
                    // gives up and retracts in prep for teleop
                    slidesDistanceTraveled = -1;
                    retractIntakeAction();
                    setPathState(-100);
                    break;
                }
                else if (pathTimer.getElapsedTimeSeconds() > SUB_GRAB_TIMEOUT_1 && slidesDistanceTraveled == 1 && intake.chamberState == Intake.IntakeChamberState.EMPTY) {
                    slidesDistanceTraveled = 2;
                    runningActions.add(new InstantAction(() -> horizontalSlides.extend()));
                    break;
                    // stays in this case
                }
                break;



            /// SCORE: sub 1 to bucket
            case 1004:
                if (intake.wristFlippedUp && !horizontalSlides.slidesRetracted && intake.chamberState != Intake.IntakeChamberState.EMPTY) {
                    runningActions.add(new InstantAction(() -> horizontalSlides.retract()));
                    break;
                }
                else if (intake.wristFlippedUp) {
                    follower.followPath(scoreFrom1, true);
                    setPathState(1005);
                    break;
                }
                break;
            case 1005: // transfer from sub 1
                if (horizontalSlides.slidesRetracted && intake.wristFlippedUp) { // ready to transfer, runs during middle of path
                    transferAndToScoreSubAction();
                    setPathState(1006);
                }
                break;
            case 1006: // score from sub 1
                if (!follower.isBusy() && verticalSlides.getCurrentPos() > VERT_SLIDES_EXTENDED_THRESHOLD) {
                    depositSampleSubAction();
                    setPathState(2001); // back to sub to grab second sample
                    // guaranteed hasn't gotten 2 samples yet
                    // TODO: change num when 7 sample
                }
                break;
            /// SCORE: sub 2 to bucket
            case 2004:
                if (intake.wristFlippedUp && !horizontalSlides.slidesRetracted && intake.chamberState != Intake.IntakeChamberState.EMPTY) {
                    runningActions.add(new InstantAction(() -> horizontalSlides.retract()));
                    break;
                }
                else if (intake.wristFlippedUp) {
                    follower.followPath(scoreFrom2, true);
                    setPathState(2005);
                    break;
                }
                break;
            case 2005: // transfer from sub 2
                if (horizontalSlides.slidesRetracted && intake.wristFlippedUp) { // ready to transfer, runs during middle of path
                    transferAndToScoreSubAction();
                    setPathState(2006);
                }
                break;
            case 2006: // score from sub 2
                if (!follower.isBusy() && verticalSlides.getCurrentPos() > VERT_SLIDES_EXTENDED_THRESHOLD) {
                    depositSampleSubAction();
                    if (samplesGrabbedFromSub == 1) {
                        setPathState(3001); // not done, go back
                        break;
                    } else if (samplesGrabbedFromSub == 2) {
//                        stowToParkAction();
                        setPathState(-1); // already done, park
                        break;
                    } else {
                        telemetry.addLine("SAMPLES GRABBED INTEGER BROKE");
                        follower.turn(Math.toRadians(90), true);
                        break;
                    }
                    // TODO: COMMENT OUT above in 7 sample, no need anymore, because guaranteed not at 3 yet
                }
                break;
            /// SCORE: sub 3 to bucket
            case 3004:
                if (intake.wristFlippedUp && !horizontalSlides.slidesRetracted && intake.chamberState != Intake.IntakeChamberState.EMPTY) {
                    runningActions.add(new InstantAction(() -> horizontalSlides.retract()));
                    break;
                }
                else if (intake.wristFlippedUp) {
                    follower.followPath(scoreFrom3, true);
                    setPathState(3005);
                    break;
                }
                break;
            case 3005: // transfer from sub 3
                if (horizontalSlides.slidesRetracted && intake.wristFlippedUp) { // ready to transfer, runs during middle of path
                    transferAndToScoreSubAction();
                    setPathState(3006);
                }
                break;
            case 3006: // score from sub 3
                if (!follower.isBusy() && verticalSlides.getCurrentPos() > VERT_SLIDES_EXTENDED_THRESHOLD) {
                    depositSampleSubAction();
                    if (samplesGrabbedFromSub == 1) {
                        setPathState(4001); // not done, go back
                        break;
                    } else if (samplesGrabbedFromSub == 2) {
//                        stowToParkAction();
                        setPathState(-1); // park, already done
                        break;
                    }
//                    if (samplesGrabbedFromSub <= 2) {
//                        setPathState(4001); // not done, go back
//                    } else if (samplesGrabbedFromSub == 3) {
//                        setPathState(-1); // done, park
//                    }
                    // TODO: uncomment above when 7 sample
                }
                break;
            /// SCORE: sub 4 to bucket
            case 4004:
                if (intake.wristFlippedUp && !horizontalSlides.slidesRetracted && intake.chamberState != Intake.IntakeChamberState.EMPTY) {
                    runningActions.add(new InstantAction(() -> horizontalSlides.retract()));
                    break;
                }
                else if (intake.wristFlippedUp) {
                    follower.followPath(scoreFrom4, true);
                    setPathState(4005);
                    break;
                }
                break;
            case 4005: // transfer from sub 3
                if (horizontalSlides.slidesRetracted && intake.wristFlippedUp) { // ready to transfer, runs during middle of path
                    transferAndToScoreSubAction();
                    setPathState(4006);
                }
                break;
            case 4006: // score from sub 3
                if (!follower.isBusy() && verticalSlides.getCurrentPos() > VERT_SLIDES_EXTENDED_THRESHOLD) {
                    depositSampleParkAction();
                    setPathState(-1);
                    // even if only 1 sample grabbed, it's time to give up, there's just no way I add even more cases
                    // even if 1, 2, or 3 samples grabbed, probably out of time
                }
                break;

            /// park
            case -1:
                if (outtake.armPitch.armState != ExtendingOuttake.ArmPitch.STATE.SCORING_BUCKET) {
                    follower.followPath(park);
                    setPathState(-2);
                }
                break;
            case -2:
                if (pathTimer.getElapsedTimeSeconds() > 0.75) {
                    stowToParkAction();
                    setPathState(-100);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        // init subsystems
        elapsedtime = new ElapsedTime();
        robotHardware.initialize(this);
        verticalSlides.initialize(this, robotHardware, true);
        horizontalSlides.initialize(this, robotHardware, true);
        outtake.initialize(this, robotHardware);
        intake.initialize(this, robotHardware);

        // bulk cache reading
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();

        // ready to go
        outtake.closeClawLoose();
        outtake.toStow();
        intake.flipUp();
    }

    @Override
    public void init_loop() {
        // very important to select for when eventually >4 sample
        if (gamepad1.b || gamepad2.b) {
            redAlliance = true;
        } else if (gamepad1.x || gamepad2.x) {
            redAlliance = false;
        }

        COLOR_TO_REJECT = (redAlliance ? Intake.IntakeChamberState.BLUE : Intake.IntakeChamberState.RED);
        telemetry.addData("red alliance? ", redAlliance);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        // action and bulk caching loops
        // clearing bulk cache
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // for RR Action execution
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            if (action.run(packet)) { // actually running actions
                newActions.add(action); // if failed (run() returns true), try again next loop
            }
        }
        runningActions = newActions;

        // loops
        verticalSlides.operate();
        horizontalSlides.operateAuto();
        intake.operateColorChecking();

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addLine("\n States");
        telemetry.addData("Path state: ", pathState);
        telemetry.addData("Slide dist traveled: ", slidesDistanceTraveled);
        telemetry.addData("Samples grabbed from sub: ", samplesGrabbedFromSub);
        telemetry.addData("\n isBusy? ", follower.isBusy());
        telemetry.addData("pinpoint cooked? ", follower.isLocalizationNAN());
        telemetry.addData("robot stuck? ", follower.isRobotStuck());
        telemetry.addData("path timer seconds", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("current vert pos",  verticalSlides.getCurrentPos());

        telemetry.addLine("\n Pose");
        telemetry.addData("x: ", follower.getPose().getX());
        telemetry.addData("y: ", follower.getPose().getY());
        telemetry.addData("heading: ", follower.getPose().getHeading());

        telemetry.addData("\n Loop Times: ", elapsedtime.milliseconds());
        elapsedtime.reset();
    }

    // Abstracted Action Methods
    public void allToScoreAction() {
        // prep to score
        runningActions.add(new SequentialAction(
                new InstantAction(() -> verticalSlides.raiseToHighBucket()),
                new SleepAction(0.3),
                new InstantAction(() -> outtake.toVert()),
                new InstantAction(() -> outtake.armExtend.extendToScoreBucket()),
                new SleepAction(0.5),
                new InstantAction(() -> outtake.toScoreBucket())
        ));
    }

    public void transferAndPrepToScoreAction() {
        // full transfer and prep to score sequence
        runningActions.add(new SequentialAction(
                new InstantAction(() -> intake.setIntake(0.5)), // push sample all the way in, kinda jank, maybe not necessary
                new InstantAction(() -> outtake.toTransfer()),
                new SleepAction(0.1),
                new InstantAction(() -> outtake.closeClawTight()),
                new SleepAction(0.35), // TODO: just changed this from 0.45
                new InstantAction(() -> outtake.toStow()),
                new InstantAction(() -> intake.setIntake(0)),
                new InstantAction(() -> verticalSlides.raiseToHighBucket()),
                new SleepAction(0.2),
                new InstantAction(() -> outtake.toVert())
        ));
    }

    public void transferAndToScoreSubAction() {
        runningActions.add(new SequentialAction(
                new InstantAction(() -> intake.setIntake(Intake.INTAKING_POWER)),
                new InstantAction(() -> outtake.toTransfer()),
                new SleepAction(0.2),
                new InstantAction(() -> outtake.closeClawTight()),
                new SleepAction(0.35),
                new InstantAction(() -> outtake.toStow()),
                new SleepAction(0.1),
                new InstantAction(() -> intake.setIntake(Intake.IDLE_POWER)),
                new InstantAction(() -> verticalSlides.raiseToHighBucket()),
                new SleepAction(0.2),
                new InstantAction(() -> outtake.toScoreBucket())
        ));
    }

    public void depositSampleAction() {
        // deposit in bucket, then retract all
        runningActions.add(new SequentialAction(
                new InstantAction(() -> outtake.toScoreBucket()),
                new SleepAction(0.3),
                new InstantAction(() -> outtake.openClaw()),
                new SleepAction(0.2),
                new ParallelAction(
                        new InstantAction(() -> outtake.toStow()),
                        new InstantAction(() -> verticalSlides.retract())
                )
        ));
    }

    public void depositSampleSubAction() {
        runningActions.add(new SequentialAction(
                new InstantAction(() -> outtake.openClaw()),
                new SleepAction(0.2),
                new ParallelAction(
                        new InstantAction(() -> outtake.toStow()),
                        new InstantAction(() -> verticalSlides.retract())
                )
        ));
    }

    public void depositSampleParkAction() {
        runningActions.add(new SequentialAction(
                new InstantAction(() -> outtake.openClaw()),
                new SleepAction(0.2),
                new InstantAction(() -> outtake.toVert())
        ));
    }

    public void prepToIntakeAction() {
        runningActions.add(new SequentialAction(
                new InstantAction(() -> horizontalSlides.extendPartial()),
                new InstantAction(() -> intake.intake()),
                new InstantAction(() -> intake.dropDown())
        ));
    }

    public void subIntakeAction() {
        runningActions.add(new SequentialAction(
                new InstantAction(() -> horizontalSlides.extendBarely()),
                new InstantAction(() -> intake.intake()),
                new InstantAction(() -> intake.dropDown()),
                new InstantAction(() -> horizontalSlides.extendPartial()),
                new SleepAction(0.3),
                new InstantAction(() -> horizontalSlides.extendBarely())
        ));
    }

    public void retractIntakeAction() {
        runningActions.add(new SequentialAction(
                new InstantAction(() -> intake.flipUp()),
                new InstantAction(() -> intake.idle()),
                new InstantAction(() -> horizontalSlides.retract())
        ));
    }

    public void stowToParkAction() {
        runningActions.add(new SequentialAction(
                new ParallelAction(
                        new InstantAction(() -> intake.flipUp()),
                        new InstantAction(() -> intake.idle()),
                        new InstantAction(() -> outtake.toSubPark()),
                        new InstantAction(() -> outtake.closeClawLoose())
                ),
                new SleepAction(0.35),
                new ParallelAction(
                        new InstantAction(() -> verticalSlides.retract()),
                        new InstantAction(() -> horizontalSlides.retract())
                )
        ));
    }

}
