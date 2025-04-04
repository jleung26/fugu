package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;
import org.firstinspires.ftc.teamcode.Subsystem.VerticalSlides;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

import java.util.ArrayList;
import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "4 Sample Intake Only", group = "A")
public class FourSampleIntakeOnly extends OpMode {
    // declaring subsystems
    RobotHardware robotHardware = new RobotHardware();
    VerticalSlides verticalSlides = new VerticalSlides();
    HorizontalSlides horizontalSlides = new HorizontalSlides();
    Outtake outtake = new Outtake();
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

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    // TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
    // no visualizer access rn, so need to determine poses
    // also modify extendPartial fraction, also find 42" extension limit
    // use 42" limit in visualizer, might have to input 84" bot lol

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(6.25, 115, Math.toRadians(270)); // TODO: tune

    /** Bucket Scoring Pose */
    private final Pose scorePose = new Pose(12, 132, Math.toRadians(315)); // TODO: tune

    /** First Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(15, 123, Math.toRadians(0)); // TODO: tune

    /** Second Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(15, 133, Math.toRadians(0)); // tuned

    /** Third Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(25, 120, Math.toRadians(55)); // tuned

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(62, 98, Math.toRadians(270)); // tuned

    private final Pose parkControlPose = new Pose(64, 110, Math.toRadians(999)/* heading unused*/); // done

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;


    public void buildPaths() {

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. Straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                // TODO: maybe constant heading here?
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    // extend to prep for picking up sample
                    prepToIntakeAction();
                    setPathState(990);
                }
                break;
            case 990:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) { // delay
                    follower.followPath(grabPickup1,true);
                    setPathState(2);
                }
                break;
            case 2:
                // TODO: follower gets stuck here usually, maybe just a goofy pathing thing
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    runningActions.add(new InstantAction(() -> horizontalSlides.extend()));

                    setPathState(3);
                }
                break;
            case 3:
                // wait until sample picked up
                if (intake.chamberState != Intake.IntakeChamberState.EMPTY) {
                    // retract and prep for transfer
                    retractIntakeAction();

                    // drive up to scoring
                    follower.followPath(scorePickup1,true);
                    setPathState(4);
                } else if (pathTimer.getElapsedTimeSeconds() > 2.5) { // intake's emptiness already assumed
                    setPathState(991);
                } else if (pathTimer.getElapsedTimeSeconds() > 1) { // Empty and less than 3 already assumed
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> intake.flipUp()),
                            new SleepAction(0.5),
                            new InstantAction(() -> intake.dropDown())
                    ));
                }
                break;
            case 4:
                /* ready to transfer */
                if(/*arm stuff*/horizontalSlides.slidesRetracted && intake.wristFlippedUp) {
                    setPathState(991);
                }
                break;
            case 991:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.5 && intake.chamberState == Intake.IntakeChamberState.EMPTY) {
                    prepToIntakeAction();

                    follower.followPath(grabPickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    runningActions.add(new InstantAction(() -> horizontalSlides.extend()));

                    setPathState(7);
                }
                break;
            case 7:
                // wait until sample picked up
                if (intake.chamberState != Intake.IntakeChamberState.EMPTY) {
                    // retract and prep for transfer
                    retractIntakeAction();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    setPathState(8);
                }
                break;
            case 8:
                /* ready to transfer */
                if(/*arm stuff*/ horizontalSlides.slidesRetracted && intake.wristFlippedUp) {

                    setPathState(992);
                }
                break;
            case 992:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.5 && intake.chamberState == Intake.IntakeChamberState.EMPTY) {
                    follower.followPath(grabPickup3,true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    prepToIntakeAction();
                    runningActions.add(new InstantAction(() -> horizontalSlides.extend()));
                    setPathState(11);
                }
                break;
            case 11:
                // wait until sample picked up
                if (intake.chamberState != Intake.IntakeChamberState.EMPTY) {
                    // retract and prep for transfer
                    retractIntakeAction();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3,true);
                    setPathState(12);
                }
                break;
            case 12:
                /* ready to transfer */
                if(/*arm stuff*/horizontalSlides.slidesRetracted && intake.wristFlippedUp) {
                    /* Transfer and prep to score*/
                    setPathState(993);
                }
                break;
            case 993:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.5 && intake.chamberState == Intake.IntakeChamberState.EMPTY) { // delay
                    follower.followPath(park,true);
                    setPathState(13);
                }
                break;
            case 13:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /** Level 1 Ascent */
                    // TODO: find a pose where the arm won't be out of servo range after teleop start
                    //       or if we're feeling frisky, we try grabbing another sample to-go for teleop
                    //       if we're feeling crazy, we try a 5/6 sample auto

                    /* Set the state to a case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
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
        opmodeTimer.resetTimer();

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
        outtake.toStow();
        outtake.closeClawLoose();
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
//        verticalSlides.operate();
        horizontalSlides.operateAuto();
        intake.operateColorChecking();

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addLine("\n States");
        telemetry.addData("path state: ", pathState);
        telemetry.addData("isBusy? ", follower.isBusy());
        telemetry.addData("pinpoint cooked? ", follower.isLocalizationNAN());
        telemetry.addData("robot stuck? ", follower.isRobotStuck());
        telemetry.addData("path timer seconds", pathTimer.getElapsedTimeSeconds());

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
                new InstantAction(() -> verticalSlides.raiseToLowBucket()),
                new SleepAction(0.5),
                new InstantAction(() -> outtake.toScoreBucket())
        ));
    }

    public void transferAndScoreAction() {
        // full transfer and prep to score sequence
        runningActions.add(new SequentialAction(
                new InstantAction(() -> verticalSlides.retract()),
                new SleepAction(0.5),
                new InstantAction(() -> outtake.closeClawLoose()),
                new SleepAction(0.2),
                new InstantAction(() -> intake.dropDown()),
                new SleepAction(0.1),
                new InstantAction(() -> verticalSlides.raiseToLowBucket()),
                new SleepAction(0.4),
                new InstantAction(() -> intake.flipUp()),
                new SleepAction(0.3),
                new InstantAction(() -> outtake.toScoreBucket())
        ));
    }

    public void depositSampleAction() {
        // deposit in bucket, then retract all
        runningActions.add(new SequentialAction(
                // might need delay here
                new InstantAction(() -> outtake.openClaw()),
                new SleepAction(0.25),
                new ParallelAction(
                        new InstantAction(() -> outtake.toStow()),
                        new InstantAction(() -> verticalSlides.retract())
                )
        ));
    }

    public void prepToIntakeAction() {
        runningActions.add(new SequentialAction(
                new InstantAction(() -> horizontalSlides.extendPartial()),
                new InstantAction(() -> intake.dropDown()),
                new InstantAction(() -> intake.intake())
        ));
    }

    public void retractIntakeAction() {
        runningActions.add(new SequentialAction(
                new InstantAction(() -> horizontalSlides.retract()),
                new InstantAction(() -> intake.flipUp()),
                new InstantAction(() -> intake.neutral())
        ));
    }

//    public void extendIntakeSlowlyAction() {
//        double DELAY = 0.05; // seconds, not milliseconds
//        // This is jank, but I don't wanna touch the PID loop that's running, so this will have to do for now
//        runningActions.add(new SequentialAction(
//                new InstantAction(() -> horizontalSlides.stepExtend()),
//                new SleepAction(DELAY),
//                new InstantAction(() -> horizontalSlides.stepExtend()),
//                new SleepAction(DELAY),
//                new InstantAction(() -> horizontalSlides.stepExtend()),
//                new SleepAction(DELAY),
//                new InstantAction(() -> horizontalSlides.stepExtend()),
//                new SleepAction(DELAY),
//                new InstantAction(() -> horizontalSlides.stepExtend()),
//                new SleepAction(DELAY),
//                new InstantAction(() -> horizontalSlides.stepExtend()),
//                new SleepAction(DELAY),
//                new InstantAction(() -> horizontalSlides.stepExtend()),
//                new SleepAction(DELAY),
//                new InstantAction(() -> horizontalSlides.stepExtend()),
//                new SleepAction(DELAY),
//                new InstantAction(() -> horizontalSlides.stepExtend()),
//                new SleepAction(DELAY),
//                new InstantAction(() -> horizontalSlides.stepExtend())
//        ));
//    }
}
