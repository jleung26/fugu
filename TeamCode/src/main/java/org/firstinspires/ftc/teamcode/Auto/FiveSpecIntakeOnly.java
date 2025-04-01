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


@Autonomous(name = "5 Spec Intake and Pathing Only", group = "A")
public class FiveSpecIntakeOnly extends OpMode {
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
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of Robot */
    private final Pose startPose = new Pose(6.95, 65, Math.toRadians(0));

    /** Scoring Pose for Preloaded Spec */
    private final Pose score0Pose = new Pose(42, 72, Math.toRadians(0));

    /** Intermediate pose so robot doesn't bang into sub */
    private final Pose intake1ControlPose = new Pose(24, 58, Math.toRadians(290));

    /** Intake First Sample from the Spike Mark */
    private final Pose intake1Pose = new Pose(24, 44, Math.toRadians(317));

    /** Spit out First Sample */
    private final Pose eject1Pose = new Pose(24, 42, Math.toRadians(230));

    /** Intake Second Sample from the Spike Mark */
    private final Pose intake2Pose = new Pose(24, 38, Math.toRadians(318));

    /** Spit out Second Sample */
    private final Pose eject2Pose = new Pose(24, 35, Math.toRadians(230));

    /** Intake Third Sample from the Spike Mark */
    private final Pose intake3Pose = new Pose(28, 26, Math.toRadians(310));

    /** Spit out Third Sample */
    private final Pose eject3Pose = new Pose(24, 33, Math.toRadians(230));

    /** Pick up from wall, reusable */
    private final Pose pickupWallPose = new Pose(6.95, 35, Math.toRadians(0));

    private final Pose pickupWall1ControlPose = new Pose(12, 35, Math.toRadians(0));

    /** Bezier Control Point, reusable  */ // using tangential heading interpolation maybe?
    private final Pose scoreControlPose1 = new Pose(15, 34.5, Math.toRadians(999) /*heading unused*/);

    private final Pose scoreIntermediatePose = new Pose(19.6, 37, Math.toRadians(30));

    // might use a second, third, fourth control point for pathing,

    /** Score 2nd sample (sample index 1) */
    private final Pose score1Pose = new Pose(40, 62, Math.toRadians(30));

    /** Score 3rd sample (sample index 2) */
    private final Pose score2Pose = new Pose(40, 62, Math.toRadians(30));

    /** Score 4th sample (sample index 3) */
    private final Pose score3Pose = new Pose(40, 62, Math.toRadians(30));

    /** Score 5th sample (sample index 4) */
    private final Pose score4Pose = new Pose(40, 62, Math.toRadians(30));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(6.95, 30, Math.toRadians(0));

    private int movingEjectAngleThreshold = 250;

//    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
//     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
//    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private PathChain score0, prepIntake1, intake1, eject1, intake2, eject2, intake3, eject3, grab1, score1, grab2, score2, grab3, score3, grab4, score4, park;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        score0 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(score0Pose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), score0Pose.getHeading())
                .build();

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score0Pose), new Point(intake1ControlPose), new Point(intake1Pose)))
                .setLinearHeadingInterpolation(score0Pose.getHeading(), intake1Pose.getHeading()) // TODO: could try tangential tbh
                .build();

        eject1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake1Pose), new Point(eject1Pose)))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), eject1Pose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(eject1Pose), new Point(intake2Pose)))
                .setLinearHeadingInterpolation(eject1Pose.getHeading(), intake2Pose.getHeading())
                .build();

        eject2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake2Pose), new Point(eject2Pose)))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), eject2Pose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(eject2Pose), new Point(intake3Pose)))
                .setLinearHeadingInterpolation(eject2Pose.getHeading(), intake3Pose.getHeading())
                .build();

        eject3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake3Pose), new Point(eject3Pose)))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), eject3Pose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        grab1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(eject3Pose), new Point(eject3Pose))) // veiled turnTo
                .setLinearHeadingInterpolation(eject3Pose.getHeading(), eject3Pose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .addPath(new BezierCurve(new Point(eject3Pose), new Point(pickupWall1ControlPose), new Point(pickupWallPose)))
                .setConstantHeadingInterpolation(pickupWallPose.getHeading())
                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupWallPose), new Point(scoreControlPose1), new Point(scoreIntermediatePose)))
                .setTangentHeadingInterpolation()
                .setReversed(false)
                .setPathEndTimeoutConstraint(0)
                .addPath(new BezierLine(new Point(scoreIntermediatePose), new Point(score1Pose)))
                .setConstantHeadingInterpolation(score1Pose.getHeading())
                .setPathEndTimeoutConstraint(0) // maybe this would help as a "looser" clause as mentioned below in switch statement?
                .build();

        grab2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score1Pose), new Point(scoreIntermediatePose)))
                .setConstantHeadingInterpolation(scoreIntermediatePose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .addPath(new BezierCurve(new Point(scoreIntermediatePose), new Point(scoreControlPose1), new Point(pickupWallPose)))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupWallPose), new Point(scoreControlPose1), new Point(scoreIntermediatePose)))
                .setTangentHeadingInterpolation()
                .setReversed(false)
                .setPathEndTimeoutConstraint(0)
                .addPath(new BezierLine(new Point(scoreIntermediatePose), new Point(score2Pose)))
                .setConstantHeadingInterpolation(score2Pose.getHeading())
                .build();

        grab3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score2Pose), new Point(scoreIntermediatePose)))
                .setConstantHeadingInterpolation(scoreIntermediatePose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .addPath(new BezierCurve(new Point(scoreIntermediatePose), new Point(scoreControlPose1), new Point(pickupWallPose)))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupWallPose), new Point(scoreControlPose1), new Point(scoreIntermediatePose)))
                .setTangentHeadingInterpolation()
                .setReversed(false)
                .setPathEndTimeoutConstraint(0)
                .addPath(new BezierLine(new Point(scoreIntermediatePose), new Point(score3Pose)))
                .setConstantHeadingInterpolation(score3Pose.getHeading())
                .build();

        grab4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score1Pose), new Point(scoreIntermediatePose)))
                .setConstantHeadingInterpolation(scoreIntermediatePose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .addPath(new BezierCurve(new Point(scoreIntermediatePose), new Point(scoreControlPose1), new Point(pickupWallPose)))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();

        score4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupWallPose), new Point(scoreControlPose1), new Point(scoreIntermediatePose)))
                .setTangentHeadingInterpolation()
                .setReversed(false)
                .setPathEndTimeoutConstraint(0)
                .addPath(new BezierLine(new Point(scoreIntermediatePose), new Point(score4Pose)))
                .setConstantHeadingInterpolation(score4Pose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score4Pose), new Point(parkPose)))
                .setLinearHeadingInterpolation(score4Pose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // drive up to bar
//                grabAndPrepClipAction();
                follower.followPath(score0);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    // get away from sub, and get ready to grab first sample
                    follower.followPath(intake1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    // grab first sample
                    prepToIntakeAction();
                    extendIntakeAction();
                    setPathState(4);
                }
                break;
            case 4:
                if (intake.chamberState != Intake.IntakeChamberState.EMPTY) {
                    // successfully grabbed, turn to eject
                    flipUpAction();
                    follower.followPath(eject1);
                    setPathState(5);
                }
                break;
            case 5:
                if (follower.getPose().getHeading() < Math.toRadians(movingEjectAngleThreshold)) {
                    // eject
                    ejectAction();
                    setPathState(6);
                }
                break;
            case 6:
                if (intake.chamberState == Intake.IntakeChamberState.EMPTY) {
                    // rinse and repeat, turn and get ready to grab
                    prepToIntakeAction();
                    follower.followPath(intake2, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    extendIntakeAction();
                    setPathState(8);
                }
                break;
            case 8:
                if (intake.chamberState != Intake.IntakeChamberState.EMPTY) {
                    flipUpAction();
                    follower.followPath(eject2);
                    setPathState(9);
                }
                break;
            case 9:
                if (follower.getPose().getHeading() < Math.toRadians(movingEjectAngleThreshold)) {
                    ejectAction();
                    setPathState(10);
                }
                break;
            case 10:
                if (intake.chamberState == Intake.IntakeChamberState.EMPTY) {
                    prepToIntakeAction();
                    follower.followPath(intake3, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    extendIntakeAction();
                    setPathState(12);
                }
                break;
            case 12:
                if (intake.chamberState != Intake.IntakeChamberState.EMPTY) {
                    flipUpAction();
                    runningActions.add(new InstantAction(() -> horizontalSlides.extendPartial()));
                    follower.followPath(eject3);
                    setPathState(13);
                }
                break;
            case 13:
                if (follower.getPose().getHeading() < Math.toRadians(movingEjectAngleThreshold)) {
                    ejectAction();
                    setPathState(14);
                }
                break;
            case 14: // done with grabbing samples, now cycles
                if (intake.chamberState == Intake.IntakeChamberState.EMPTY) {
                    // drive over and prep to grab
                    retractIntakeAction();
                    follower.setMaxPower(0.7);
                    follower.followPath(grab1, true);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
//                    // at pose, now grab clip
//                    grabAndPrepClipAction();
//                    setPathState(16);
//                }
//                break;
//            case 16:
//                if (outtake.armPitch.armPos == ExtendingOuttake.ArmPitch.STATE.SCORING_CLIP) {
                    // grabbed, drive to score
                    follower.setMaxPower(1);
                    follower.followPath(score1, true);
                    setPathState(17);
                }
                break;
            case 17:
                if(!follower.isBusy()) { // might use a different, more loose clause here
                    // let go to score once at pose, and shoved on
//                    finishScoringClipAction();
//                    setPathState(18);
//                }
//                break;
//            case 18:
//                if (outtake.armPitch.armPos == ExtendingOuttake.ArmPitch.STATE.GRABBING_CLIP) {
                    // drive to grab another once done with slam
                    follower.setMaxPower(0.8); // slower so human player can react
                    follower.followPath(grab2, true);
                    setPathState(19);
                }
                break;
            case 19:
                if (!follower.isBusy()) {
//                    // at pose, grab clip, rinse and repeat
//                    grabAndPrepClipAction();
//                    setPathState(20);
//                }
//                break;
//            case 20:
//                if (outtake.armPitch.armPos == ExtendingOuttake.ArmPitch.STATE.SCORING_CLIP) {
                    follower.setMaxPower(1);
                    follower.followPath(score2, true);
                    setPathState(21);
                }
                break;
            case 21:
                if(!follower.isBusy()) {
//                    finishScoringClipAction();
//                    setPathState(22);
//                }
//                break;
//            case 22:
//                if (outtake.armPitch.armPos == ExtendingOuttake.ArmPitch.STATE.GRABBING_CLIP) {
                    follower.setMaxPower(0.8);
                    follower.followPath(grab3, true);
                    setPathState(23);
                }
                break;
            case 23:
                if (!follower.isBusy()) {
//                    grabAndPrepClipAction();
//                    setPathState(24);
//                }
//                break;
//            case 24:
//                if (outtake.armPitch.armPos == ExtendingOuttake.ArmPitch.STATE.SCORING_CLIP) {
                    follower.setMaxPower(1);
                    follower.followPath(score3, true);
                    setPathState(25);
                }
                break;
            case 25:
                if(!follower.isBusy()) {
//                    finishScoringClipAction();
//                    setPathState(26);
//                }
//                break;
//            case 26:
//                if (outtake.armPitch.armPos == ExtendingOuttake.ArmPitch.STATE.GRABBING_CLIP) {
                    follower.setMaxPower(0.8);
                    follower.followPath(grab4, true);
                    setPathState(27);
                }
                break;
            case 27:
                if (!follower.isBusy()) {
//                    grabAndPrepClipAction();
//                    setPathState(28);
//                }
//                break;
//            case 28:
//                if (outtake.armPitch.armPos == ExtendingOuttake.ArmPitch.STATE.SCORING_CLIP) {
                    follower.setMaxPower(1);
                    follower.followPath(score3, true);
                    setPathState(29);
                }
                break;
            case 29:
                if(!follower.isBusy()) {
//                    finishScoringClipAction();
//                    setPathState(30);
//                }
//                break;
//            case 30:
//                if (outtake.armPitch.armPos == ExtendingOuttake.ArmPitch.STATE.GRABBING_CLIP) {
                    // huzzah we done, park
                    // please let this is within the 30 seconds, I'm programming completely blind with no pedro visualizer
                    follower.followPath(park, true);
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
        actionTimer = new Timer();

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
        outtake.closeClawTight();
        outtake.toGrabClip();
        intake.flipUp();
    }

    @Override
    public void init_loop() {
        // very important to select for when eventually >5 spec
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
        actionTimer.resetTimer();
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
        telemetry.addData("path state index: ", pathState);
        telemetry.addData("isBusy? ", follower.isBusy());
        telemetry.addData("pinpoint cooked? ", follower.isLocalizationNAN());
        telemetry.addData("robot stuck? ", follower.isRobotStuck());

        telemetry.addLine("\n Pose");
        telemetry.addData("x: ", follower.getPose().getX());
        telemetry.addData("y: ", follower.getPose().getY());
        telemetry.addData("heading: ", follower.getPose().getHeading());

        telemetry.addData("\n Loop Times: ", elapsedtime.milliseconds());
    }


    // Abstraction Action Methods
//    public void grabAndPrepClipAction() {
//        runningActions.add(new SequentialAction(
//                new InstantAction(() -> outtake.closeClawTight()),
//                new SleepAction(0.3),
//                new InstantAction(() -> outtake.toPrepScoreClip()),
//                new InstantAction(() -> verticalSlides.raiseToPrepClip())
//        ));
//    }
//
//    public void finishScoringClipAction() {
//        runningActions.add(new SequentialAction(
//                new InstantAction(() -> outtake.openClaw()),
//                new SleepAction(0.2),
//                new InstantAction(() -> outtake.toGrabClip()),
//                new InstantAction(() -> verticalSlides.raiseToPickupClip())
//        ));
//    }

    public void flipUpAction() {
        runningActions.add(new ParallelAction(
                new InstantAction(() -> intake.flipUp()),
                new InstantAction(() -> intake.neutral())
        ));
    }

    public void ejectAction() {
        runningActions.add(new InstantAction(() -> intake.reverse()));
    }

    public void extendIntakeAction() { // fully extend to grab
        runningActions.add(new InstantAction(() -> horizontalSlides.extend()));
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
                new InstantAction(() -> intake.fullStop())
        ));
    }
}