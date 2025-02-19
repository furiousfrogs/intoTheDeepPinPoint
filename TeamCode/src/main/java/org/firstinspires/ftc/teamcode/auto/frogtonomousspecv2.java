package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.var;

@Config
@Autonomous(name = "frogtonomousspecv2", group = "Autonomous")
public class frogtonomousspecv2 extends LinearOpMode {

    public boolean motorswitch = false;

    public class intake{
        private Servo intakeL, intakeR, intakedown, gate;
        private TouchSensor htouch;
        private DcMotor hslide, roller;
        private ElapsedTime timer = new ElapsedTime();

        public intake(HardwareMap hardwareMap){
            hslide = hardwareMap.get(DcMotor.class, "horslide");
            hslide.setDirection(DcMotor.Direction.REVERSE);
            hslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hslide.setTargetPosition(0);
            hslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            intakedown = hardwareMap.get(Servo.class, "inwrist");
            intakedown.setDirection(Servo.Direction.FORWARD);
            intakedown.setPosition(var.inWristTransfer);

            intakeL = hardwareMap.get(Servo.class, "leftin");
            intakeL.setDirection(Servo.Direction.REVERSE);
            intakeL.setPosition(var.inTransfer);

            intakeR = hardwareMap.get(Servo.class, "rightin");
            intakeR.setDirection(Servo.Direction.FORWARD);
            intakeR.setPosition(var.inTransfer);

            roller = hardwareMap.get(DcMotor.class, "intake");
            roller.setDirection(DcMotor.Direction.FORWARD);
            roller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            roller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            htouch = hardwareMap.get(TouchSensor.class, "hortouch");

            gate = hardwareMap.get(Servo.class, "gate");
            gate.setDirection(Servo.Direction.FORWARD);
            gate.setPosition(var.gateClose);
        }

        public class getblock implements Action{
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    roller.setPower(1);
                    hslide.setPower(0.9);
                    intakeL.setPosition(var.inDown);
                    intakeR.setPosition(var.inDown);
                    intakedown.setPosition(var.inWristIntaking);
                    initialized = true;
                }
                hslide.setTargetPosition(450);
                motorswitch = true;
                return false;
            }
        }
        public Action getspecimen() {
            return new getblock();
        }

        public class getblockshort implements Action{
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    roller.setPower(1);
                    hslide.setPower(0.9);
                    intakeL.setPosition(var.inDown);
                    intakeR.setPosition(var.inDown);
                    intakedown.setPosition(var.inWristIntaking);
                    initialized = true;
                }
                hslide.setTargetPosition(400);
                motorswitch = true;
                return false;
            }
        }
        public Action getspecimenshort() {
            return new getblockshort();
        }

        public class outblock implements Action{
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    intakedown.setPosition(var.inWristSpitAuto);
                    roller.setPower(-1);
                    initialized = true;
                }
                intakedown.setPosition(var.inWristIntaking);
                return false;
            }
        }
        public Action outspecimen() {
            return new outblock();
        }

        public class comeback implements Action{
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakedown.setPosition(var.inWristSpitAuto);
                    hslide.setPower(-1);
                    hslide.setTargetPosition(200);
                    initialized = true;
                }
                return false;
            }
        }
        public Action slideback() {
            return new comeback();
        }

        public class intakereturns implements Action{
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    intakeL.setPosition(var.inTransfer);
                    intakeR.setPosition(var.inTransfer);
                    roller.setPower(0);
                    hslide.setPower(1);
                    hslide.setTargetPosition(0);
                    initialized = true;
                }
                if (motorswitch == false && !htouch.isPressed()){
                    hslide.setPower(0.3);
                    hslide.setTargetPosition(-10);
                    return true;
                }
                if (motorswitch == false && htouch.isPressed()){
                    hslide.setPower(0);
                    hslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    hslide.setTargetPosition(0);
                    hslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    return true;
                }
                if (motorswitch == true){
                    return false;
                }
                return true;
            }
        }
        public Action intakereturn() {
            return new intakereturns();
        }

        public class wristdowns implements Action{
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeL.setPosition(var.inSpit);
                    intakeR.setPosition(var.inSpit);
                    initialized = true;
                }
                return false;
            }
        }
        public Action wristdown() {
            return new wristdowns();
        }
    }

    public class outtake{
        private DcMotor vslideL, vslideR;
        private Servo armbase, wrist, claw;
        private TouchSensor vtouch;
        private ElapsedTime timer = new ElapsedTime();

        double specPickupTimer = Double.MAX_VALUE;
        double specPickupTimer2 = Double.MAX_VALUE;

        public void init() {

            timer.reset();
            vslideL = hardwareMap.get(DcMotor.class, "leftvertical");
            vslideL.setDirection(DcMotor.Direction.FORWARD);
            vslideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vslideL.setTargetPosition(0);
            vslideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            vslideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            vslideR = hardwareMap.get(DcMotor.class, "rightvertical");
            vslideR.setDirection(DcMotor.Direction.REVERSE);
            vslideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vslideR.setTargetPosition(0);
            vslideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            vslideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
        public outtake(HardwareMap hardwareMap){
            armbase = hardwareMap.get(Servo.class, "outarm");
            armbase.setDirection(Servo.Direction.FORWARD);
            armbase.setPosition(var.armInit);

            wrist = hardwareMap.get(Servo.class, "wrist");
            wrist.setDirection(Servo.Direction.FORWARD);
            wrist.setPosition(var.wristInit);

            claw = hardwareMap.get(Servo.class, "claw");
            claw.setDirection(Servo.Direction.FORWARD);
            claw.setPosition(var.clawCloseTight);

            vtouch = hardwareMap.get(TouchSensor.class, "vertouch");
        }

        public class scorespecs implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    claw.setPosition(var.clawCloseTight);
                    armbase.setPosition(var.armSpecScore);
                    wrist.setPosition(var.wristSpecScore);
                    vslideL.setPower(1);
                    vslideR.setPower(1);
                    initialized = true;
                }
                vslideL.setTargetPosition(var.slideSpecScore);
                vslideR.setTargetPosition(var.slideSpecScore);
                return false;
            }
        }
        public Action scorespecimen() {
            return new scorespecs();
        }

        public class grabspecs implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    timer.reset();
                    claw.setPosition(var.clawClose);
                    vslideL.setPower(1);
                    vslideR.setPower(1);
                    initialized = true;
                }
                vslideL.setTargetPosition(var.slideSpecPickup);
                vslideR.setTargetPosition(var.slideSpecPickup);

                if (timer.seconds() > 0.2){//can be faster?
                    if (timer.seconds() > 0.5){
                        claw.setPosition(var.clawOpenWide);
                        timer.reset();
                        return false;
                    }
                    wrist.setPosition(var.wristSpec);
                    armbase.setPosition(var.armSpec);
                    return true;
                }
                return true;
            }
        }
        public Action grabspecimen() {
            return new grabspecs();
        }

        public class outtakereturns implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    wrist.setPosition(var.wristInit);
                    armbase.setPosition(var.armTransfer);//TODO idk here
                    claw.setPosition(var.clawClose);
                    vslideL.setPower(-1);
                    vslideR.setPower(-1);
                    initialized = true;
                }
                vslideL.setTargetPosition(0);
                vslideR.setTargetPosition(0);
                return false;
            }
        }
        public Action outtakereturn() {
            return new outtakereturns();
        }

        public class openclaws implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    claw.setPosition(var.clawOpenWide);
                    initialized = true;
                }
                return false;
            }
        }
        public Action openclaw() {
            return new openclaws();
        }

        public class closeclaws implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    claw.setPosition(var.clawCloseTight);
                    initialized = true;
                }
                return false;
            }
        }
        public Action closeclaw() {
            return new closeclaws();
        }

        public class raiseslight implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    vslideL.setPower(1);
                    vslideR.setPower(1);
                    initialized = true;
                }
                vslideL.setTargetPosition(600);
                vslideR.setTargetPosition(600);
                return false;
            }
        }
        public Action specraise() {
            return new raiseslight();
        }
    }

    @Override
    public void runOpMode() {
        int startPosition = 1;
        Pose2d initialPose = new Pose2d(-9.3, 61.7, Math.toRadians(270));
        MecanumDrive frogbot = new MecanumDrive(hardwareMap, initialPose);

        intake eatfroggy = new intake(hardwareMap);
        outtake scorefroggy = new outtake(hardwareMap);
        scorefroggy.init();

        TrajectoryActionBuilder traj1 = frogbot.actionBuilder(initialPose)
                .afterTime(0.0001, scorefroggy.scorespecimen())

                .strafeToConstantHeading(new Vector2d(4, 29))
                .afterTime(0.001, scorefroggy.openclaw())
                .waitSeconds(0.15)
                .afterTime(0.4, scorefroggy.grabspecimen())
                .afterTime(0.8, eatfroggy.wristdown())

                .strafeToLinearHeading(new Vector2d(var.specPickupx1, 45), Math.toRadians(var.spec1PickupHeading)) //1
                .afterTime(0.0001, eatfroggy.getspecimenshort())
                .waitSeconds(0.7)
                .afterTime(0.1, eatfroggy.slideback())
                .waitSeconds(0.2)

                .strafeToLinearHeading(new Vector2d(var.specPickupx1+1, 45), Math.toRadians(140))
                .afterTime(0.1, eatfroggy.outspecimen())
                .waitSeconds(0.2)

                .strafeToLinearHeading(new Vector2d(var.specPickupx2, 45), Math.toRadians(var.spec2PickupHeading)) //2
                .afterTime(0.0001, eatfroggy.getspecimenshort())
                .waitSeconds(0.7)
                .afterTime(0.1, eatfroggy.slideback())
                .waitSeconds(0.2)

                .strafeToLinearHeading(new Vector2d(var.specPickupx2+1, 45), Math.toRadians(140))
                .afterTime(0.1, eatfroggy.outspecimen())
                .waitSeconds(0.2)


                .strafeToLinearHeading(new Vector2d(var.specPickupx3, 45), Math.toRadians(var.spec3PickupHeading)) //3
                .afterTime(0.0001, eatfroggy.getspecimen())
                .waitSeconds(0.7)
                .afterTime(0.1, eatfroggy.slideback())
                .waitSeconds(0.1)

                .strafeToLinearHeading(new Vector2d(-40, 48), Math.toRadians(140))
                .afterTime(0.1, eatfroggy.outspecimen())
                .waitSeconds(0.2)
                .afterTime(0.0001, eatfroggy.slideback())
                .afterTime(0.4, eatfroggy.intakereturn())

                .strafeToSplineHeading(new Vector2d(-39, 62), Math.toRadians(270))
                .afterTime(0.00001, scorefroggy.closeclaw())
                .waitSeconds(0.15)
                .afterTime(0.2, scorefroggy.specraise())
                .waitSeconds(0.3)
                .afterTime(0.1, scorefroggy.scorespecimen())

                .strafeToConstantHeading(new Vector2d(0, 27.5))
                .afterDisp(0.2, scorefroggy.openclaw())
                .afterTime(0.8, scorefroggy.grabspecimen())

                .strafeToConstantHeading(new Vector2d(-39, 62))
                .afterTime(0.00001, scorefroggy.closeclaw())
                .waitSeconds(0.2)
                .afterTime(0.1, scorefroggy.specraise())
                .waitSeconds(0.3)
                .afterTime(0.1, scorefroggy.scorespecimen())

                .strafeToConstantHeading(new Vector2d(0, 27.5))
                .afterDisp(0.2, scorefroggy.openclaw())
                .afterTime(0.8, scorefroggy.grabspecimen())

                .strafeToConstantHeading(new Vector2d(-39, 62))
                .afterTime(0.00001, scorefroggy.closeclaw())
                .waitSeconds(0.2)
                .afterTime(0.1, scorefroggy.specraise())
                .waitSeconds(0.3)
                .afterTime(0.1, scorefroggy.scorespecimen())

                .strafeToConstantHeading(new Vector2d(-0, 27.5))
                .afterDisp(0.2, scorefroggy.openclaw())
                .afterTime(0.8, scorefroggy.grabspecimen())

                .strafeToConstantHeading(new Vector2d(-39, 62))
                .afterTime(0.00001, scorefroggy.closeclaw())
                .waitSeconds(0.2)
                .afterTime(0.1, scorefroggy.specraise())
                .waitSeconds(0.3)
                .afterTime(0.1, scorefroggy.scorespecimen())

                .strafeToConstantHeading(new Vector2d(-2, 27.5))
                .afterDisp(0.2, scorefroggy.openclaw())
                .strafeToConstantHeading(new Vector2d(-4, 31))
                .strafeToLinearHeading(new Vector2d(-30, 40), Math.toRadians(130))
                .afterDisp(0.0001, eatfroggy.getspecimen())
                .waitSeconds(2);

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        traj1.build()
                )
        );
    }
}
