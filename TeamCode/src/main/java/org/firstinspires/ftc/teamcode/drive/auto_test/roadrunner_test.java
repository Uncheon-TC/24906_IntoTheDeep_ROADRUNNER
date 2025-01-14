package org.firstinspires.ftc.teamcode.drive.auto_test;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Non-RR imports
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "AUTOTEST", group = "Autonomous")

public class roadrunner_test extends LinearOpMode{

    public class H_factor {
        private Servo H_wristL;
        private Servo H_wristR;

        private Servo H_angleR;
        private Servo H_angleL;

        private Servo H_length;



        private Servo V_wristL;

        //setting default var
        int arm_target = 0;

        boolean first_count = false;

        double V_Grip_OPEN = 0.35;
        double V_Grip_CLOSE = 0.63;

        double H_Grip_OPEN = 0.55;
        double H_Grip_CLOSE = 0.85;

        boolean HG_OPEN = true;
        boolean VG_OPEN = true;

        //TODO: find Horizon Griper value

        int Low_basket = 2400;
        int High_basket = 4200;
        int clip_pick = 0;

        int High_chamber = 1800;
        int High_chamber_hang = 1350;

        //TODO: make rigging mechanism and find tick
        int Low_rigging = 0;

        double V_wrist_outside_90degree = 0.83;
        double V_wrist_clip_pickup = 0.86;
        double V_wrist_trans = 0.07;
        double V_wrist_trans_temp = 0.2;
        double V_wrist_basket = 0.76;


        int trans_status = 0;

        int chamber_status = 0;

        int pick_status = 0;

        double H_wristL_POS90 = 0.5;
        double H_wristR_POS90 = 0.5;

        double H_wristL_POS180 = 0.95;
        double H_wristR_POS180 = 0.95;

        double H_length_IN = 0.85;
        double H_length_OUT = 0.5;

        double H_angle_Ready = 0.3;
        double H_angle_pickup = 0.17;
        double H_angle_trans = 0.7;

        public H_factor(HardwareMap hardwareMap) {
            H_wristL = hardwareMap.servo.get("H_wristL");
            H_wristR = hardwareMap.servo.get("H_wristR");

            V_wristL = hardwareMap.servo.get("V_wristL");

            H_angleR = hardwareMap.servo.get("H_angleR"); // Wrist right Servo
            H_angleL = hardwareMap.servo.get("H_angleL"); // Wrist left Servo

            H_length = hardwareMap.servo.get("H_length");

            H_wristL.setDirection(Servo.Direction.REVERSE);
            H_angleL.setDirection(Servo.Direction.REVERSE);
        }

        public class H_out implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                H_length.setPosition(H_length_OUT);
                return false;
            }
        }
        public Action H_OUT() {
            return new H_out();
        }

        public class H_in implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                H_length.setPosition(H_length_IN);
                return false;
            }
        }
        public Action H_IN() {
            return new H_in();
        }

        public class H_pick implements Action {


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                    H_wristL.setPosition(H_wristL_POS90);  //trans
                    H_wristR.setPosition(H_wristR_POS90);  //trans

                    H_angleL.setPosition(H_angle_pickup);  //ready
                    H_angleR.setPosition(H_angle_pickup);  //ready

                    V_wristL.setPosition(V_wrist_trans);  //

                return false;
            }
        }

        public Action H_Pick() {
            return new H_pick();
        }

        public class H_up implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {


                H_angleL.setPosition(H_angle_trans);      //trans
                H_angleR.setPosition(H_angle_trans);      //trans

                H_wristL.setPosition(H_wristL_POS180);      //trans
                H_wristR.setPosition(H_wristR_POS180);      //trans

                return false;
            }
        }
        public Action H_UP() {
            return new H_up();
        }
    }

    public class V_factor {
        private DcMotorEx AL;
        private DcMotorEx AR;
        private Servo V_wristL;

        //setting default var

        int arm_target = 0;



        //TODO: find Horizon Griper value

        int Low_basket = 2400;
        int High_basket = 4200;
        int clip_pick = 0;

        int High_chamber = 1800;
        int High_chamber_hang = 1350;

        //TODO: make rigging mechanism and find tick
        int Low_rigging = 0;

        double V_wrist_outside_90degree = 0.83;
        double V_wrist_clip_pickup = 0.86;
        double V_wrist_trans = 0.07;
        double V_wrist_trans_temp = 0.2;
        double V_wrist_basket = 0.76;


        int trans_status = 0;

        int chamber_status = 0;

        int pick_status = 0;

        double H_wristL_POS90 = 0.5;
        double H_wristR_POS90 = 0.5;

        double H_wristL_POS180 = 0.95;
        double H_wristR_POS180 = 0.95;

        double H_length_IN = 0.85;
        double H_length_OUT = 0.5;

        double H_angle_Ready = 0.3;
        double H_angle_pickup = 0.17;
        double H_angle_trans = 0.7;


        public V_factor(HardwareMap hardwareMap) {
            AL = hardwareMap.get(DcMotorEx.class, "AL");
            AR = hardwareMap.get(DcMotorEx.class, "AR");

            AR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            AR.setDirection(DcMotorSimple.Direction.REVERSE);

            V_wristL = hardwareMap.servo.get("V_wristL");
        }

        public class V_basket implements Action {
            private boolean init = false;

            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                if (!init) {
                    AL.setPower(0.5);
                    AR.setPower(0.5);

                    init = true;
                }

                double pos = AL.getCurrentPosition();
                packet.put("AL_POS", pos);
                if (pos < 4200) {
                    return true;
                } else {
                    AL.setPower(0);
                    AR.setPower(0);
                    return false;
                }

            }
        }

        public Action V_Basket() {
            return new V_basket();
        }

        public class V_ground implements Action {
            private boolean init = false;

            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                if (!init) {
                    AL.setPower(0.5);
                    AR.setPower(0.5);

                    init = true;
                }

                double pos = AL.getCurrentPosition();
                packet.put("AL_POS", pos);
                if (pos >= 0) {
                    return true;
                } else {
                    AL.setPower(0);
                    AR.setPower(0);
                    return false;
                }

            }
        }

        public Action V_Ground() {
            return new V_ground();
        }

        public class V_chamber_high implements Action {
            private boolean init = false;

            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                if (!init) {
                    AL.setPower(0.5);
                    AR.setPower(0.5);

                    init = true;
                }

                double pos = AL.getCurrentPosition();
                packet.put("AL_POS", pos);
                if (pos >= High_chamber) {
                    return true;
                } else {
                    AL.setPower(0);
                    AR.setPower(0);
                    return false;
                }

            }
        }

        public Action V_Chamber_High() {
            return new V_chamber_high();
        }

        public class V_chamber_hang implements Action {
            private boolean init = false;

            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                if (!init) {
                    AL.setPower(0.5);
                    AR.setPower(0.5);

                    init = true;
                }

                double pos = AL.getCurrentPosition();
                packet.put("AL_POS", pos);
                if (pos >= High_chamber_hang) {
                    return true;
                } else {
                    AL.setPower(0);
                    AR.setPower(0);
                    return false;
                }

            }
        }

        public Action V_Chamber_Hang() {
            return new V_chamber_hang();
        }

        public class clip_pick implements Action {
            private boolean init = false;

            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                V_wristL.setPosition(V_wrist_clip_pickup);
                return false;

            }
        }

        public Action chamber_grip() {
            return new clip_pick();
        }

    }

    public class Grip_factor {
        private Servo H_grip;
        private Servo V_grip;


        double V_Grip_OPEN = 0.35;
        double V_Grip_CLOSE = 0.63;

        double H_Grip_OPEN = 0.55;
        double H_Grip_CLOSE = 0.85;

        public Grip_factor(HardwareMap hardwareMap) {
            H_grip = hardwareMap.servo.get("H_grip");
            V_grip = hardwareMap.servo.get("V_grip");
        }

        public class V_grip_close implements Action {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                V_grip.setPosition(V_Grip_CLOSE);
                return false;
            }
        }
        public Action V_grip_CLOSE() {
            return new V_grip_close();
        }

        public class V_grip_open implements  Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                V_grip.setPosition(V_Grip_OPEN);
                return false;
            }
        }
        public Action V_grip_OPEN() {
            return new V_grip_open();
        }

        public class H_grip_close implements  Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                H_grip.setPosition(H_Grip_CLOSE);
                return false;
            }
        }
        public Action H_grip_CLOSE() {
            return new H_grip_close();
        }

        public class H_grip_open implements  Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                H_grip.setPosition(H_Grip_OPEN);
                return false;
            }
        }
        public Action H_grip_OPEN() {
            return new H_grip_open();
        }

    }

   @Override
    public void runOpMode() throws InterruptedException {

       Pose2d initialPose = new Pose2d(-20, 61, Math.PI / 2);
       MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
       H_factor h_factor = new H_factor(hardwareMap);
       V_factor v_factor = new V_factor(hardwareMap);
       Grip_factor grip_factor = new Grip_factor(hardwareMap);

       waitForStart();

        if (isStopRequested()) return;

       Actions.runBlocking(
               drive.actionBuilder(initialPose)

                       .lineToY(35)
                       .strafeTo(new Vector2d(-35.5, 35))
                   //    .lineToY(-10)
                       /*
               .setTangent(3 * Math.PI / 2)
               .splineToLinearHeading(new Pose2d(-12, 35, Math.PI / 2), 3 * Math.PI / 2) //1st chamb
               .waitSeconds(0.5)
               .splineToSplineHeading(new Pose2d(-35.5, 36, 3 * Math.PI / 2), 3 * Math.PI / 2)
               .splineToSplineHeading(new Pose2d(-36, 8, 3 * Math.PI / 2), 3 * Math.PI / 2)
               .splineToLinearHeading(new Pose2d(-46, 11, 3 * Math.PI / 2), Math.PI / 2)
               .lineToY(50)
               .setTangent(3 * Math.PI / 2)
               .splineToConstantHeading(new Vector2d(-46, 11), 3 * Math.PI / 2)
               .splineToConstantHeading(new Vector2d(-54, 16), Math.PI / 2)
               .lineToY(50)
               .setTangent(3 * Math.PI / 2)
               .splineToConstantHeading(new Vector2d(-54, 11), 3 * Math.PI / 2)
               .splineToConstantHeading(new Vector2d(-60, 16), Math.PI / 2)
               .lineToY(40)
               .splineToSplineHeading(new Pose2d(-47, 60, 3 * Math.PI / 2), Math.PI / 2) //clip pickup
               .waitSeconds(0.5)
               .setTangent(3 * Math.PI / 2)
               .splineToSplineHeading(new Pose2d(-8, 35, Math.PI / 2), 3 * Math.PI / 2) //2nd chamber
               .waitSeconds(0.5)
               .splineToLinearHeading(new Pose2d(-47, 60, 3 * Math.PI / 2), Math.PI / 2)
               .waitSeconds(0.5)
               .splineToLinearHeading(new Pose2d(-4, 35, Math.PI / 2), 3 * Math.PI / 2) //3rd chamber
               .waitSeconds(0.5)
               .splineToLinearHeading(new Pose2d(-47, 60, 3 * Math.PI / 2), Math.PI / 2)
               .waitSeconds(0.5)
               .splineToLinearHeading(new Pose2d(0, 35, Math.PI / 2), 3 * Math.PI / 2) //4th chamber
               */
                       .build());



   }


}
