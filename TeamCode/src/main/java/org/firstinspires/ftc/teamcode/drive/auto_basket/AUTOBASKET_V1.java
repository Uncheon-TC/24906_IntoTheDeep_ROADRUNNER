package org.firstinspires.ftc.teamcode.drive.auto_basket;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Non-RR imports
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "AUTObasket_v1", group = "Autonomous")

public class AUTOBASKET_V1 extends LinearOpMode {

    private PIDController controller;

    public static double p = 0.02, i = 0, d = 0.0005;

    public static double f = 0.001;
    public static int target = 0;

    private DcMotorEx AL;
    private DcMotorEx AR;

    private Servo V_wristL;
    private Servo V_wristR;
    private Servo H_lengthL;
    private Servo H_lengthR;
    private Servo H_wristL;
    private Servo H_wristR;
    private Servo H_angleL;
    private Servo H_angleR;
    private Servo V_angleL;
    private Servo V_angleR;

    private Servo H_grip;
    private Servo V_grip;
    private final double ticks_in_degree = 700 / 180.0;

    public class H_factor {


        //setting default var
        int target = 0;

        boolean first_count = false;

        double V_Grip_OPEN = 0.35;
        double V_Grip_CLOSE = 0.63;

        double H_Grip_OPEN = 0.55;
        double H_Grip_CLOSE = 0.85;

        boolean HG_OPEN = true;
        boolean VG_OPEN = true;

        //TODO: find Horizon Griper value

        int clip_pick = 0;
        int High_backet = 2450;

        int High_chamber_hang = 640;


        //TODO: make rigging mechanism and find tick


        double V_wrist_L_pick = 0.5;
        double V_wrist_R_pick = 0.5;
        double V_wrist_L_hang = 0.97;
        double V_wrist_R_hang = 0.34;
        double V_wrist_L_trans = 0.98;
        double V_wrist_R_trans =1;
        double V_wrist_L_backet = 0.53;
        double V_wrist_R_backet = 0.55;

        double V_angle_pick = 0.16;
        double V_angle_up = 0.2;
        double V_angle_hang = 0.82;
        double V_angle_hang_down = 0.85;
        double V_angle_backet =0.72;
        double V_angle_trans_ready =0.54;
        double V_angle_trans = 0.48;



        int chamber_status = 0;



        double H_wristL_Ready = 0.45+0.03;
        double H_wristR_Ready = 0.45+0.03;

        double H_wrist_L_hide = 0.5;
        double H_wrist_R_hide = 0.5;
        double H_wrist_L_trans = 0.98;
        double H_wrist_R_trans = 1;

        double H_angleL_back = 0.47;
        double H_angleR_back = 0.47;


        double H_wristL_pickup = 0.5;
        double H_wristR_pickup = 0.5;
        double H_length_IN = 0.4;
        //double H_length_L_IN = 0.62;
        //double H_length_R_IN = 0.60;
        double H_length_OUT = 0.13;
        //double H_length_L_OUT = 0.32;
        //double H_length_R_OUT = 0.30;
        double H_length_L_trans = 0.60;
        double H_length_R_trans = 0.58;

        double H_angle_Ready = 0.50;
        double H_angle_pickup = 0.68;
        double H_angle_hide = 0.76;
        double H_angle_trans = 0.41;

        public H_factor(HardwareMap hardwareMap) {
            V_wristL = hardwareMap.servo.get("V_wristL");
            V_wristR = hardwareMap.servo.get("V_wristR");//Bucket Wrist left Servo
            V_angleL = hardwareMap.servo.get("V_angleL");
            V_angleR = hardwareMap.servo.get("V_angleR");
            H_angleR = hardwareMap.servo.get("H_angleR");
            H_lengthL = hardwareMap.servo.get("H_lengthL");//Slide right Servo
            H_lengthR = hardwareMap.servo.get("H_lengthR");
            H_wristR = hardwareMap.servo.get("H_wristR"); // Ground Gripper right Servo
            H_wristL = hardwareMap.servo.get("H_wristL"); // Ground Gripper Left Servo
            H_angleR = hardwareMap.servo.get("H_angleR"); // Wrist right Servo
            H_angleL = hardwareMap.servo.get("H_angleL"); // Wrist left Servo

            H_wristL.setDirection(Servo.Direction.REVERSE);
            V_wristL.setDirection(Servo.Direction.REVERSE);

            H_lengthL.setDirection(Servo.Direction.REVERSE);

            H_angleL.setDirection(Servo.Direction.REVERSE);
            V_angleL.setDirection(Servo.Direction.REVERSE);
        }

        public class H_out implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                H_lengthL.setPosition(H_length_OUT);
                H_lengthR.setPosition(H_length_OUT);
                H_wristL.setPosition(H_wristL_Ready);
                H_wristR.setPosition(H_wristR_Ready);
                H_angleL.setPosition(H_angle_pickup-0.02);
                H_angleR.setPosition(H_angle_pickup-0.02);
                return false;
            }
        }


        public Action H_OUT() {
            return new H_out();
        }

        public class H_spin implements Action {
            private long startTime = -1;
            private final long durationMs = 500; // 0.5초 유지

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startTime < 0) startTime = System.currentTimeMillis();

                H_lengthL.setPosition(H_length_OUT);
                H_lengthR.setPosition(H_length_OUT);
                H_wristL.setPosition(H_wristL_Ready - 0.075);
                H_wristR.setPosition(H_wristR_Ready + 0.075);
                H_angleL.setPosition(H_angle_pickup);
                H_angleR.setPosition(H_angle_pickup);

                return System.currentTimeMillis() - startTime < durationMs;
            }
        }

        public Action H_Spin() {
            return new H_spin();
        }
        public class H_down implements Action {
            private long startTime = -1;
            private final long durationMs = 500; // 0.5초 유지

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startTime < 0) startTime = System.currentTimeMillis();

                H_lengthL.setPosition(H_length_OUT);
                H_lengthR.setPosition(H_length_OUT);
                H_angleL.setPosition(H_angle_pickup + 0.08);
                H_angleR.setPosition(H_angle_pickup + 0.08);
                H_wristL.setPosition(H_wristL_Ready+0.04);
                H_wristR.setPosition(H_wristR_Ready+0.04);

                return System.currentTimeMillis() - startTime < durationMs;
            }
        }

        public Action H_Down() {
            return new H_down();
        }



        public class H_in implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                H_lengthL.setPosition(H_length_IN);
                H_lengthR.setPosition(H_length_IN);
                return false;
            }
        }

        public Action H_IN() {
            return new H_in();
        }

        public class H_pick implements Action {


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                H_angleL.setPosition(H_angle_pickup);
                H_angleR.setPosition(H_angle_pickup);
                H_wristL.setPosition(H_wristL_Ready);
                H_wristR.setPosition(H_wristR_Ready);
                H_lengthL.setPosition(H_length_IN);
                H_lengthR.setPosition(H_length_IN);


                return false;
            }
        }

        public Action H_Pick() {
            return new H_pick();
        }

        public class H_up implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {


                H_angleL.setPosition(H_angle_trans);
                H_angleR.setPosition(H_angle_trans);

                H_wristL.setPosition(H_wrist_L_trans);
                H_wristR.setPosition(H_wrist_R_trans);

                H_lengthL.setPosition(H_length_IN);
                H_lengthR.setPosition(H_length_IN);

                V_angleL.setPosition(V_angle_trans);
                V_angleR.setPosition(V_angle_trans);

                V_wristL.setPosition(V_wrist_L_trans);
                V_wristR.setPosition(V_wrist_R_trans);


                return false;
            }
        }

        public Action H_UP() {
            return new H_up();
        }

        public class HoldHAction implements Action {
            private final Servo H_angleL, H_angleR, H_wristL, H_wristR, H_lengthL, H_lengthR;

            private  double anglePos;
            private  double wristPos;
            private  double lengthPos;
            private final long durationMs;
            private long startTime = -1;

            public HoldHAction(
                    Servo H_angleL, Servo H_angleR,
                    Servo H_wristL, Servo H_wristR,
                    Servo H_lengthL, Servo H_lengthR,
                    double anglePos, double wristPos, double lengthPos,
                    double durationSeconds) {

                this.H_angleL = H_angleL;
                this.H_angleR = H_angleR;
                this.H_wristL = H_wristL;
                this.H_wristR = H_wristR;
                this.H_lengthL = H_lengthL;
                this.H_lengthR = H_lengthR;

                this.anglePos = anglePos;
                this.wristPos = wristPos;
                this.lengthPos = lengthPos;
                this.durationMs = (long) (durationSeconds * 1000);
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startTime < 0) startTime = System.currentTimeMillis();

                H_angleL.setPosition(anglePos);
                H_angleR.setPosition(anglePos);
                H_wristL.setPosition(wristPos);
                H_wristR.setPosition(wristPos);
                H_lengthL.setPosition(lengthPos);
                H_lengthR.setPosition(lengthPos);

                return System.currentTimeMillis() - startTime < durationMs;
            }
        }
        public Action holdHPosition(double anglePos, double wristPos, double lengthPos, double durationSeconds) {
            return new HoldHAction(
                    H_angleL, H_angleR,
                    H_wristL, H_wristR,
                    H_lengthL, H_lengthR,
                    anglePos, wristPos, lengthPos,
                    durationSeconds
            );
        }

    }

    public class V_factor {

        //setting default var

        int target = 0;


        //TODO: find Horizon Griper value

        int clip_pick = 0;
        int High_backet = 2470;
        int parking =960;

        int High_chamber_hang = 800;


        //TODO: make rigging mechanism and find tick

        double V_Grip_OPEN = 0.43;
        double V_Grip_CLOSE = 0.65;

        double H_Grip_OPEN = 0.47;
        double H_Grip_CLOSE = 0.25;


        double V_wrist_L_pick = 0.5;
        double V_wrist_R_pick = 0.5;
        double V_wrist_L_hang = 0.9;
        double V_wrist_R_hang = 0.34;
        double V_wrist_L_trans = 0.97;
        double V_wrist_R_trans = 0.97;
        double V_wrist_L_backet = 0.53;
        double V_wrist_R_backet = 0.55;

        double V_angle_pick = 0.16;
        double V_angle_up = 0.77;
        double V_angle_hang = 0.82;
        double V_angle_hang_down = 0.14;
        double V_angle_backet = 0.73;
        double V_angle_trans_ready = 0.54;
        double V_angle_trans = 0.465;


        int chamber_status = 0;


        double H_wristL_Ready = 0.45;
        double H_wristR_Ready = 0.45;

        double H_wrist_L_hide = 0.5;
        double H_wrist_R_hide = 0.5;
        double H_wrist_L_trans = 0.98;
        double H_wrist_R_trans = 1;

        double H_angleL_back = 0.47;
        double H_angleR_back = 0.47;
        double H_length_IN = 0.4;
        double H_length_L_OUT = 0.32;
        double H_length_R_OUT = 0.30;
        double H_length_L_trans = 0.60;
        double H_length_R_trans = 0.58;

        double H_angle_Ready = 0.62;
        double H_angle_pickup = 0.69;
        double H_angle_hide = 0.76;
        double H_angle_trans = 0.46;


        public V_factor(HardwareMap hardwareMap) {
            AL = hardwareMap.get(DcMotorEx.class, "AL");
            AR = hardwareMap.get(DcMotorEx.class, "AR");

            AR.setDirection(DcMotorSimple.Direction.REVERSE);

            AL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            AR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            V_wristL = hardwareMap.servo.get("V_wristL");
            V_wristR = hardwareMap.servo.get("V_wristR");//Bucket Wrist left Servo
            V_angleL = hardwareMap.servo.get("V_angleL");
            V_angleR = hardwareMap.servo.get("V_angleR");
            H_angleR = hardwareMap.servo.get("H_angleR");
            H_lengthL = hardwareMap.servo.get("H_lengthL");//Slide right Servo
            H_lengthR = hardwareMap.servo.get("H_lengthR");
            H_wristR = hardwareMap.servo.get("H_wristR"); // Ground Gripper right Servo
            H_wristL = hardwareMap.servo.get("H_wristL"); // Ground Gripper Left Servo
            H_angleR = hardwareMap.servo.get("H_angleR"); // Wrist right Servo
            H_angleL = hardwareMap.servo.get("H_angleL"); // Wrist left Servo

            H_wristL.setDirection(Servo.Direction.REVERSE);
            V_wristL.setDirection(Servo.Direction.REVERSE);

            H_lengthL.setDirection(Servo.Direction.REVERSE);

            H_angleL.setDirection(Servo.Direction.REVERSE);
            V_angleL.setDirection(Servo.Direction.REVERSE);
        }


        public class V_basket implements Action {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    AL.setTargetPosition(High_backet);
                    AR.setTargetPosition(High_backet);

                    AL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    AR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // 목표 위치로 이동할 때의 전력 설정
                    AL.setPower(1);
                    AR.setPower(1);
                    // 서보 모터 동시 동작 추가


                    init = true;
                }

                target = High_backet;
                double pos = AL.getCurrentPosition();
                packet.put("AL_POS", pos);

                if (pos < High_backet) {
                    return true;
                } else {
                    // 현재 위치에 모터 고정
                    AL.setTargetPosition((int) pos);
                    AR.setTargetPosition((int) pos);

                    AL.setPower(0.2);
                    AR.setPower(0.2);

                    return false;
                }

            }
        }

        public Action V_Basket() {
            return new V_basket();
        }
        public class V_parking implements Action {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    AL.setTargetPosition(parking);
                    AR.setTargetPosition(parking);

                    AL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    AR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // 목표 위치로 이동할 때의 전력 설정
                    AL.setPower(1);
                    AR.setPower(1);
                    // 서보 모터 동시 동작 추가
                    V_angleL.setPosition(0.72);
                    V_angleR.setPosition(0.72);
                    V_wristL.setPosition(0.45);
                    V_wristR.setPosition(0.45);


                    init = true;
                }

                target = parking;
                double pos = AL.getCurrentPosition();
                packet.put("AL_POS", pos);

                if (pos < parking) {
                    return true;
                } else {
                    // 현재 위치에 모터 고정
                    AL.setTargetPosition((int) pos);
                    AR.setTargetPosition((int) pos);

                    AL.setPower(0.2);
                    AR.setPower(0.2);

                    return false;
                }

            }
        }

        public Action V_Parking() {
            return new V_parking();
        }

        public class V_angle implements Action {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    AL.setTargetPosition(High_backet);
                    AR.setTargetPosition(High_backet);

                    AL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    AR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // 목표 위치로 이동할 때의 전력 설정
                    AL.setPower(1);
                    AR.setPower(1);
                    // 서보 모터 동시 동작 추가
                    V_angleL.setPosition(0.73);
                    V_angleR.setPosition(0.73);
                    V_wristL.setPosition(0.53);
                    V_wristR.setPosition(0.55);
                    init = true;


                    init = true;
                }

                target = High_backet;
                double pos = AL.getCurrentPosition();
                packet.put("AL_POS", pos);

                if (pos < High_backet) {
                    return true;
                } else {
                    // 현재 위치에 모터 고정
                    AL.setTargetPosition((int) pos);
                    AR.setTargetPosition((int) pos);

                    AL.setPower(0.2);
                    AR.setPower(0.2);

                    return false;
                }

            }
        }

        public Action V_Angle() {
            return new V_angle();
        }



        public class V_ground implements Action {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    AL.setTargetPosition(clip_pick);
                    AR.setTargetPosition(clip_pick);

                    AL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    AR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // 목표 위치로 이동할 때의 전력 설정
                    AL.setPower(1);
                    AR.setPower(1);
                    // 서보 모터 동시 동작 추가

                    V_angleL.setPosition(V_angle_trans);
                    V_angleR.setPosition(V_angle_trans);

                    V_wristL.setPosition(V_wrist_L_trans);
                    V_wristR.setPosition(V_wrist_R_trans);
                    init = true;
                }

                target = clip_pick;
                double pos = AL.getCurrentPosition();
                packet.put("AL_POS", pos);

                if (pos > clip_pick) {
                    return true;
                } else {
                    // 현재 위치에 모터 고정
                    AL.setTargetPosition((int) pos);
                    AR.setTargetPosition((int) pos);

                    AL.setPower(0.2);
                    AR.setPower(0.2);

                    return false;
                }
            }
        }

        public Action V_Ground() {
            return new V_ground();
        }

        public class V_chamber_basket2 implements Action {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                {
                    if (!init) {
                        AL.setTargetPosition(High_backet);
                        AR.setTargetPosition(High_backet);

                        AL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        AR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        // 목표 위치로 이동할 때의 전력 설정
                        AL.setPower(1);
                        AR.setPower(1);
                        // 서보 모터 동시 동작 추가
                        V_angleL.setPosition(V_angle_backet);
                        V_angleR.setPosition(V_angle_backet);
                        V_wristL.setPosition(V_wrist_L_backet);
                        V_wristR.setPosition(V_wrist_R_backet);
                        H_grip.setPosition(H_Grip_OPEN);
                        init = true;
                    }

                    target = High_backet;
                    double pos = AL.getCurrentPosition();
                    packet.put("AL_POS", pos);

                    if (pos > High_backet) {
                        return true;
                    } else {
                        // 현재 위치에 모터 고정
                        AL.setTargetPosition((int) pos);
                        AR.setTargetPosition((int) pos);

                        AL.setPower(0.2);
                        AR.setPower(0.2);

                        return false;
                    }
                }
            }

            public Action V_Chamber_Basket2() {
                return new V_chamber_basket2();
            }

            /*        public class V_chamber_hang implements Action {
                        private boolean init = false;

                        @Override
                        public boolean run (@NonNull TelemetryPacket packet) {
                            if (!init) {
                                AL.setPower(1);
                                AR.setPower(1);
                                init = true;
                            }

                            target = High_chamber_hang;




                            double pos = AL.getCurrentPosition();
                            packet.put("AL_POS", pos);
                            if (pos < High_chamber_hang) {
                                return true;
                            } else {
                                AL.setPower(0);
                                AR.setPower(0);
                                return false;
                            }

                        }
                    }*/

            public class clip_pick implements Action {
                private boolean init = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    H_lengthL.setPosition(H_length_IN);
                    H_lengthR.setPosition(H_length_IN);
                    H_angleL.setPosition(H_angle_hide);
                    H_angleR.setPosition(H_angle_hide);
                    H_wristL.setPosition(H_wrist_L_hide);
                    H_wristR.setPosition(H_wrist_R_hide);
                    V_angleL.setPosition(V_angle_pick);
                    V_angleR.setPosition(V_angle_pick);
                    V_wristL.setPosition(V_wrist_L_pick);
                    V_wristR.setPosition(V_wrist_R_pick);
                    return false;

                }
            }

            public Action Clip_PICK() {
                return new clip_pick();
            }


            public class chamber_hang implements Action {
                private boolean init = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    V_angleL.setPosition(V_angle_hang);
                    V_angleR.setPosition(V_angle_hang);
                    V_wristL.setPosition(V_wrist_L_hang);
                    V_wristR.setPosition(V_wrist_R_hang);
                    return false;

                }
            }

            public Action chamber() {
                return new chamber_hang();
            }


        }
        public class HoldPowerAction implements Action {
            private final DcMotorEx AL, AR;
            private final Servo V_angleL, V_angleR, V_wristL, V_wristR;

            private final double power;
            private final long durationMs;
            private long startTime = -1;

            private double vangle;
            private double vwrist;

            public HoldPowerAction(
                    DcMotorEx AL, DcMotorEx AR,
                    Servo V_angleL, Servo V_angleR,
                    Servo V_wristL, Servo V_wristR,
                    double power, double durationSeconds,
                    double vangle, double vwrist) {
                this.AL = AL;
                this.AR = AR;
                this.V_angleL = V_angleL;
                this.V_angleR = V_angleR;
                this.V_wristL = V_wristL;
                this.V_wristR = V_wristR;

                this.power = power;
                this.durationMs = (long) (durationSeconds * 1000);
                this.vangle = vangle;
                this.vwrist = vwrist;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startTime < 0) startTime = System.currentTimeMillis();

                AL.setPower(power);
                AR.setPower(power);
                V_angleL.setPosition(vangle);
                V_angleR.setPosition(vangle);
                V_wristL.setPosition(vwrist);
                V_wristR.setPosition(vwrist);

                return System.currentTimeMillis() - startTime < durationMs;
            }
        }
            public Action holdLiftPower(double power, double durationSeconds, double vangle, double vwrist) {
                return new HoldPowerAction(AL, AR, V_angleL, V_angleR, V_wristL, V_wristR, power, durationSeconds, vangle, vwrist);
        }



    }

    public class Grip_factor {
        private Servo H_grip;
        private Servo V_grip;
        private Servo H_wristL;
        private Servo H_wristR;
        private Servo H_angleL;
        private Servo H_angleR;


        double V_Grip_OPEN = 0.43;
        double V_Grip_CLOSE = 0.65;

        double H_Grip_OPEN = 0.47;
        double H_Grip_CLOSE = 0.25;
        double H_wristL_Ready = 0.45+0.03;
        double H_wristR_Ready = 0.45+0.03;
        double H_angle_Ready = 0.50;
        double H_angle_pickup = 0.68;

        public Grip_factor(HardwareMap hardwareMap) {
            H_grip = hardwareMap.servo.get("H_grip");
            V_grip = hardwareMap.servo.get("V_grip");
            H_wristR = hardwareMap.servo.get("H_wristR"); // Ground Gripper right Servo
            H_wristL = hardwareMap.servo.get("H_wristL"); // Ground Gripper Left Servo
            H_angleR = hardwareMap.servo.get("H_angleR"); // Wrist right Servo
            H_angleL = hardwareMap.servo.get("H_angleL"); // Wrist left Servo
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

        public class V_grip_open implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                V_grip.setPosition(V_Grip_OPEN);
                return false;
            }
        }

        public Action V_grip_OPEN() {
            return new V_grip_open();
        }

        public class H_grip_close implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                H_grip.setPosition(H_Grip_CLOSE);
                return false;
            }
        }

        public Action H_grip_CLOSE() {
            return new H_grip_close();
        }

        public class H_grip_open implements Action {

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

        Pose2d initialPose = new Pose2d(31, 65, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        H_factor h_factor = new H_factor(hardwareMap);
        V_factor v_factor = new V_factor(hardwareMap);
        Grip_factor grip_factor = new Grip_factor(hardwareMap);

        controller = new PIDController(p, i, d);

        AL = hardwareMap.get(DcMotorEx.class, "AL");
        AR = hardwareMap.get(DcMotorEx.class, "AR");
        AR.setDirection(DcMotorSimple.Direction.REVERSE);


        TrajectoryActionBuilder traj = drive.actionBuilder(initialPose)

                .afterTime(0, grip_factor.V_grip_CLOSE())
                .afterTime(0, grip_factor.H_grip_OPEN())
                .afterTime(0.0, v_factor.V_Basket())
                .afterTime(0.75, v_factor.V_Angle())
                .setTangent(Math.toRadians(330))
                .splineToLinearHeading(new Pose2d(53,57.8, Math.toRadians(225)),Math.toRadians(330))
                .stopAndAdd(() -> Actions.runBlocking(v_factor.holdLiftPower(0.1, 0.3, 0.72, 0.53)))
                .stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_OPEN()))
                .stopAndAdd(() -> Actions.runBlocking(h_factor.holdHPosition(0.65,0.48,0.13,0.3)))
                .afterTime(0.5, v_factor.V_Ground())
                .stopAndAdd(() -> Actions.runBlocking(h_factor.holdHPosition(0.65,0.48,0.13,0.3)))
                .afterTime(0.6, h_factor.H_OUT())
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(49,49.5,Math.toRadians(270)),Math.toRadians(270) )
                .stopAndAdd(() -> Actions.runBlocking(h_factor.holdHPosition(0.65,0.48,0.13,0.3)))
                .stopAndAdd(() -> Actions.runBlocking(h_factor.holdHPosition(0.74,0.54,0.13,0.3)))
                .stopAndAdd(() -> Actions.runBlocking(grip_factor.H_grip_CLOSE()))
                .stopAndAdd(() -> Actions.runBlocking(h_factor.holdHPosition(0.74,0.54,0.13,0.3)))
                .stopAndAdd(() -> Actions.runBlocking(h_factor.H_UP()))
                .stopAndAdd(() -> Actions.runBlocking(h_factor.holdHPosition(0.48,0.99,0.4,0.3)))
                .stopAndAdd(() -> Actions.runBlocking(v_factor.holdLiftPower(0.1,0.5,0.45,0.97)))
                .stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_CLOSE()))
                .stopAndAdd(() -> Actions.runBlocking(h_factor.holdHPosition(0.48,0.99,0.4,0.3)))
                .stopAndAdd(() -> Actions.runBlocking(grip_factor.H_grip_OPEN()))
                .stopAndAdd(() -> Actions.runBlocking(v_factor.V_Basket()))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(53.5,58.3, Math.toRadians(225)),Math.toRadians(45))
                .afterTime(0.75, v_factor.V_Angle())
                .stopAndAdd(() -> Actions.runBlocking(v_factor.holdLiftPower(0.1, 0.3, 0.72, 0.53)))
                .stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_OPEN()))
                .afterTime(0.5, v_factor.V_Ground())
                .afterTime(0.6, h_factor.H_OUT())
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(59,49.5,Math.toRadians(271)),Math.toRadians(270) )
                .stopAndAdd(() -> Actions.runBlocking(h_factor.holdHPosition(0.65,0.48,0.13,0.3)))
                .stopAndAdd(() -> Actions.runBlocking(h_factor.holdHPosition(0.74,0.54,0.13,0.3)))
                .stopAndAdd(() -> Actions.runBlocking(grip_factor.H_grip_CLOSE()))
                .stopAndAdd(() -> Actions.runBlocking(h_factor.holdHPosition(0.74,0.54,0.13,0.3)))
                .stopAndAdd(() -> Actions.runBlocking(h_factor.H_UP()))
                .stopAndAdd(() -> Actions.runBlocking(h_factor.holdHPosition(0.48,0.99,0.4,0.3)))
                .stopAndAdd(() -> Actions.runBlocking(v_factor.holdLiftPower(0.1,0.5,0.45,0.97)))
                .stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_CLOSE()))
                .stopAndAdd(() -> Actions.runBlocking(h_factor.holdHPosition(0.48,0.99,0.4,0.3)))
                .stopAndAdd(() -> Actions.runBlocking(grip_factor.H_grip_OPEN()))
                .stopAndAdd(() -> Actions.runBlocking(v_factor.V_Basket()))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(53.5,58.3, Math.toRadians(225)),Math.toRadians(90))
                .afterTime(0.75, v_factor.V_Angle())
                .stopAndAdd(() -> Actions.runBlocking(v_factor.holdLiftPower(0.1, 0.3, 0.72, 0.53)))
                .stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_OPEN()))
                .afterTime(0.5, v_factor.V_Ground())
                .afterTime(0.6, h_factor.H_OUT())
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(55.5,45,Math.toRadians(313)),Math.toRadians(270) )
                .stopAndAdd(() -> Actions.runBlocking(h_factor.holdHPosition(0.65,0.48,0.13,0.3)))
                .stopAndAdd(() -> Actions.runBlocking(h_factor.holdHPosition(0.74,0.54,0.13,0.3)))
                .stopAndAdd(() -> Actions.runBlocking(h_factor.H_Spin()))
                .stopAndAdd(() -> Actions.runBlocking(grip_factor.H_grip_CLOSE()))
                .stopAndAdd(() -> Actions.runBlocking(h_factor.holdHPosition(0.74,0.54,0.13,0.3)))
                .stopAndAdd(() -> Actions.runBlocking(h_factor.H_UP()))
                .stopAndAdd(() -> Actions.runBlocking(h_factor.holdHPosition(0.48,0.99,0.4,0.3)))
                .stopAndAdd(() -> Actions.runBlocking(v_factor.holdLiftPower(0.1,0.5,0.45,0.97)))
                .stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_CLOSE()))
                .stopAndAdd(() -> Actions.runBlocking(h_factor.holdHPosition(0.48,0.99,0.4,0.3)))
                .stopAndAdd(() -> Actions.runBlocking(grip_factor.H_grip_OPEN()))
                .stopAndAdd(() -> Actions.runBlocking(v_factor.V_Basket()))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(53.5,58.3, Math.toRadians(225)),Math.toRadians(45))
                .afterTime(0.75, v_factor.V_Angle())
                .stopAndAdd(() -> Actions.runBlocking(v_factor.holdLiftPower(0.1, 0.5, 0.72, 0.53)))
                .stopAndAdd(() -> Actions.runBlocking(grip_factor.V_grip_OPEN()))
                .afterTime(0.5, v_factor.V_Ground())
                .stopAndAdd(() -> Actions.runBlocking(h_factor.holdHPosition(0.68,0.48,0.4,0.2)))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(53,10, Math.toRadians(0)),Math.toRadians(270))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(17,10, Math.toRadians(0)),Math.toRadians(180))
                .stopAndAdd(() -> Actions.runBlocking(v_factor.V_Parking()))
                .stopAndAdd(() -> Actions.runBlocking(v_factor.holdLiftPower(0.1, 8, 0.72, 0.45)));










        //.stopAndAdd(() -> Actions.runBlocking(h_factor.H_Pick()))
                //.stopAndAdd(() -> Actions.runBlocking(grip_factor.H_grip_CLOSE()));

        Action traj_END = traj.endTrajectory().fresh()
                .build();


        Actions.runBlocking(grip_factor.H_grip_CLOSE());
        Actions.runBlocking(grip_factor.V_grip_CLOSE());
        Actions.runBlocking(h_factor.H_IN());


        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        traj.build(),


                        traj_END
                )
        );

        while (opModeIsActive()) {
            controller.setPID(p, i, d);
            int armPos = AL.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;

            AL.setPower(power);
            AR.setPower(power);


            telemetry.addData("armpower ", power);
            telemetry.addData("armtarget ", target);
            telemetry.update();

            if (isStopRequested()) return;
        }


    }


}

