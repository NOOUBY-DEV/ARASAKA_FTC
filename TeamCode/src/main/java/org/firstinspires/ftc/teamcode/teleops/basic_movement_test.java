package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

@TeleOp(name = "basic movement test", group = "testing")
public class basic_movement_test extends LinearOpMode
{
        ArrayList<apriltag_data> apriltag_data_list = new ArrayList<>();
        private DcMotorEx motor_lf, motor_rf, motor_lb, motor_rb, shoot_motor, turret_motor;
        private Limelight3A limelight;
        private double joystick_strafe_x, joystick_strafe_y, joystick_rotate_x, shooter_trigger, turret_angle;
        private boolean is_red_team = true, turret_is_recentering = false;


        @Override
        public void runOpMode()
        {
                init_hardware();

                update_team_color(true);

                waitForStart();

                while (opModeIsActive())
                {
                        run_loop();
                }
        }

        private void init_hardware()
        {
                final int LIMELIGHT_POLLRATE = 100;

                telemetry.setMsTransmissionInterval(11);

                motor_lf = hardwareMap.get(DcMotorEx.class, "motor_lf");
                motor_rf = hardwareMap.get(DcMotorEx.class, "motor_rf");
                motor_lb = hardwareMap.get(DcMotorEx.class, "motor_lb");
                motor_rb = hardwareMap.get(DcMotorEx.class, "motor_rb");
                shoot_motor = hardwareMap.get(DcMotorEx.class, "shoot_motor");
                turret_motor = hardwareMap.get(DcMotorEx.class, "turret_motor");

                motor_lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor_rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor_lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor_rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                motor_lf.setDirection(DcMotorSimple.Direction.REVERSE);
                motor_lb.setDirection(DcMotorSimple.Direction.REVERSE);

                limelight = hardwareMap.get(Limelight3A.class, "limelight");
                limelight.setPollRateHz(LIMELIGHT_POLLRATE);
                limelight.pipelineSwitch(0);
                limelight.start();

                telemetry.addData("waiting for limelight...", "");
                while (!limelight.isConnected());
                telemetry.clearAll();

                turret_angle = 0;
        }

        private void run_loop()
        {
                update_team_color(false);
                update_gamepad_values();
                update_robot_movement();
                update_shoot_motor();
                update_telemetry_display();
        }

        private void update_gamepad_values()
        {
                joystick_strafe_x = gamepad1.left_stick_x;
                joystick_strafe_y = gamepad1.left_stick_y;
                joystick_rotate_x = gamepad1.right_stick_x;
                shooter_trigger = gamepad1.right_trigger;
        }

        private void update_team_color(boolean to_wait)
        {
                if (!to_wait)
                {
                        if (!gamepad1.back) return;

                        if (gamepad1.right_bumper)
                        {
                                is_red_team = true;

                        } else if (gamepad1.left_bumper)
                        {
                                is_red_team = false;
                        }

                        return;
                }

                while (!opModeIsActive() && !isStopRequested())
                {
                        if (gamepad1.right_bumper)
                        {
                                is_red_team = true;

                        } else if (gamepad1.left_bumper)
                        {
                                is_red_team = false;
                        }
                }
        }

        private void update_robot_movement()
        {
                final double rotate_speed = 0.8;

                double x = joystick_strafe_x;
                double y = -joystick_strafe_y;

                // Translation (polar trig)
                double speed = Math.min(1.0, Math.hypot(x, y));
                double angle = Math.atan2(y, x) - (Math.PI / 4.0);

                // Rotation
                double turn = joystick_rotate_x * rotate_speed;

                double base_lf_rb = speed * Math.cos(angle) * Math.sqrt(2.0);
                double base_rf_lb = speed * Math.sin(angle) * Math.sqrt(2.0);

                double lf = base_lf_rb + turn;
                double rf = base_rf_lb - turn;
                double lb = base_rf_lb + turn;
                double rb = base_lf_rb - turn;

                double max = Math.max(
                        Math.max(Math.abs(lf), Math.abs(rf)),
                        Math.max(Math.abs(lb), Math.abs(rb))
                );

                if (max > 1.0)
                {
                        lf /= max;
                        rf /= max;
                        lb /= max;
                        rb /= max;
                }

                motor_lf.setPower(lf);
                motor_rf.setPower(rf);
                motor_lb.setPower(lb);
                motor_rb.setPower(rb);
        }

        private void update_shoot_motor() // not tested yet
        {
                final double activation_point = 0.3;

                if (shooter_trigger >= activation_point)
                {
                        shoot_motor.setPower(shooter_trigger);
                }
                else
                {
                        shoot_motor.setPower(0);
                }
        }

        private void update_limelight_vision() // not tested yet
        {
                final boolean DEBUG = true;

                apriltag_data_list.clear();

                LLResult limelight_result = limelight.getLatestResult();

                if (limelight_result == null || !limelight_result.isValid())
                {
                        return;
                }

                List<LLResultTypes.FiducialResult> fiducials = limelight_result.getFiducialResults();

                if (fiducials == null || fiducials.isEmpty())
                {
                        return;
                }

                if (DEBUG) telemetry.addData("apriltags found :- ", fiducials.size());

                for (LLResultTypes.FiducialResult current_apriltag : fiducials)
                {
                        apriltag_data new_apriltag_data = new apriltag_data(
                                current_apriltag.getFiducialId(),
                                current_apriltag.getTargetXDegrees(),
                                current_apriltag.getTargetYDegrees(),
                                true
                        );

                        final Boolean apriltag_is_red_team = new_apriltag_data.is_red_team();

                        if (apriltag_is_red_team == null || is_red_team != new_apriltag_data.is_red_team()) continue;

                        apriltag_data_list.add(new_apriltag_data);

                        if (DEBUG) telemetry.addData(
                                "tag_id :- " + new_apriltag_data.id,
                                "tx: %.1f° | ty: %.1f°",
                                new_apriltag_data.tx, new_apriltag_data.ty
                        );
                }

                if (!apriltag_data_list.isEmpty())
                {
                        apriltag_data_list.sort(Comparator.comparingDouble(apriltag_data::get_abs_tx));
                }
        }

        private void update_turrent_angle() // not finished nor tested
        {
                // has to do with ticks
        }

        private void update_turret_position() // not finished nor tested
        {
                final double MAX_TURRET_ROTATE_SPEED = 0.7;
                final double MAX_TURRET_ANGLE = 45;
                final double SEARCH_ROTATE_SPEED = 0.5;
                final double MAX_ANGLE_ERROR_DEG = 1;

                final boolean no_apriltags = apriltag_data_list.isEmpty();
                final double absolute_turret_angle = Math.abs(turret_angle);


                if (no_apriltags && absolute_turret_angle >= MAX_TURRET_ANGLE)
                {
                        turret_is_recentering = true;
                }

                if (turret_is_recentering)
                {
                        if (!no_apriltags || absolute_turret_angle <= MAX_ANGLE_ERROR_DEG)
                        {
                                turret_motor.setPower(0);

                                turret_is_recentering = false;

                                return;
                        }

                        final double rotate_power = - (turret_angle / MAX_TURRET_ANGLE) * SEARCH_ROTATE_SPEED;

                        turret_motor.setPower(rotate_power);

                        return;
                }

                if (no_apriltags)
                {
                        return;
                }

                // [turn to apriltag]
                {
                        final apriltag_data current_apriltag = apriltag_data_list.get(0);

                        final double final_tx = Math.signum(current_apriltag.tx) * Math.min(Math.abs(current_apriltag.tx), MAX_TURRET_ANGLE);

                        final double turn_power = - (final_tx / MAX_TURRET_ANGLE) * MAX_TURRET_ROTATE_SPEED;

                        if ( Math.abs(turret_angle) >= MAX_TURRET_ANGLE &&
                                Math.signum(turret_angle) == Math.signum(final_tx)
                        )
                        {
                                turret_motor.setPower(0);

                                return;
                        }

                        turret_motor.setPower(turn_power);
                }
        }

        private void update_telemetry_display()
        {
                final int shoot_motor_speed_percent = (int) (shoot_motor.getPower() * 100);
                final String team_color = is_red_team ? "red" : "blue";

                telemetry.addData("team_color :- ", team_color);
                telemetry.addData("joystick_strafe.x :- ", joystick_strafe_x);
                telemetry.addData("joystick_strafe.y :- ", joystick_strafe_y);
                telemetry.addData("joystick_rotate.x :- ", joystick_rotate_x);
                telemetry.addData("shooter_motor_usage_speed (%) :- ", shoot_motor_speed_percent);

                telemetry.update();
        }
}
