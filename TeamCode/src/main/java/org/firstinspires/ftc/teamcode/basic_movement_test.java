package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "basic movement test", group = "testing")
public class basic_movement_test extends LinearOpMode
{
        private DcMotorEx motor_lf, motor_rf, motor_lb, motor_rb, shoot_motor;
        private double joystick_strafe_x, joystick_strafe_y, joystick_rotate_x, shooter_trigger;
        private boolean is_red_team = true;


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
                motor_lf = hardwareMap.get(DcMotorEx.class, "motor_lf");
                motor_rf = hardwareMap.get(DcMotorEx.class, "motor_rf");
                motor_lb = hardwareMap.get(DcMotorEx.class, "motor_lb");
                motor_rb = hardwareMap.get(DcMotorEx.class, "motor_rb");
                shoot_motor = hardwareMap.get(DcMotorEx.class, "shoot_motor");

                motor_lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor_rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor_lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor_rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                motor_lf.setDirection(DcMotorSimple.Direction.REVERSE);
                motor_lb.setDirection(DcMotorSimple.Direction.REVERSE);
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

        private void update_telemetry_display()
        {
                final int shoot_motor_speed_percent = (int) (shoot_motor.getPower() * 100);

                String team_color;
                if (is_red_team)
                {
                        team_color = "red";
                }
                else
                {
                        team_color = "blue";
                }

                telemetry.addData("team_color :- ", team_color);
                telemetry.addData("joystick_strafe.x :- ", joystick_strafe_x);
                telemetry.addData("joystick_strafe.y :- ", joystick_strafe_y);
                telemetry.addData("joystick_rotate.x :- ", joystick_rotate_x);
                telemetry.addData("shooter_motor_usage_speed (%) :- ", shoot_motor_speed_percent);

                telemetry.update();
        }
}
