

#include "mbed.h"
#include "PESBoardPinMap.h"
#include "DebounceIn.h"
#include "DCMotor.h"
#include <Eigen/Dense>
#include "SensorBar.h"

#define M_PIf 3.14159265358979323846f

bool do_execute_main_task = false;
bool do_reset_all_once = false;

DebounceIn user_button(BUTTON1);
void toggle_do_execute_main_fcn();

// Maze navigation states
enum MazeState { FOLLOW_LINE, TURN_RIGHT, TURN_LEFT, TURN_AROUND };
MazeState state = FOLLOW_LINE;

int main() {
    user_button.fall(&toggle_do_execute_main_fcn);
    const int main_task_period_ms = 20;
    Timer main_task_timer;
    DigitalOut user_led(LED1);
    DigitalOut led1(PB_9);
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    const float voltage_max = 12.0f;
    const float gear_ratio = 391.00f;
    const float kn = 36.0f / 12.0f;

    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);

    const float d_wheel = 0.034f; // wheel diameter in meters
    const float b_wheel = 0.15572f;  // wheelbase, distance from wheel to wheel in meters
    const float bar_dist = 0.114f;

    const float r1_wheel = d_wheel / 2.0f;
    const float r2_wheel = d_wheel / 2.0f;

    Eigen::Matrix2f Cwheel2robot;
    Cwheel2robot << r1_wheel / 2.0f, r2_wheel / 2.0f,
                    r1_wheel / b_wheel, -r2_wheel / b_wheel;

    SensorBar sensor_bar(PB_9, PB_8, bar_dist);

    float angle{0.0f};
    float angular_vel{0.0f};
    float velocity{0.0f};

    const float Kp{5.0f};
    const float Kp_nl{2.0f};
    const float gain{8.0f};

    const float wheel_vel_max = 0.5f * M_PIf * motor_M2.getMaxPhysicalVelocity();

    main_task_timer.start();

    while (true) {
        main_task_timer.reset();

        if (do_execute_main_task) {
            led1 = 1;
            enable_motors = 1;

            if (sensor_bar.isAnyLedActive())
                angle = sensor_bar.getAvgAngleRad();

            // Maze navigation logic
            if (sensor_bar.getRaw() == 0b00000000) {
                // Dead end
                state = TURN_AROUND;
            } else if ((sensor_bar.getRaw() & 0b11100000) == 0b11100000) {
                // Left junction
                state = TURN_LEFT;
            } else if ((sensor_bar.getRaw() & 0b00000111) == 0b00000111) {
                // Right junction
                state = TURN_RIGHT;
            } else {
                state = FOLLOW_LINE;
            }

            Eigen::Vector2f robot_coord;

            switch (state) {
                case FOLLOW_LINE:
                    if (angle >= 0.0f) {
                        angular_vel = Kp * angle + Kp_nl * angle * std::abs(angle);
                    } else {
                        angular_vel = Kp * angle - Kp_nl * angle * std::abs(angle);
                    }
                    velocity = wheel_vel_max * r1_wheel * exp(-gain * angle * angle);
                    break;

                case TURN_LEFT:
                    velocity = 0.1f;
                    angular_vel = 2.5f;
                    break;

                case TURN_RIGHT:
                    velocity = 0.1f;
                    angular_vel = -2.5f;
                    break;

                case TURN_AROUND:
                    velocity = 0.0f;
                    angular_vel = 3.0f;
                    break;
            }

            robot_coord << velocity, angular_vel;
            Eigen::Vector2f wheel_speed = Cwheel2robot.inverse() * robot_coord;

            motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf));
            motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PIf));
        } else {
            if (do_reset_all_once) {
                do_reset_all_once = false;
                led1 = 0;
                enable_motors = 0;
            }
        }

        user_led = !user_led;

        int main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - main_task_elapsed_time_ms < 0)
            printf("Warning: Main task took longer than main_task_period_ms\n");
        else
            thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void toggle_do_execute_main_fcn() {
    do_execute_main_task = !do_execute_main_task;
    if (do_execute_main_task)
        do_reset_all_once = true;
}