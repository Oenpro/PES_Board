#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "IMU.h"
#include "Servo.h"
#include "DCMotor.h"
#include <Eigen/Dense>
#include "SensorBar.h"

#define M_PIf 3.14159265358979323846f // pi

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user
                                   // button gets pressed, definition at the end

// main runs as an own thread
int main()
{
    // attach button fall function address to user button object
    user_button.fall(&toggle_do_execute_main_fcn);

    // while loop gets executed every main_task_period_ms milliseconds, this is a
    // simple approach to repeatedly execute main
    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, therefore
                                        // the main task will run 50 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task
                                        // every main_task_period_ms

    // led on nucleo board
    DigitalOut user_led(LED1);

    // --- adding variables and objects and applying functions starts here ---

    // servo
    Servo servo_roll(PB_D0);
    Servo servo_pitch(PB_D1);

    // imu
    ImuData imu_data;
    IMU imu(PB_IMU_SDA, PB_IMU_SCL);
    Eigen::Vector2f rp(0.0f, 0.0f);

    // minimal pulse width and maximal pulse width obtained from the servo calibration process
    // careful, these values might differ from servo to servo
    // reely s-0090
    float servo_D0_ang_min = 0.0350f; 
    float servo_D0_ang_max = 0.1200f;
    // reely S3003
    float servo_D1_ang_min = 0.0250f;
    float servo_D1_ang_max = 0.1300f;

    // servo.setPulseWidth: before calibration (0,1) -> (min pwm, max pwm)
    // servo.setPulseWidth: after calibration (0,1) -> (servo_D0_ang_min, servo_D0_ang_max)
    servo_roll.calibratePulseMinMax(servo_D1_ang_min, servo_D1_ang_max);
    servo_pitch.calibratePulseMinMax(servo_D0_ang_min, servo_D0_ang_max);

    // angle limits of the servos
    const float angle_range_min = -M_PIf / 2.0f;
    const float angle_range_max =  M_PIf / 2.0f;

    // angle to pulse width coefficients
    const float normalised_angle_gain = 1.0f / M_PIf;
    const float normalised_angle_offset = 0.5f;

    // pulse width
    static float roll_servo_width = 0.5f;
    static float pitch_servo_width = 0.5f;

    servo_roll.setPulseWidth(roll_servo_width);
    servo_pitch.setPulseWidth(pitch_servo_width);

    // linear 1-D mahony filter
    const float Ts = static_cast<float>(main_task_period_ms) * 1.0e-3f; // sample time in seconds
    const float kp = 3.0f;
    float roll_estimate = 0.0f;
    float pitch_estimate = 0.0f;

        // create object to enable power electronics for the dc motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
                                     // 6.0f V if you only use one battery pack
    const float gear_ratio = 100.00f;
    const float kn = 140.0f / 12.0f;

    // motor M1 and M2, do NOT enable motion planner when used with the LineFollower (disabled per default)
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);

    // differential drive robot kinematics
    const float d_wheel = 0.0673f; // wheel diameter in meters
    const float b_wheel = 0.147f;  // wheelbase, distance from wheel to wheel in meters
    const float bar_dist = 0.180f; // distance from wheel axis to leds on sensor bar / array in meters
    // const float d_wheel = 0.07f; // wheel diameter in meters
    // const float b_wheel = 0.147f;  // wheelbase, distance from wheel to wheel in meters
    // const float bar_dist = 0.114f; // distance from wheel axis to leds on sensor bar / array in meters
    const float r1_wheel = d_wheel / 2.0f; // right wheel radius in meters
    const float r2_wheel = d_wheel / 2.0f; // left  wheel radius in meters
    // transforms wheel to robot velocities
    Eigen::Matrix2f Cwheel2robot;
    Cwheel2robot << r1_wheel / 2.0f   ,  r2_wheel / 2.0f   ,
                    r1_wheel / b_wheel, -r2_wheel / b_wheel;

    // sensor bar
    SensorBar sensor_bar(PB_9, PB_8, bar_dist);

    // angle measured from sensor bar (black line) relative to robot
    float angle{0.0f};

    // rotational velocity controller
    const float Kp = 14.0f;
    const float Kp_nl = 1.2f * 17.0f;
    const float wheel_vel_max = 2.0f * M_PIf * motor_M2.getMaxPhysicalVelocity()* 0.2f;
    const float maximum_velocity = wheel_vel_max*r1_wheel; // maximum velocity

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        // --- code that runs every cycle at the start goes here ---

        // read imu data
        imu_data = imu.getImuData();

        // // roll, pitch, yaw according to Tait-Bryan angles ZYX
        // // where R = Rz(yaw) * Ry(pitch) * Rx(roll) for ZYX sequence
        // // singularity at pitch = +/-pi/2 radians (+/- 90 deg)
        // rp(0) = imu_data.rpy(0); // roll angle
        // rp(1) = imu_data.rpy(1); // pitch angle

        // // pitch, roll, yaw according to Tait-Bryan angles ZXY
        // // where R = Rz(yaw) * Rx(roll) * Ry(pitch)
        // // singularity at roll = +/-pi/2
        // rp(0) = imu_data.pry(1); // roll angle
        // rp(1) = imu_data.pry(0); // pitch angle

        // linear 1-D mahony filter
        const float roll_acc = atan2f(imu_data.acc(1), imu_data.acc(2)); // roll angle from accelerometer
        const float pitch_acc = atan2f(-imu_data.acc(0), imu_data.acc(2)); // pitch angle from accelerometer
        // // roll, pitch, yaw according to Tait-Bryan angles ZYX
        // const float roll_acc = atan2f(imu_data.acc(1), imu_data.acc(2)); // roll angle from accelerometer
        // const float pitch_acc = atan2f(-imu_data.acc(0), sqrtf(imu_data.acc(1) * imu_data.acc(1) + imu_data.acc(2) * imu_data.acc(2))); // pitch angle from accelerometer
        // // pitch, roll, yaw according to Tait-Bryan angles ZXY
        // const float pitch_acc = atan2f(-imu_data.acc(0), imu_data.acc(2)); // pitch angle from accelerometer
        // const float roll_acc = atan2f(imu_data.acc(1), sqrtf(imu_data.acc(0) * imu_data.acc(0) + imu_data.acc(2) * imu_data.acc(2))); // roll angle from accelerometer
        roll_estimate  += Ts * (imu_data.gyro(0) + kp * (roll_acc  - roll_estimate ));
        pitch_estimate += Ts * (imu_data.gyro(1) + kp * (pitch_acc - pitch_estimate));
        rp(0) = roll_estimate; // roll angle
        rp(1) = pitch_estimate; // pitch angle

        if (do_execute_main_task) {

        // --- code that runs when the blue button was pressed goes here ---

            enable_motors = 1;

            // only update sensor bar angle if an led is triggered
            if (sensor_bar.isAnyLedActive())
                angle = sensor_bar.getAvgAngleRad();
            // printf("angle: %f \n ", angle);

            // // control algorithm for robot velocities
            // Eigen::Vector2f robot_coord = {0.5f * wheel_vel_max * r1_wheel,  // half of the max. forward velocity
            //                                Kp * angle                     }; // simple proportional angle controller

            // // control algorithm for robot velocities
            // Eigen::Vector2f robot_coord = {0.5f * wheel_vel_max * r1_wheel,  // half of the max. forward velocity
            //                                Kp * angle + Kp_nl * angle * fabsf(angle)                 }; // simple proportional angle controller
        
            Eigen::Vector2f robot_coord = {maximum_velocity/(1+(4*fabsf(angle))),  // forward velocity changes with the error value
                                           Kp * angle + Kp_nl * angle * fabsf(angle)                 }; // simple proportional angle controller

            // map robot velocities to wheel velocities in rad/sec
            Eigen::Vector2f wheel_speed = Cwheel2robot.inverse() * robot_coord;

            // setpoints for the dc motors in rps
            motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M1
            motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M2

            // enable the servos
            if (!servo_roll.isEnabled())
                servo_roll.enable();
            if (!servo_pitch.isEnabled())
                servo_pitch.enable();

            // map to servo commands
            roll_servo_width  = -normalised_angle_gain * rp(0) + normalised_angle_offset;
            pitch_servo_width =  normalised_angle_gain * rp(1) + normalised_angle_offset;
            if (angle_range_min <= rp(0) && rp(0) <= angle_range_max)
                servo_roll.setPulseWidth(roll_servo_width);
            if (angle_range_min <= rp(1) && rp(1) <= angle_range_max)
                servo_pitch.setPulseWidth(pitch_servo_width);

        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // --- variables and objects that should be reset go here ---

                // reset variables and objects
                roll_servo_width = 0.5f;
                pitch_servo_width = 0.5f;
                servo_roll.setPulseWidth(roll_servo_width);
                servo_pitch.setPulseWidth(pitch_servo_width);
                enable_motors = 0;
            }
        }

        // toggling the user led
        user_led = !user_led;

        // --- code that runs every cycle at the end goes here ---

        // print to the serial terminal
        // printf("%6.2f, %6.2f \n", roll_servo_width, pitch_servo_width);
        printf("%6.2f, %6.2f ", imu_data.rpy(0) * 180.0f / M_PIf, imu_data.rpy(1) * 180.0f / M_PIf);
        printf("%6.2f, %6.2f \n", roll_estimate * 180.0f / M_PIf, pitch_estimate * 180.0f / M_PIf);
        // printf("%6.2f, %6.2f ", imu_data.pry(1) * 180.0f / M_PIf, imu_data.pry(0) * 180.0f / M_PIf);
        // printf("%6.2f, %6.2f \n", roll_estimate * 180.0f / M_PIf, pitch_estimate * 180.0f / M_PIf);

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - main_task_elapsed_time_ms < 0)
            printf("Warning: Main task took longer than main_task_period_ms\n");
        else
            thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void toggle_do_execute_main_fcn()
{
    // toggle do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    // set do_reset_all_once to true if do_execute_main_task changed from false to true
    if (do_execute_main_task)
        do_reset_all_once = true;
}
