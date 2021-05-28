/**
 * Thanh Tran
 * thanhtran@berkeley.edu
 * LEAPFROG, Summer 2021
*/

#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "actuators/srv/actuator_move_gimbal.hpp"
#include "sensors/msg/sensor_linear_actuator.hpp"
#include <wiringPi.h>

using namespace std::chrono_literals;

/**
 * GPIO Pins
 */
#define PIN_ROLL_P 18   // White
#define PIN_ROLL_N 23   // Grey
#define PIN_PITCH_P 24  // Purple
#define PIN_PITCH_N 17  // Yellow

/**
 * Gimbal Properties
 * 
 * @pin_f Forward pin
 * @pin_b Backward pin
 * @analog_read Pin that reads analog input
 * @max_angle Maximum angle of linear actuator
 * @min_angle Minimum angle of linear actuator
 */
struct GimbalProp
{
    int pin_f;
    int pin_b;
    int analog_read;
    int origin;
    int max_angle;
    int min_angle;
};

class GimbalManager : public rclcpp::Node
{
private:
    // ROS variables
    rclcpp::Service<actuators::srv::ActuatorMoveGimbal>::SharedPtr gimbal_service_;
    rclcpp::Subscription<sensors::msg::SensorLinearActuator>::SharedPtr actuator_subscriber_;
    GimbalProp gimbalProp[2] = {
        {
            // Responsible for roll
            PIN_ROLL_P,PIN_ROLL_N, 0, 0, 0, 0
        },
        {
            // Responsible for pitch
            PIN_PITCH_P,PIN_PITCH_N, 0, 0, 0, 0    
        }
    };
    // Error tolerance
    const float ERROR = 0.5;
    // Measurement in millimeters of Gimbal system required to convert angular values to linear values
    const int H = 378;
    const int L = 158;
    const int X = 287;
    // Mapping values 
    int in_min = 0;
    int in_max = 1023;
    int out_min = 0;
    int out_max = 50;

public:
    GimbalManager() : Node("Gimbal")
    {
        // Initialize pin modes and calibrating roll then pitch
        //  0: roll, 1: pitch
        int gimbalProperties = 2;
        for(int i = 0; i < gimbalProperties; ++i) {
            pinMode(gimbalProp[i].pin_f, OUTPUT);
            pinMode(gimbalProp[i].pin_b, OUTPUT);
            pinMode(gimbalProp[i].analog_read, INPUT);
            calibrate(gimbalProp[i]);
        }

        // Create linear actuator subscriber and change the analog read of each gimbal property (roll, pitch)
        this->actuator_subscriber_ = this->create_subscription<sensors::msg::SensorLinearActuator>("/sensors/linear_position", 10, [this](const sensors::msg::SensorLinearActuator::SharedPtr msg) -> void {
            gimbalProp[0].analog_read = msg->position[0];
            gimbalProp[1].analog_read = msg->position[1];
        });

        // Create threads in order to perform roll & pitch simultaneously
        this->gimbal_service_ = this->create_service<actuators::srv::ActuatorMoveGimbal>("move_gimbal", [this](const std::shared_ptr<actuators::srv::ActuatorMoveGimbal::Request> request, std::shared_ptr<actuators::srv::ActuatorMoveGimbal::Response> response) -> void {
            std::thread threads[2];
            for (int i = 0; i < 2; i++)
            {
                threads[i] = std::thread([this](int expected_pos, GimbalProp prop) {
                    int curr_pos = mapFunc(prop.analog_read);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current Position: curr_pos = %d", curr_pos);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving to Expected Position: expected_pos = %d", expected_pos);

                    // Gimbal movement negative feedback logic
                    while (curr_pos != expected_pos) {
                        int diff = curr_pos - expected_pos;
                        if (abs(diff) < this->ERROR) {
                            halt(prop);
                            break;
                        } else if (diff > 0) {
                            moveForward(prop);
                        } else if (diff < 0) {
                            moveBackward(prop);
                        }
                        curr_pos = mapFunc(prop.analog_read);
                    }

                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current Position: curr_pos = %d", curr_pos);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Expected Position: expected_pos = %d", expected_pos);
                },
                                        angularToLinear(request->angles[i]), gimbalProp[i]);
            }

            // Perform threads
            for (int i = 0; i < 2; i++)
            {
                threads[i].join();
                
            }
            response->status = true;
        });
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gimbal service complete!");
    }
    /**
     * Calibrates the GimbalProp Obj
     * 
     * @prop Gimbal property to calibrate: roll or pitch
     */
    void calibrate(GimbalProp prop) {
        int duration = 10000;           // TODO: Find the actual time the linear actuator takes to fully extend
        halt(prop);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calibrating gimbal...");

        // Get origin
        prop.origin = mapFunc(prop.analog_read);

        moveForward(prop);
        std::this_thread::sleep_for(std::chrono::milliseconds(duration));
        halt(prop);
        prop.max_angle = mapFunc(prop.analog_read);
        
        moveBackward(prop);
        std::this_thread::sleep_for(std::chrono::milliseconds(duration));
        halt(prop);
        prop.min_angle = mapFunc(prop.analog_read);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Done Calibration!");
        halt(prop);
    }

    /**
     * Converts float angular value to int linear value in millimeters
     * 
     * @angle Float of gimbal in terms of angular
     */
    int angularToLinear(float angle) {
        float piDiv180 = 0.017460;
        int linear = (int) (sqrt(H*H + L*L - 2*H*L*cos(angle * piDiv180)) - X);
        return linear;
    }

    /**
     * Maps values from Arduino values [0, 1023] -> [0, 50] in millimeters
     * When using ADC instead of Arduino -> [0, 4096]
     * 
     * @source https://www.arduino.cc/reference/en/language/functions/math/map/
     * @x Position of linear actuator from actuator_subscriber_
     */
    int mapFunc(int x) {
        return (x - this->in_min) * (this->out_max - out_min) / (in_max - in_min) + out_min;
    }
    

    /**
     * Move forward
     * 
     * @prop Gimbal property to move: roll or pitch
     */
    void moveForward(GimbalProp prop) {
        digitalWrite(prop.pin_f, HIGH);
        digitalWrite(prop.pin_b, LOW);
    }

    /**
     * Move backward
     * 
     * @prop Gimbal property to move: roll or pitch
     */
    void moveBackward(GimbalProp prop) {
        digitalWrite(prop.pin_f, LOW);
        digitalWrite(prop.pin_b, HIGH);
    }

    /**
     * Halts movement
     * 
     * @prop Gimbal property to halt: roll or pitch
     */
    void halt(GimbalProp prop) {
        digitalWrite(prop.pin_f, HIGH);
        digitalWrite(prop.pin_b, HIGH);
    }
};

int main(int argc, char *argv[])
{
    wiringPiSetupGpio();
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GimbalManager>());
    rclcpp::shutdown();
    return 0;
}