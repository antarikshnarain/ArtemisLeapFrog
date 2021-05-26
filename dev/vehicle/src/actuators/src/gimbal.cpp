/**
 * Thanh Tran
 * thanhtran@berkeley.edu
 * LEAPFROG, Summer 2021
*/

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "actuators/srv/actuator_move_gimbal.hpp"
#include <wiringPi.h>

using namespace std::chrono_literals;

/* TODO: Initialize pin numbers
#define PIN_ROLL_P 11
#define PIN_ROLL_N 10
#define PIN_PITCH_P 24
#define PIN_PITCH_N 17
#define READ_0 
#define READ_1
*/

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
    GimbalProp gimbalProp[2] = {
        {
            // Responsible for roll
            PIN_ROLL_P,PIN_ROLL_N,READ_0, READ_0, 0, 0
        },
        {
            // Responsible for pitch
            PIN_PITCH_P,PIN_PITCH_N,READ_1, READ_1, 0, 0    
        }
    };
    // Error tolerance
    const float ERROR = 0.5;

public:
    GimbalManager() : Node("Move")
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

        // Create threads in order to perform roll & pitch simultaneously
        this->gimbal_service_ = this->create_service<actuators::srv::ActuatorMoveGimbal>("gimbal", [this](const std::shared_ptr<actuators::srv::ActuatorMoveGimbal::Request> request, std::shared_ptr<actuators::srv::ActuatorMoveGimbal::Response> response) -> void {
            std::thread threads[2];
            for (int i = 0; i < 2; i++)
            {
                threads[i] = std::thread([](int expected_pos, GimbalProp prop) {
                    int curr_pos = analogRead(prop.analog_read); // TODO: change to subscriber
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
                        curr_pos = analogRead(prop.analog_read); // TODO: change to subscriber
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
        halt();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calibrating gimbal...");

        // Get origin
        prop.origin = analogRead(prop.analog_read);

        moveForward();
        std::this_thread::sleep_for(std::chrono::milliseconds(duration));
        halt();
        prop.max_angle = analogRead(prop.analog_read);
        
        moveBackward();
        std::this_thread::sleep_for(std::chrono::milliseconds(duration));
        halt();
        prop.min_angle = analogRead(prop.analog_read);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Done Calibration!");
        halt();
    }

    /**
     * Converts float angular value to int linear value
     * 
     * @angle Float of gimbal in terms of angular
     */
    int angularToLinear(float angle) {
        // TODO: map float angular value to int linear value
        int linear = 0;

        return linear;
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

    /**
     * Moves gimbal to origin
     * 
     * @prop Gimbal property to move to origin: roll or pitch
     */
    void moveToOrigin(GimbalProp prop) {
       this->gimbal_service_ = this->create_service<actuators::srv::ActuatorMoveGimbal>("gimbal", [this](const std::shared_ptr<actuators::srv::ActuatorMoveGimbal::Request> request, std::shared_ptr<actuators::srv::ActuatorMoveGimbal::Response> response) -> void {
            std::thread threads[2];
            for (int i = 0; i < 2; i++)
            {
                threads[i] = std::thread([](GimbalProp prop) {
                    int curr_pos = analogRead(prop.analog_read); // TODO: change to subscriber
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current Position: curr_pos = %d", curr_pos);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving to Origin Position: prop.origin = %d", prop.origin);

                    // Gimbal movement negative feedback logic
                    int duration;               // TODO: should we pass in duration?
                    while (curr_pos != prop.origin) {
                        int diff = prop.origin - prop.origin;
                        if (abs(diff) < this->ERROR) {
                            halt(prop);
                            break;
                        } else if (diff > 0) {
                            moveForward(prop);
                        } else if (diff < 0) {
                            moveBackward(prop);
                        }
                        curr_pos = analogRead(prop.analog_read); // TODO: change to subscriber
                    }

                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current Position: curr_pos = %d", curr_pos);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Origin Position: prop.origin = %d", prop.origin);
                },
                                        gimbalProp[i]);
            }

            // Perform threads
            for (int i = 0; i < 2; i++)
            {
                threads[i].join();
                
            }
            response->status = true;
        });
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gimbal moved to origin complete!");
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