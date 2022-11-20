#pragma once

#include <cmath>

#ifndef ACTUATORS_H
#define ACTUATORS_H

// defines
#define pi 3.14159
#define g  9.81


// Typedeffs
typedef enum SG_error_t{
    SUCCESS,
    GENERAL_FAILUE
}SG_error_t;

typedef enum actuator_type_t{
    LINEAR,
    ROTATION
}actuator_type_t;

typedef struct FBD_forces_t{
    float F_gx;
    float F_fric;
    float F_a;
    float F_net;
}FBD_forces_t;

typedef struct robot_requirements_t{
    
    float ground_slope;
    float friction_coefficient;
    float mass;
    float acceleration_time;
    float wheel_diameter;
    float max_vel;
    int   num_wheels;
    robot_requirements_t()
    {
        ground_slope = 0.0;
        friction_coefficient = 0.0;
        mass = 0.0;
        acceleration_time = 0.0;
        wheel_diameter = 0.0;
        max_vel = 0.0;
        num_wheels = 4;
    }
    
}robot_requirements_t;

typedef struct motor_specification_t{
    float stall_torque = 0.0;
    float angular_velocity = 0.0;
}motor_specification_t;

// Conversion Functions
float rads(float degrees);
float degs(float radians);
float rpm(float rad_s);
float rad_s(float rpm);
float kg_cm(float newton_meters);
float n_m(float kg_cm);


SG_error_t calculate_FBD_forces(robot_requirements_t i, FBD_forces_t *o);

SG_error_t wheel_motor_specification(   robot_requirements_t r,
                                        FBD_forces_t f, 
                                        motor_specification_t *o);


#endif