#include <stdio.h>
#include "../include/actuators.h"

float rads(float degrees){return (degrees/180)*pi;}
float degs(float radians){return (radians/pi)*180;}
float rpm(float rad_s){return (rad_s * 9.549297 );}
float rad_s(float rpm){return (rpm * 0.10472);}
float kg_cm(float newton_meters){return(newton_meters * 10.197162129779);}
float n_m(float kg_cm){return (kg_cm * 0.0980665);}

SG_error_t calculate_FBD_forces(robot_requirements_t i, FBD_forces_t *o)
{
    o->F_gx = sin(i.ground_slope)*i.mass*g;
    o->F_fric = cos(i.ground_slope)*i.mass*g*i.friction_coefficient;
    o->F_a = i.mass*(i.max_vel/i.acceleration_time);
    o->F_net = o->F_a+ o->F_gx + o->F_fric;
    return SUCCESS;
}

SG_error_t wheel_motor_specification(robot_requirements_t r,FBD_forces_t f, motor_specification_t *o)
{
    float net_torque = f.F_net*(r.wheel_diameter/2);
    float individual_torque = net_torque/r.num_wheels;
    // Stall torque should be 4 times greater than operating torque
    o->stall_torque = individual_torque * 4;
    float R = r.wheel_diameter/2;
    o->angular_velocity = r.max_vel/R;
    return SUCCESS;
}