#pragma GCC diagnostic warning "-Wunused-but-set-variable"

#include <iostream>
#include <fstream>
#include <filesystem>
#include "include/actuators.h"
#include <matplot/matplot.h>
#include <cmath>
using namespace std;

const char* fp = "../Data/spec-report.tex";    

int main() {
    cout << "Starting Report generator\n";
    
    robot_requirements_t req;
    req.acceleration_time = 2.00;
    req.friction_coefficient = 0.400;
    req.ground_slope = rads(40);
    req.mass = 5.00/g;
    req.max_vel = 3.00;             // m/s
    req.wheel_diameter = 0.120;  // 12 cm
    
    FBD_forces_t fr;
    calculate_FBD_forces(req, &fr);

    cout << "Pull force = " << fr.F_gx << " N" << endl;
    cout << "Friction force = " << fr.F_fric << " N" << endl;
    cout << "Acceleration force = " << fr.F_i << " N" << endl;
    cout << "Motor Forces = " << fr.F_net << " N" << endl;

    motor_specification_t ms;
    wheel_motor_specification(req,fr, &ms);

    cout << endl;
    cout << "Wheel torque demand = " << kg_cm(ms.stall_torque)<< " kg-cm" << endl;
    cout << "Wheel speed demand = " << rpm(ms.angular_velocity) << " RPM" << endl;

    using namespace matplot;
    
    //plot(iota(-10*sin(req.ground_slope),10*sin(req.ground_slope)));
    textarrow(0,0,-fr.F_i*cos(req.ground_slope),fr.F_i*-sin(req.ground_slope),"F_i");
    textarrow(0,-1,-fr.F_fric*cos(req.ground_slope),fr.F_fric*-sin(req.ground_slope)-1,"F_f");
    textarrow(0,-2,-fr.F_gx*cos(req.ground_slope),fr.F_gx*-sin(req.ground_slope)-2,"F_g_x");
    textarrow(0,0,fr.F_net*cos(req.ground_slope),fr.F_net*sin(req.ground_slope)+0,"F_net");
    textarrow(0,9.8,0,0,"F_g");
    xlim({-6, 6});
    ylim({-6, 6});
    show();

    // cout << "File path: ${workspaceFolder}\n";
    // ofstream myfile;
    // myfile.open(fp,ios::out);
    // myfile << "\\documentclass{article}\n";
    // myfile << "\\title{Robot Specification}\n";
    // myfile << "\\author{SpecGen}\n";
    // myfile << "\\date{\today}\n";
    
    // myfile << "\\begin{document}\n";
    // myfile << "\\maketitle\n";
    // myfile << "\\section{Introduction}\n";

    // myfile << "This document has been auto-generated by SpecGen.\n";
    
    // myfile << "\\section{Functional Requirements}\n";
    // myfile << "Speed: 4 m/s\nMass: 5 kg\nDimensions: 200 x 150 x 120\n";
    // myfile << "\\end{document}\n";

    // int speed = 0;
    // cout << "How fast should the robot go? (m/s) remember, walking speed is about 1.5 m/s\n";
    // cin >> speed;

    // int incline = 0;
    // cout << "What is the steepest incline the robot will climib? Answer in degrees\n";
    // cin >> incline;

    // int weight_limit = 0;
    // cout << "How heavy will the robot be? Answer in kg\n";
    // cin >> weight_limit;


    // struct dimensions_t{
    //     int x;
    //     int y;
    //     int z;
    // };
    // dimensions_t dims;
    // cout << "What are the size limits?\n";
    // cin >> dims.x;
    // myfile.close();



    return 0;
}

