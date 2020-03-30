//
//  main.cpp
//  GrassHopper
//
//  Created by Quan Bach  on 11/26/19.
//  Copyright © 2019 QB. All rights reserved.
//

#include <chrono>
#include <thread>
#include <krpc.hpp>
#include <krpc/services/space_center.hpp>
#include <krpc/services/krpc.hpp>
#include <fstream>
#include <iostream>
#include <string>


const int SET_TARGET = 1000;    //hoovering target in meters
const float G_FIELD = 9.81;     //gravity field of Kerbin
int main()
{
    krpc::Client conn = krpc::connect("Grass Hopper", "192.168.1.87",50000,50001);
    
    
    //    //establish connection to KSP
    //    krpc::Client conn = krpc::connect("F9-First-Launch");
    krpc::services::SpaceCenter space_center(&conn);
    
    //assign var vessel to the active vessel on launch pad
    auto vessel = space_center.active_vessel();
    
    //Set up stream
    auto ut = space_center.ut_stream();
    auto altitude = vessel.flight().mean_altitude_stream();
    auto apoapsis = vessel.orbit().apoapsis_altitude_stream();
    auto periapsis = vessel.orbit().period_stream();
    
    //    double targetPitch = 90;
    
    //SET UP AUTO-PILOT
    //activate auto pilot
    vessel.auto_pilot().engage();
    
    //pitch to target pitch
    vessel.auto_pilot().target_pitch_and_heading(90, 90);
    
    //set throttle power to 0%
    vessel.control().set_throttle(0);
    
/* ************* DISBALED FOR CURRENT MISSION - GRASS HOPPER  ********** */
    //enbale SAS and set to stability assist
    //    vessel.auto_pilot().set_sas(true);
    //    vessel.auto_pilot().set_sas_mode(krpc::services::SpaceCenter::SASMode::stability_assist);
    
    vessel.control().set_sas(false);
    //disbale RCS
    vessel.control().set_rcs(false);
    
    //set reference frame
    auto srf_frame = vessel.orbit().body().reference_frame();

    
    //START COUNTING DOWN
    std::cout << "3..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "2..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "1..." << "Ignition!" << std::endl;
    vessel.control().activate_next_stage();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    //Lift Off
    std::cout << "...and LIFT OFF!" << std::endl;
    
    
    // Necessary variables for testing
    //save data to files for analysis
    std::ofstream data_log;
    const std::string data_log_filename = "data_log.csv";
    
    try
    {
        data_log.open(data_log_filename, std::ios::out);
        if (!data_log.is_open())
            throw std::string ("Can't open the find");
    }
    catch (std::string &s)
    {
        std::cout << s  << data_log_filename << std::endl;
    }
    
   
    //tuning parameters
    double k_p = 0.00001;    // propotional gain
    double k_i = 0.00001;    // intergral gain
    double k_d = 0.0001;     // derrivative gain
    
    data_log << "kP" << ',' << "kI" << ',' << "kD" <<  std::endl;
    data_log << k_p << ',' << k_i << ',' << k_d <<  std::endl;
    data_log <<  "Target" << ',' << "Altitude" << ',' << "Time" << ',' << "P" << ',' << "I" << ',' << "D" << ',' << "Thrott" << ',' << "Thrott_G" <<  std::endl;
       
    
    
    
    
    //main flight loop
        while (1)
        {
            bool done_running = false;
            double error_sum = 0.0;                 // intergral error
            double error_derrivative = 0.0;         // derrivative error
            double current_error = 0.0;             // current error at time t
            double prev_error = 0.0;                // prev error at prev t
            double prev_time_t = 0.1;               // previous elapsed time of the mission, set to 0.1 so the first change is not zero to avoid                                        division by zero
            double thrott = 0.0;                    // thrott value for vehicle
            auto now = vessel.met();                // set new prev time t
            auto current_altitude = altitude();     // current altitude at time t of computation
            
            volatile float available_thrust = vessel.available_thrust();  //The amount of thrust, in Newtons, that would be produced by the engine when activated and with its throttle set to 100%. Returns zero if the engine does not have any fuel. Takes the engine’s current Engine::thrust_limit() and atmospheric conditions into account.
            float g_field_thrott = (vessel.mass()*G_FIELD)/available_thrust; //thrott level to win g force
            
            //PID controller
            if (!done_running)
            {
                current_error = SET_TARGET - current_altitude;
               
                
                if (now!= 0)
                {
                    error_sum += (current_error - prev_error)/2 * (now - prev_time_t);
                    error_derrivative =  (current_error - prev_error) / (now - prev_time_t);

                }
                
                thrott =  g_field_thrott + k_p * current_error  + k_i * error_sum  + k_d * error_derrivative ;
                vessel.control().set_throttle(thrott);
                prev_error = current_error;
                prev_time_t = now;
            }
            
            
            
            data_log << SET_TARGET  << ',' << altitude() << ',' << now << ',' << current_error << ',' << error_sum << ',' << error_derrivative << ',' << thrott << ',' << g_field_thrott << std::endl;
            std::cout << current_error << "      " << error_sum << "      "<< error_derrivative  << "      "   << thrott << std::endl;
        }
    
    


    return 0;
}
