#include "units.h"

using namespace unit_literals;
int main()
{
    // Simple Harmonic Motion
    unit::Length A = .15;
    unit::AngularSpeed w = 2.0_rad / unit::second; 
    unit::Angle phi = 0.5_pi;
    
    unit::Time t = 0.0;

    unit::Frequency f = w / (2_pi * unit::radian);

    std::cout << "y = Asin(wt+phi) = " << (A*sin(w * t + phi)).getValue() << std::endl;


    // Ideal pendulum
    unit::Length L = 1.0;
    unit::Acceleration g = 9.8_m / (1_s * 1_s);

    unit::Time Period = 2_pi * unit::sqrt(L / g);


    // Physical pendulum that is the case of the ideal pendulum
    unit::Length D = L;
    unit::Mass m = 1.0;
    unit::Quantity<1, 2, 0, 0, 0> MOI = m * D * D;


    unit::Time Period2 = 2_pi * sqrt(MOI / (m * g * D));


    auto thing = sqrt(1_m * 1_m);

    std::cout << "T = sqrt(L / g) = "<< Period.getValue()<< std::endl;
    std::cout << "T = sqrt(I / mgd) = "<< Period2.getValue()<< std::endl;

}