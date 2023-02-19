#include "units.h"
#include "geometry.h"
#include "filters.h"
#include "geometry.h"
#include "control_systems.h"

using namespace unit;

unit::Angle measure_encoder_left()
{
    return 1 * unit::radian;
}
unit::Angle measure_encoder_right()
{
    return 1 * unit::radian;
}

struct robot_config_t
{
    unit::Length dist_between_wheels;
    unit::Length odom_wheel_radius;
};

const robot_config_t robot_specs = {
    .dist_between_wheels = 10_in,
    .odom_wheel_radius = 1.5_in,
};

// s = r theta
// length = length * angle

unit::Angle get_robot_heading()
{
    using namespace unit;
    const Length wheel_circumference = robot_specs.odom_wheel_radius * 2_pi;

    Angle left_sensor = measure_encoder_left();
    Angle right_sensor = measure_encoder_right();

    Length left_dist = wheel_circumference / unit::revolution * left_sensor;
    Length right_dist = wheel_circumference / unit::revolution * right_sensor;

    Length arc_length_traveled = (right_dist - left_dist) / 2.0;
    // This is kinda funny. theta is a number because the angles are not really a unit.
    // arc length (length) = r (length) * theta (angle) - here, angle is just a number quantity.
    Number arc_theta = (arc_length_traveled / robot_specs.dist_between_wheels);
    Angle heading = unit::radian * arc_theta; // to ensure unit safety, we make it back into an angle now

    return heading;
}

unit::Length get_robot_distance()
{
    using namespace unit;
    Angle left_sensor = measure_encoder_left();
    Angle right_sensor = measure_encoder_right();

    Length left_dist = robot_specs.dist_between_wheels / unit::revolution * left_sensor;
    Length right_dist = robot_specs.dist_between_wheels / unit::revolution * right_sensor;

    Length dist_avg = (left_dist + right_dist) / 2.0;

    return dist_avg;
}

pose2d_t robot_pose = {.pos = {.x = 0_in, .y = 0_in}, .heading = 90_deg};

// int main()
// {
//     point2d_t p1 = {.x = 1_in, .y = -1_in};
//     point2d_t p2 = {.x = 1_in, .y = 1_in};
//     Angle a = p1.direction();

//     std::cout << p1.distance(p2).Convert(inch) << "in" << std::endl;
//     std::cout << a.Convert(degree) << "deg" << std::endl;

// }

volatile double length_input = .2;
volatile double voltage_output;

int main()
{
    typedef PID<ANGLE_DIMENSIONS, VOLTAGE_DIMENSIONS> TurnPID;
    TurnPID::pid_config_t turn_config = {
        .kP = 0.1_v / degree,
        .kI = 0.1_v / (degree * second),
        .kD = 0_v / (degree / second),
        .deadband = 1_deg};
    TurnPID *p = new PID<ANGLE_DIMENSIONS, VOLTAGE_DIMENSIONS>(turn_config);

    
    PID<>::pid_config_t normal_config = {
        .kP = 0.1,
        .kI = 0.1,
        .kD = 0,
        .deadband = 1};
    PID<> *p2 = new PID<>(normal_config);




    Feedback<ANGLE_DIMENSIONS, VOLTAGE_DIMENSIONS> &fb_cont = *p;
    fb_cont.set_target(355_deg);

    fb_cont.update(0 * degree);
    Angle err = p->get_error();
    Voltage v = fb_cont.calculate();
    voltage_output = v.getValue();
    std::cout << "err1: " << err.Convert(degree) << std::endl;
    std::cout << "out1: " << voltage_output << std::endl;

    fb_cont.update((double)0 * degree);
    Angle err2 = p->get_error();
    Voltage v2 = fb_cont.calculate();
    voltage_output = v2.getValue();

    std::cout << "err2: " << err2.Convert(degree) << std::endl;
    std::cout << "out2: " << voltage_output << std::endl;
}
