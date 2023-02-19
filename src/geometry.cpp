#include "geometry.h"

unit::Length point2d_t::length() const
{
    return sqrt(x * x + y * y);
}

unit::Length point2d_t::distance(const point2d_t other) const
{
    return (*this - other).length();
}
unit::Angle point2d_t::direction() const
{
    return atan2(y, x);
}
// +
point2d_t point2d_t::operator+=(const point2d_t rhs){
    return {.x = x + rhs.x, .y = y + rhs.y};
}

// -
point2d_t point2d_t::operator-=(const point2d_t rhs){
    return {.x = x - rhs.x, .y = y - rhs.y};
}

// +
point2d_t operator+(const point2d_t lhs, const point2d_t rhs){
    return {.x = lhs.x + rhs.x, .y = lhs.y + rhs.y};
}

// -
point2d_t operator-(const point2d_t lhs, const point2d_t rhs){
    return {.x = lhs.x - rhs.x, .y = lhs.y - rhs.y};
}

// *
point2d_t operator*(const point2d_t lhs, const double rhs){
    return {.x = lhs.x * rhs, .y = lhs.y * rhs};
}
point2d_t operator*(const double lhs, const point2d_t rhs){
    return {.x = rhs.x * lhs, .y = rhs.y * lhs};
}

// /
point2d_t operator/(point2d_t lhs, double rhs){
    return {.x = lhs.x / rhs, .y = lhs.y / rhs};
}
point2d_t operator/(double lhs, point2d_t rhs){
    return {.x = rhs.x / lhs, .y = rhs.y / lhs};
}
// ==
bool operator==(point2d_t lhs, point2d_t rhs){
    return lhs.x == rhs.x && lhs.y == rhs.y;
}


/**
 * Calculates the smallest angle between two angles
 * non trivial as angles wrap at 2_pi
 */
unit::Angle smallest_angle(unit::Angle from, unit::Angle to)
{
    using namespace unit_literals;
    unit::Angle retval;
    // get the difference between 0 and 2_pi
    retval = fmod(to - from, 2_pi * unit::radian);
    if (retval < 0_rad)
        retval += 2_pi * unit::radian;

    // Get the closest angle, now between -pi (turn left) and +pi (turn right)
    if (retval > 1_pi * unit::radian)
        retval -= 2_pi * unit::radian;

    return retval;
}

// Wraps an angle from (-inf, +inf) to (0, 2_pi)
unit::Angle wrap_angle(unit::Angle input)
{
    using namespace unit_literals;

    unit::Angle angle = fmod(input, 2_pi * unit::radian);
    if (angle < 0_rad)
        angle += 2_pi * unit::radian;

    return angle;
}