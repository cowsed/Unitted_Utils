#pragma once
#include "units.h"

/**
 * point2d is a unit-ed 2 dimensional coordinate
 */
struct point2d_t
{
    unit::Length x;
    unit::Length y;

    point2d_t operator+=(point2d_t rhs);
    point2d_t operator-=(point2d_t rhs);



    unit::Length length() const;
    unit::Length distance(const point2d_t other) const;
    unit::Angle direction() const;
};

point2d_t operator+(point2d_t lhs, point2d_t rhs);
point2d_t operator-(point2d_t lhs, point2d_t rhs);

point2d_t operator*(point2d_t lhs, double rhs);
point2d_t operator*(double lhs, point2d_t rhs);

point2d_t operator/(point2d_t lhs, double rhs);
point2d_t operator/(double lhs, point2d_t rhs);

bool operator==(point2d_t lhs, point2d_t rhs);


/**
 * Standard way of representing a robots state on the field
 * position in 2D and heading (heading = 0 -> +X° , heading = 90°  -> +Y)
 */
struct pose2d_t
{
    point2d_t pos;
    unit::Angle heading;
};

/**
 * Calculates the smallest angle between two angles
 * non trivial as angles wrap at 2_pi
 */
unit::Angle smallest_angle(unit::Angle from, unit::Angle to);

// Wraps an angle from (-inf, +inf) to (0, 2_pi)
unit::Angle wrap_angle(unit::Angle input);
