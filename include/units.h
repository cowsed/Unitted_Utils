#pragma once
#include <iostream>
#include <string>
#include <math.h>

// https://benjaminjurke.com/content/articles/2015/compile-time-numerical-unit-dimension-checking/#c11-string-literals

// static_assert(M1!=M2 || L1!=L2 || T1!=T2 || A1!=A2 || C1!=C2, "Mass dimension should match");
//  if we want better error messages, take template signature from * but apply to +, - and have that assert

/// units header comment
namespace unit
{

    /// Base Quantity class
    /// Don't use this by itself unless you have a really novel type, use a type made with  the typedefs lower down
    template <int MassDim1, int LengthDim1, int TimeDim1, int AngleDim1, int CurrentDim1>
    class Quantity
    {
    private:
        double value;

    public:
        constexpr Quantity() : value(0.0) {}
        constexpr Quantity(double val) : value(val) {}

        //+=
        template <int MassDim2, int LengthDim2, int TimeDim2, int AngleDim2, int CurrentDim2>
        constexpr Quantity const &operator+=(const Quantity<MassDim2, LengthDim2, TimeDim2, AngleDim2, CurrentDim2> &rhs)
        {
            static_assert(((MassDim1 == MassDim2) && (LengthDim1 == LengthDim2) && (TimeDim1 == TimeDim2) && (AngleDim1 == AngleDim2) && (CurrentDim1 == CurrentDim2)), "cannot assign incompatible units (unit1) += (unit2)");
            value += rhs.getValue();
            return *this;
        }
        // -=
        template <int MassDim2, int LengthDim2, int TimeDim2, int AngleDim2, int CurrentDim2>
        constexpr Quantity const &operator-=(const Quantity<MassDim2, LengthDim2, TimeDim2, AngleDim2, CurrentDim2> &rhs)
        {
            static_assert(((MassDim1 == MassDim2) && (LengthDim1 == LengthDim2) && (TimeDim1 == TimeDim2) && (AngleDim1 == AngleDim2) && (CurrentDim1 == CurrentDim2)), "cannot assign incompatible units (unit1) -= (unit2)");
            value -= rhs.getValue();
            return *this;
        }

        // - (negation, not subtraction)
        constexpr Quantity const &operator-()
        {
            value = -value;
            return *this;
        }

        /// convert to a double in the units specified
        constexpr double Convert(const Quantity &rhs) const
        {
            return value / rhs.value;
        }

        /// Get raw stored value
        constexpr double getValue() const
        {
            return value;
        }

        constexpr void printRepresentation() const
        {
            if (MassDim1 != 0)
            {
                printf("M^%d ", MassDim1);
            }
            if (LengthDim1 != 0)
            {
                printf("L^%d ", LengthDim1);
            }
            if (TimeDim1 != 0)
            {
                printf("T^%d ", TimeDim1);
            }
            if (AngleDim1 != 0)
            {
                printf("A^%d ", AngleDim1);
            }
            if (CurrentDim1 != 0)
            {
                printf("I^%d ", CurrentDim1);
            }
        }
    };

    // Giving Names
#define NUMBER_DIMENSIONS 0, 0, 0, 0, 0
#define MASS_DIMENSIONS 1, 0, 0, 0, 0
#define LENGTH_DIMENSIONS 0, 1, 0, 0, 0
#define AREA_DIMENSIONS 0, 2, 0, 0, 0
#define VOLUME_DIMENSIONS 0, 2, 0, 0, 0
#define TIME_DIMENSIONS 0, 0, 1, 0, 0

#define SPEED_DIMENSIONS 0, 1, -1, 0, 0
#define ACCELERATION_DIMENSIONS 0, 1, -2, 0, 0

#define FREQUENCY_DIMENSIONS 0, 0, -1, 0, 0
#define FORCE_DIMENSIONS 1, 1, -2, 0, 0

#define ANGLE_DIMENSIONS 0, 0, 0, 1, 0
#define ANGULAR_SPEED_DIMENSIONS 0, 0, -1, 1, 0

#define CURRENT_DIMENSIONS 0, 0, 0, 0, 1
#define VOLTAGE_DIMENSIONS 1, 2, -3, 0, 1

    typedef Quantity<NUMBER_DIMENSIONS> Number;

    typedef Quantity<MASS_DIMENSIONS> Mass;

    typedef Quantity<LENGTH_DIMENSIONS> Length;
    typedef Quantity<AREA_DIMENSIONS> Area;
    typedef Quantity<VOLUME_DIMENSIONS> Volume;

    typedef Quantity<TIME_DIMENSIONS> Time;
    typedef Quantity<SPEED_DIMENSIONS> Speed;
    typedef Quantity<ACCELERATION_DIMENSIONS> Acceleration;

    typedef Quantity<FREQUENCY_DIMENSIONS> Frequency;
    typedef Quantity<FORCE_DIMENSIONS> Force;

    typedef Quantity<ANGLE_DIMENSIONS> Angle;
    typedef Quantity<ANGULAR_SPEED_DIMENSIONS> AngularSpeed;

    typedef Quantity<CURRENT_DIMENSIONS> Current;
    typedef Quantity<VOLTAGE_DIMENSIONS> Voltage;

    // Standard operators + - * / with 2 unitted quantities

    /// + operator
    template <int M, int L, int T, int A, int C>
    constexpr Quantity<M, L, T, A, C>
    operator+(const Quantity<M, L, T, A, C> &lhs, const Quantity<M, L, T, A, C> &rhs)
    {
        return Quantity<M, L, T, A, C>(lhs.getValue() + rhs.getValue());
    }

    /// - operator
    template <int M1, int L1, int T1, int A1, int C1,
              int M2, int L2, int T2, int A2, int C2>
    constexpr Quantity<M1, L1, T1, A1, C1>
    operator-(const Quantity<M1, L1, T1, A1, C1> &lhs, const Quantity<M2, L2, T2, A2, C2> &rhs)
    {
        static_assert(((M1 == M2) && (L1 == L2) && (T1 == T2) && (A1 == A2) && (C1 == C2)), "unit error. can not do operation (-) on (unit 1) - (unit 2). See template error for more info");
        return Quantity<M1, L1, T1, A1, C1>(lhs.getValue() - rhs.getValue());
    }

    /// * operator
    template <int M1, int L1, int T1, int A1, int C1,
              int M2, int L2, int T2, int A2, int C2>
    constexpr Quantity<M1 + M2, L1 + L2, T1 + T2, A1 + A2, C1 + C2>
    operator*(const Quantity<M1, L1, T1, A1, C1> &lhs, const Quantity<M2, L2, T2, A2, C2> &rhs)
    {
        return Quantity<M1 + M2, L1 + L2, T1 + T2, A1 + A2, C1 + C2>(lhs.getValue() * rhs.getValue());
    }

    /// / operator
    template <int M1, int L1, int T1, int A1, int C1,
              int M2, int L2, int T2, int A2, int C2>
    constexpr Quantity<M1 - M2, L1 - L2, T1 - T2, A1 - A2, C1 - C2>
    operator/(const Quantity<M1, L1, T1, A1, C1> &lhs, const Quantity<M2, L2, T2, A2, C2> &rhs)
    {
        return Quantity<M1 - M2, L1 - L2, T1 - T2, A1 - A2, C1 - C2>(lhs.getValue() / rhs.getValue());
    }

    // Scalar multiplication and division

    // Multiplication
    // scalar * unit
    template <int M, int L, int T, int A, int C>
    constexpr Quantity<M, L, T, A, C>
    operator*(const double &lhs, const Quantity<M, L, T, A, C> &rhs)
    {
        return Quantity<M, L, T, A, C>(lhs * rhs.getValue());
    }

    // unit * scalar
    template <int M, int L, int T, int A, int C>
    constexpr Quantity<M, L, T, A, C>
    operator*(const Quantity<M, L, T, A, C> &lhs, const double &rhs)
    {
        return Quantity<M, L, T, A, C>(lhs.getValue() * rhs);
    }

    // Division
    // scalar / unit
    template <int M, int L, int T, int A, int C>
    constexpr Quantity<M, L, T, A, C>
    operator/(const double &lhs, const Quantity<M, L, T, A, C> &rhs)
    {
        return Quantity<-M, -L, -T, -A, -C>(lhs / rhs.getValue());
    }

    // unit / scalar
    template <int M, int L, int T, int A, int C>
    constexpr Quantity<M, L, T, A, C>
    operator/(const Quantity<M, L, T, A, C> &lhs, const double &rhs)
    {
        return Quantity<M, L, T, A, C>(lhs.getValue() / rhs);
    }

    // Comparisons
    // Comparison operators for quantities:
    // ------------------------------------
    template <int M, int L, int T, int A, int I>
    constexpr bool operator==(const Quantity<M, L, T, A, I> &lhs, const Quantity<M, L, T, A, I> &rhs)
    {
        return (lhs.getValue() == rhs.getValue());
    }
    template <int M, int L, int T, int A, int I>
    constexpr bool operator!=(const Quantity<M, L, T, A, I> &lhs, const Quantity<M, L, T, A, I> &rhs)
    {
        return (lhs.getValue() != rhs.getValue());
    }
    template <int M, int L, int T, int A, int I>
    constexpr bool operator<=(const Quantity<M, L, T, A, I> &lhs, const Quantity<M, L, T, A, I> &rhs)
    {
        return (lhs.getValue() <= rhs.getValue());
    }
    template <int M, int L, int T, int A, int I>
    constexpr bool operator>=(const Quantity<M, L, T, A, I> &lhs, const Quantity<M, L, T, A, I> &rhs)
    {
        return (lhs.getValue() >= rhs.getValue());
    }
    template <int M, int L, int T, int A, int I>
    constexpr bool operator<(const Quantity<M, L, T, A, I> &lhs, const Quantity<M, L, T, A, I> &rhs)
    {
        return (lhs.getValue() < rhs.getValue());
    }
    template <int M, int L, int T, int A, int I>
    constexpr bool operator>(const Quantity<M, L, T, A, I> &lhs, const Quantity<M, L, T, A, I> &rhs)
    {
        return (lhs.getValue() > rhs.getValue());
    }

    // Predefined units
    // ----------------

    // Mass
    constexpr Mass kg(1.0);
    constexpr Mass gram = 0.001 * kg;
    constexpr Mass ounce = 0.028349523125 * kg;
    constexpr Mass pound = 16 * ounce;

    // Length
    constexpr Length meter(1.0);
    constexpr Length centimeter = meter / 100.0;
    constexpr Length millimeter = meter / 1000.0;
    constexpr Length kilometer = meter * 1000.0;
    constexpr Length inch = 2.54 * centimeter;
    constexpr Length foot = 12 * inch;
    constexpr Length yard = 3 * foot;
    constexpr Length mile = 5280 * foot;

    // Area
    constexpr Area kilometer2 = kilometer * kilometer;
    constexpr Area meter2 = meter * meter;
    constexpr Area centimeter2 = centimeter * centimeter;
    constexpr Area millimeter2 = millimeter * millimeter;
    constexpr Area inch2 = inch * inch;
    constexpr Area foot2 = foot * foot;
    constexpr Area yard2 = yard * yard;

    // Time
    constexpr Time second(1.0);
    constexpr Time millisecond = second / 1000.0;
    constexpr Time minute = second * 60.0;
    constexpr Time hour = minute * 60.0;

    // Frequency
    constexpr Frequency hz(1.0);

    // Velocity
    constexpr Speed meter_per_second = meter / second;
    constexpr Speed inch_per_second = inch / second;
    constexpr Speed foot_per_second = foot / second;

    // Acceleration
    constexpr Acceleration meter_per_second2 = meter / (second * second);
    constexpr Acceleration inch_per_second2 = inch / (second * second);

    // Current
    constexpr Current amp(1.0);

    // Voltage
    constexpr Voltage volt = kg * meter * meter / (second * second * second) * amp;
    constexpr Voltage millivolt = volt / 1000.0;

    // Angular
    constexpr Angle radian(1.0);
    constexpr Angle degree = static_cast<double>(2 * M_PI / 360.0) * radian;
    constexpr Angle revolution = static_cast<double>(2 * M_PI) * radian;

    // Angular Speed
    constexpr AngularSpeed radian_per_sec = radian / second;
    constexpr AngularSpeed degree_per_sec = degree / second;
    constexpr AngularSpeed revolution_per_sec = revolution / second;

    // Typesafe trigonometric operations
    inline double sin(const Angle &num)
    {
        return std::sin(num.getValue());
    }
    inline double cos(const Angle &num)
    {
        return std::cos(num.getValue());
    }
    inline double tan(const Angle &num)
    {

        return std::tan(num.getValue());
    }

    // typesafe sign
    template <int M, int L, int T, int A, int I>
    constexpr Number sign(Quantity<M, L, T, A, I> op)
    {
        if (op.getValue() >= 0)
        {
            return 1;
        }
        else
        {
            return -1;
        }
    }

    // typesafe sqrt
    template <int M, int L, int T, int A, int I>
    constexpr Quantity<M / 2, L / 2, T / 2, A / 2, I / 2> sqrt(Quantity<M, L, T, A, I> op)
    {
        static_assert((M % 2 == 0) && (L % 2 == 0) && (T % 2 == 0) && (A % 2 == 0) && (I % 2 == 0), "Can not square root if it would leave us with fractional dimensions. like what would that mean");
        return std::sqrt(op.getValue());
    }

}

namespace unit_literals
{
    // Length Literals
    constexpr unit::Length operator"" _mm(long double x) { return static_cast<double>(x) * unit::millimeter; }
    constexpr unit::Length operator"" _mm(unsigned long long int x) { return static_cast<double>(x) * unit::millimeter; }

    constexpr unit::Length operator"" _cm(long double x) { return static_cast<double>(x) * unit::centimeter; }
    constexpr unit::Length operator"" _cm(unsigned long long int x) { return static_cast<double>(x) * unit::centimeter; }

    constexpr unit::Length operator"" _m(long double x) { return static_cast<double>(x) * unit::meter; }
    constexpr unit::Length operator"" _m(unsigned long long int x) { return static_cast<double>(x) * unit::meter; }

    constexpr unit::Length operator"" _km(long double x) { return static_cast<double>(x) * unit::kilometer; }
    constexpr unit::Length operator"" _km(unsigned long long int x) { return static_cast<double>(x) * unit::meter; }

    constexpr unit::Length operator"" _in(long double x) { return static_cast<double>(x) * unit::inch; }
    constexpr unit::Length operator"" _in(unsigned long long int x) { return static_cast<double>(x) * unit::inch; }

    constexpr unit::Length operator"" _ft(long double x) { return static_cast<double>(x) * unit::foot; }
    constexpr unit::Length operator"" _ft(unsigned long long int x) { return static_cast<double>(x) * unit::foot; }

    // Speed Literals
    constexpr unit::Speed operator"" _mps(long double x) { return static_cast<double>(x) * unit::meter / unit::second; }
    constexpr unit::Speed operator"" _mps(unsigned long long int x) { return static_cast<double>(x) * unit::meter / unit::second; }

    constexpr unit::Speed operator"" _inps(long double x) { return static_cast<double>(x) * unit::inch / unit::second; }
    constexpr unit::Speed operator"" _inps(unsigned long long int x) { return static_cast<double>(x) * unit::inch / unit::second; }

    // Time Literal
    constexpr unit::Time operator"" _s(long double x) { return static_cast<double>(x) * unit::second; }
    constexpr unit::Time operator"" _s(unsigned long long int x) { return static_cast<double>(x) * unit::second; }

    constexpr unit::Time operator"" _ms(long double x) { return static_cast<double>(x) * unit::millisecond; }
    constexpr unit::Time operator"" _ms(unsigned long long int x) { return static_cast<double>(x) * unit::millisecond; }

    constexpr unit::Time operator"" _min(long double x) { return static_cast<double>(x) * unit::minute; }
    constexpr unit::Time operator"" _min(unsigned long long int x) { return static_cast<double>(x) * unit::minute; }

    // Angle Literal
    constexpr unit::Angle operator"" _rad(long double x) { return static_cast<double>(x) * unit::radian; }
    constexpr unit::Angle operator"" _rad(unsigned long long int x) { return static_cast<double>(x) * unit::radian; }

    constexpr unit::Angle operator"" _deg(long double x) { return static_cast<double>(x) * unit::degree; }
    constexpr unit::Angle operator"" _deg(unsigned long long int x) { return static_cast<double>(x) * unit::degree; }

    constexpr unit::Angle operator"" _rev(long double x) { return static_cast<double>(x) * unit::revolution; }
    constexpr unit::Angle operator"" _rev(unsigned long long int x) { return static_cast<double>(x) * unit::revolution; }

    constexpr unit::AngularSpeed operator"" _rpm(long double x) { return static_cast<double>(x) * unit::revolution_per_sec; }
    constexpr unit::AngularSpeed operator"" _rpm(unsigned long long int x) { return static_cast<double>(x) * unit::revolution_per_sec; }

    // Voltage Literal
    constexpr unit::Voltage operator"" _v(long double x) { return static_cast<double>(x) * unit::volt; }
    constexpr unit::Voltage operator"" _v(unsigned long long int x) { return static_cast<double>(x) * unit::volt; }

    constexpr unit::Voltage operator"" _mv(long double x) { return static_cast<double>(x) * unit::millivolt; }
    constexpr unit::Voltage operator"" _mv(unsigned long long int x) { return static_cast<double>(x) * unit::millivolt; }

    constexpr long double operator"" _pi(long double x)
    {
        return static_cast<double>(x) * 3.1415926535897932384626433832795;
    }
    constexpr long double operator"" _pi(unsigned long long int x)
    {
        return static_cast<double>(x) * 3.1415926535897932384626433832795;
    }
};

// Global overrides
// these things overload functions that are usually used elsewhere, to make this seemless they go outside the units namespace so you dont have to qualify them
template <int M, int L, int T, int A, int C>
constexpr unit::Quantity<M, L, T, A, C> fmod(unit::Quantity<M, L, T, A, C> me, unit::Quantity<M, L, T, A, C> other)
{
    return unit::Quantity<M, L, T, A, C>(fmod(me.getValue(), other.getValue()));
}

template <int M, int L, int T, int A, int C>
constexpr unit::Quantity<M, L, T, A, C> fabs(unit::Quantity<M, L, T, A, C> me)
{
    return unit::Quantity<M, L, T, A, C>(fabs(me.getValue()));
}

constexpr unit::Angle atan2(unit::Length y, unit::Length x)
{
    return atan2(y.getValue(), x.getValue()) * unit::radian;
}

template <int M, int L, int T, int A, int C>
constexpr unit::Quantity<M, L, T, A, C> clamp(unit::Quantity<M, L, T, A, C> value, unit::Quantity<M, L, T, A, C> low, unit::Quantity<M, L, T, A, C> high)
{
    if (value < low){
        return low;
    } else if (value > high){
        return high;
    } else{
    return value;
    }
}

template <int M, int L, int T, int A, int C>
constexpr unit::Quantity<M, L, T, A, C> max(unit::Quantity<M, L, T, A, C> a, unit::Quantity<M, L, T, A, C> b){
    if (a > b){
        return a;
    } else {
        return b;
    }
}
template <int M, int L, int T, int A, int C>
constexpr unit::Quantity<M, L, T, A, C> min(unit::Quantity<M, L, T, A, C> a, unit::Quantity<M, L, T, A, C> b){
    if (a < b){
        return a;
    } else {
        return b;
    }
}
