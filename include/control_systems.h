#include "units.h"
#include "geometry.h"

using namespace unit_literals;


/**
 * Generic Abstract feedback class
 * pv : process variable - the quantity we are measuring with our sensors (ie. speed, position, angle)
 * output : the way we are controlling the system (ie. voltage, current)
*/
template <int pv_M = 0, int pv_L = 0, int pv_T = 0, int pv_A = 0, int pv_C = 0, int out_M = 0, int out_L = 0, int out_T = 0, int out_A = 0, int out_C = 0>
class Feedback
{   public:
    typedef unit::Quantity<pv_M, pv_L, pv_T, pv_A, pv_C> pv_type;
    typedef unit::Quantity<out_M, out_L, out_T, out_A, out_C> output_type;

    virtual void update(pv_type new_pv) = 0;
    virtual output_type calculate() = 0;
    virtual void set_target(pv_type new_pv) = 0;
    virtual bool is_on_target() = 0;
};

/**
 * Unit Checked PID class
 * https://en.wikipedia.org/wiki/PID_controller
*/
template <int pv_M=0, int pv_L=0, int pv_T=0, int pv_A=0, int pv_C=0, int out_M=0, int out_L=0, int out_T=0, int out_A=0, int out_C=0>
class PID : public Feedback<pv_M, pv_L, pv_T, pv_A, pv_C, out_M, out_L, out_T, out_A, out_C>
{

    typedef unit::Quantity<pv_M, pv_L, pv_T, pv_A, pv_C> pv_type;
    typedef unit::Quantity<pv_M, pv_L, pv_T - 1, pv_A, pv_C> derivative_pv_dt_type;
    typedef unit::Quantity<pv_M, pv_L, pv_T + 1, pv_A, pv_C> integral_pv_dt_type;
    typedef unit::Quantity<out_M, out_L, out_T, out_A, out_C> output_type;
    typedef unit::Quantity<out_M - pv_M, out_L - pv_L, out_T - pv_T, out_A - pv_A, out_C - pv_C> kP_type;
    typedef unit::Quantity<out_M - pv_M, out_L - pv_L, out_T - pv_T - 1, out_A - pv_A, out_C - pv_C> kI_type;
    typedef unit::Quantity<out_M - pv_M, out_L - pv_L, out_T - pv_T + 1, out_A - pv_A, out_C - pv_C> kD_type;

public:
    struct pid_config_t{
        kP_type kP;
        kI_type kI;
        kD_type kD;
        pv_type deadband;
        unit::Time on_target_time;
    };

    constexpr PID(pid_config_t config) : config(config), set_pt(0.0), last_pv(0.0), current_pv(0.0), derivative_err(0.0), integral_err(0.0) {}

    void update(pv_type pv)
    {
        unit::Time dt = 0.02 * unit::second;
        last_pv = current_pv;
        current_pv = pv;

        derivative_err = get_error() / dt;
        integral_err = integral_err + get_error() * dt;
    }
    pv_type get_error()
    {
        // if its an angle, use the angle error calculation function
        if constexpr(pv_M == 0 && pv_L == 0 && pv_T == 0 && pv_A == 1 && pv_C == 0){
            return smallest_angle(set_pt, current_pv);
        }
        // otherwise use the normal error calculation function
        return set_pt - current_pv;
    }
    output_type calculate()
    {
        return config.kP * get_error() + config.kI * integral_err + config.kD * derivative_err;
    }
    void set_target(pv_type new_sp)
    {
        set_pt = new_sp;
        derivative_err = 0.0;
        integral_err = 0.0;
    }
    bool is_on_target(){
        return fabs(get_error()) < config.deadband;
    }

private:
    pid_config_t config;
    pv_type set_pt;
    pv_type last_pv;
    pv_type current_pv;

    derivative_pv_dt_type derivative_err;
    integral_pv_dt_type integral_err;
};
