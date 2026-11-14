#pragma once
// SymbioGrip physical plant: the single physics implementation shared by the SIL benchmark
// (test_bench.cpp) and the desktop human-in-the-loop bench (hil_bridge.cpp).
//
// Axial (x): carriage + hand + reflected rotor inertia on a GT2 belt rail, viscous + Coulomb friction,
//            brushed DC motor driven by a TB6612FNG (average-voltage model, short brake in PWM off-time;
//            coast = zero current). Electrical time constant L/R ~ 0.5 ms << 2 ms and is neglected.
// Transverse (y): stylus tip held by the hand (limb impedance about an equilibrium point), stiff
//            corridor walls around a sinusoidal wire. The motor acts on x only (1-DOF device).
// Sensors:   quadrature encoder floor-quantised at COUNTS_PER_METER; digital contact sampled at 500 Hz
//            with the same edge + refractory logic as the firmware.

#include <math.h>
#include <stdint.h>
#include "../src/config.h"
#include "../src/admittance.h"

struct PlantParams {
    // Axial mechanics
    float mass_x      = 1.02f;    // kg: carriage 0.12 + hand/forearm 0.20 + reflected rotor 0.70
    float b_fric      = B_FRICTION_EST;  // N*s/m
    float coulomb_n   = 0.20f;    // N, smoothed with tanh(v / 5 mm/s)
    // Transverse mechanics
    float mass_y      = 0.060f;   // kg, stylus + fingertip
    float k_wall      = 18000.0f; // N/m
    float b_wall      = 50.0f;    // N*s/m
    // Task geometry (known to the plant and to the human's eyes, never to the controller)
    float track_len   = 0.180f;   // m
    float corridor_r  = 0.00135f; // m, half-width (2.70 mm slot)
    float a_wire      = 0.0023f;  // m
    float lambda_wire = 0.060f;   // m
    // True motor (may be set different from the controller's model to test robustness)
    ActuatorParams motor = defaultActuatorParams();
};

// Hand action held constant over one 2 ms control period.
//   Fx = fx + kx (x_eq - x) - bx vx        Fy = fy + ky (y_eq - y) - by vy
struct HandInput {
    float fx = 0.0f, x_eq = 0.0f, kx = 0.0f, bx = 0.0f;
    float fy = 0.0f, y_eq = 0.0f, ky = 0.0f, by = 0.0f;
};

struct PlantState {
    double x = 0.0, vx = 0.0, y = 0.0, vy = 0.0;
    float motor_force = 0.0f, motor_current = 0.0f, max_abs_current = 0.0f;
    double motor_work_j = 0.0;       // exact work done BY the motor on the carriage (substep integral)
    bool contact_now = false;        // contact signal sampled at the last control tick
    bool contact_event = false;      // a debounced contact was counted at the last tick
    int32_t contacts = 0;
    int refractory = 0;
};

class Plant {
public:
    static const int SUBSTEPS = 8;   // 4 kHz physics under the 500 Hz controller

    explicit Plant(const PlantParams& p = PlantParams()) : p_(p) {
        k_wave_ = 2.0f * 3.14159265358979f / p_.lambda_wire;
        reset();
    }

    void reset() { s_ = PlantState(); }

    const PlantParams& params() const { return p_; }
    const PlantState& state() const { return s_; }

    float wireY(double x) const     { return p_.a_wire * (float)sin(k_wave_ * x); }
    float wireSlope(double x) const { return p_.a_wire * k_wave_ * (float)cos(k_wave_ * x); }
    float wireCurv(double x) const  { return -p_.a_wire * k_wave_ * k_wave_ * (float)sin(k_wave_ * x); }
    float waveNumber() const        { return k_wave_; }
    float lateralError() const      { return (float)(s_.y - wireY(s_.x)); }
    bool  finished() const          { return s_.x >= p_.track_len; }

    int32_t encoderCounts() const {
        return (int32_t)floor(s_.x * (double)COUNTS_PER_METER);
    }

    // Advance one control period with zero-order-held hand action and motor command.
    void advance(const HandInput& h, const MotorCommand& cmd) {
        const double dt = (double)CONTROL_DT_SEC / SUBSTEPS;
        bool touched = false;
        for (int i = 0; i < SUBSTEPS; ++i) {
            // Motor
            float current = 0.0f;
            if (!cmd.coast) {
                const float volts = cmd.duty * p_.motor.v_supply;
                current = (volts - p_.motor.kt_eff * (float)s_.vx) / p_.motor.r_ohm;
            }
            const float f_motor = p_.motor.kt_eff * current;
            s_.motor_force = f_motor;
            s_.motor_current = current;
            if (fabsf(current) > s_.max_abs_current) s_.max_abs_current = fabsf(current);

            // Axial
            const double f_fric = -p_.b_fric * s_.vx - p_.coulomb_n * tanh(s_.vx / 0.005);
            const double f_hand_x = h.fx + h.kx * (h.x_eq - s_.x) - h.bx * s_.vx;
            const double ax = (f_hand_x + f_motor + f_fric) / p_.mass_x;

            // Transverse
            const double err = s_.y - wireY(s_.x);
            double f_wall = 0.0;
            if (fabs(err) >= p_.corridor_r) {
                const double pen = fabs(err) - p_.corridor_r;
                f_wall = -p_.k_wall * pen * (err > 0 ? 1.0 : -1.0) - p_.b_wall * s_.vy;
                touched = true;
            }
            const double f_hand_y = h.fy + h.ky * (h.y_eq - s_.y) - h.by * s_.vy;
            const double ay = (f_hand_y + f_wall) / p_.mass_y;

            // Semi-implicit Euler
            s_.vx += ax * dt;  s_.x += s_.vx * dt;
            s_.motor_work_j += (double)f_motor * s_.vx * dt;
            s_.vy += ay * dt;  s_.y += s_.vy * dt;
        }

        // Contact logic identical to firmware: 500 Hz sample, rising edge, 16 ms refractory.
        if (s_.refractory > 0) s_.refractory--;
        s_.contact_event = false;
        if (touched && !s_.contact_now && s_.refractory == 0) {
            s_.contacts++;
            s_.contact_event = true;
            s_.refractory = CONTACT_REFRACTORY_TICKS;
        }
        s_.contact_now = touched;
    }

    // Verification helper: carriage follows a prescribed trajectory (infinitely stiff hand), moving
    // linearly to x_target over one control period; the motor's work is integrated at substep level.
    void prescribe(double x_target, const MotorCommand& cmd) {
        const double dt = (double)CONTROL_DT_SEC / SUBSTEPS;
        const double v = (x_target - s_.x) / (double)CONTROL_DT_SEC;
        for (int i = 0; i < SUBSTEPS; ++i) {
            float current = 0.0f;
            if (!cmd.coast) {
                current = (cmd.duty * p_.motor.v_supply - p_.motor.kt_eff * (float)v) / p_.motor.r_ohm;
            }
            const float f_motor = p_.motor.kt_eff * current;
            s_.motor_force = f_motor;
            s_.motor_current = current;
            s_.vx = v;
            s_.x += v * dt;
            s_.motor_work_j += (double)f_motor * v * dt;
        }
        s_.x = x_target;
    }

private:
    PlantParams p_;
    PlantState s_;
    float k_wave_;
};
