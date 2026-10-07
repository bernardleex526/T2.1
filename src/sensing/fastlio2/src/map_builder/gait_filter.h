#pragma once
// ---------------------------------------------------------------------------
// Quadruped gait motion compensation - cascaded RBJ notch filters.
//
// WHY: a legged platform injects a periodic pitch/roll oscillation into the IMU
// (gait fundamental plus harmonics, typically in the 10-40 Hz band) and a
// high-frequency specific-force spike at foot contact.  The IESKF has no model
// for either, so it integrates the oscillation as if it were real rotation and
// the scan-to-map residual grows (the "layered / serrated cloud" symptom).  A
// notch cascade removes the periodic component at its source, before the
// sample reaches the filter state.
//
// DESIGN NOTES (deviations from the review report's sketch, each deliberate):
//   1. Only 2 past inputs/outputs per stage are kept (Direct Form II
//      transposed).  The report's sketch kept unbounded std::deque histories,
//      which a biquad does not need.
//   2. Axis selection is a pass-through mask, not "filter z then overwrite z
//      with the raw value".  The latter runs the yaw axis through the filter
//      state for no reason; here an unselected axis never touches the state.
//   3. Group delay is EXPOSED (groupDelaySeconds()) instead of being ignored.
//      An IIR notch delays the signal it filters; if the gyro is filtered and
//      the accel is not, the two streams acquire a relative time skew, and
//      gyro/accel alignment is exactly what the pre-integration relies on.
//      Filtering BOTH with identical coefficients gives them the SAME delay and
//      preserves alignment, which is why filter_accel_* is offered at all.
//   4. Configuration is validated (validate()) rather than trusted.  A notch at
//      or above Nyquist, or a non-positive Q, silently produces a filter that
//      does nothing or blows up.
//
// SCOPE / HONESTY: this header is a signal-processing primitive.  It contains
// no measured gait frequency, no measured IMU sample rate and no claim that a
// given notch set improves any metric on any platform.  Both must be derived on
// the target from a recorded walk (see docs/tuning_guide.md and
// tools/imu_gait_spectrum.py); until then the feature stays disabled and the
// node behaves exactly as before.
//
// Header-only and free of ROS/PCL so the behaviour can be regression-tested
// without a live sensor (test/test_gait_filter.cpp), matching the existing
// imu_init.h / gate_math.h precedent.
// ---------------------------------------------------------------------------

#include <Eigen/Eigen>

#include <cmath>
#include <complex>
#include <cstddef>
#include <string>
#include <vector>

// M_PI is not part of the C++ standard; glibc exposes it via <cmath> but MSVC
// needs _USE_MATH_DEFINES first.  Define it ourselves when absent so this header
// compiles under either toolchain.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace gait
{

// Number of axes in a gyro/accel triple.  Fixed by the IMU message layout.
constexpr int kAxes = 3;

// One second-order section, normalised so a0 == 1.
struct BiquadCoeffs
{
    double b0 = 1.0, b1 = 0.0, b2 = 0.0, a1 = 0.0, a2 = 0.0;
};

// Direct Form II transposed state for a single axis.
struct BiquadState
{
    double s1 = 0.0, s2 = 0.0;
};

// RBJ audio-EQ-cookbook notch at f0 with quality factor q, for sample rate fs.
// Returns false when the arguments cannot produce a usable filter; out is then
// left as the identity (pass-through) section.
inline bool makeNotch(double f0_hz, double q, double fs_hz, BiquadCoeffs &out)
{
    if (!(fs_hz > 0.0) || !(q > 0.0))
        return false;
    // The notch centre must be strictly inside (0, Nyquist): f0 == 0 has no
    // solution, f0 == Nyquist collapses to a zero at z = -1 and a pole at the
    // same place, i.e. a pass-through, and f0 > Nyquist aliases to a frequency
    // the user did not ask for.
    if (!(f0_hz > 0.0) || !(f0_hz < 0.5 * fs_hz))
        return false;

    const double w0 = 2.0 * M_PI * f0_hz / fs_hz;
    const double cos_w0 = std::cos(w0);
    const double alpha = std::sin(w0) / (2.0 * q);

    const double a0 = 1.0 + alpha;
    if (!(a0 > 0.0))
        return false;

    out.b0 = 1.0 / a0;
    out.b1 = -2.0 * cos_w0 / a0;
    out.b2 = 1.0 / a0;
    out.a1 = -2.0 * cos_w0 / a0;
    out.a2 = (1.0 - alpha) / a0;
    return true;
}

// Complex magnitude of a normalised biquad at normalised frequency w [rad/sample].
inline double biquadMagnitude(const BiquadCoeffs &c, double w)
{
    const std::complex<double> z1 = std::polar(1.0, -w);
    const std::complex<double> z2 = z1 * z1;
    const std::complex<double> num = c.b0 + c.b1 * z1 + c.b2 * z2;
    const std::complex<double> den = 1.0 + c.a1 * z1 + c.a2 * z2;
    const double d = std::abs(den);
    return d > 0.0 ? std::abs(num) / d : 0.0;
}

// Phase [rad] of a normalised biquad at normalised frequency w.
inline double biquadPhase(const BiquadCoeffs &c, double w)
{
    const std::complex<double> z1 = std::polar(1.0, -w);
    const std::complex<double> z2 = z1 * z1;
    const std::complex<double> num = c.b0 + c.b1 * z1 + c.b2 * z2;
    const std::complex<double> den = 1.0 + c.a1 * z1 + c.a2 * z2;
    if (std::abs(num) == 0.0 || std::abs(den) == 0.0)
        return 0.0;
    return std::arg(num) - std::arg(den);
}

// Cascade of notches applied to a 3-axis signal, with an explicit per-axis
// pass-through mask.
class GaitNotchFilter
{
public:
    // Which axes of each triple are filtered.  Defaults keep the feature inert
    // and reproduce the historical behaviour byte for byte until a caller
    // opts in: nothing is filtered unless at least one mask bit is set.
    struct Options
    {
        std::vector<double> notch_freq_hz;   // centres, one section each
        double q = 10.0;                     // quality factor, > 0
        double sample_rate_hz = 200.0;       // IMU rate, must exceed 2*max(f0)
        bool filter_gyro_x = true;
        bool filter_gyro_y = true;
        bool filter_gyro_z = false;          // yaw: not a gait-oscillation axis
        bool filter_accel_x = false;
        bool filter_accel_y = false;
        bool filter_accel_z = false;
    };

    // Verdict of validate(): what was accepted, what was refused and why.
    struct Validation
    {
        bool ok = false;                     // at least one usable section
        std::vector<double> accepted_freq_hz;
        std::vector<double> rejected_freq_hz;
        std::vector<std::string> errors;     // fatal: nothing usable
        std::vector<std::string> warnings;   // non-fatal, still worth printing
    };

    // Pure check of an Options set.  Never mutates the filter.
    static Validation validate(const Options &opt)
    {
        Validation v;
        if (!(opt.sample_rate_hz > 0.0))
        {
            v.errors.push_back("sample_rate_hz must be > 0");
            return v;
        }
        if (!(opt.q > 0.0))
        {
            v.errors.push_back("q must be > 0");
            return v;
        }
        if (opt.notch_freq_hz.empty())
        {
            v.warnings.push_back("notch_freq_hz is empty: no section configured, filter is a pass-through");
            return v;
        }

        const double nyquist = 0.5 * opt.sample_rate_hz;
        for (double f : opt.notch_freq_hz)
        {
            if (f > 0.0 && f < nyquist)
                v.accepted_freq_hz.push_back(f);
            else
            {
                v.rejected_freq_hz.push_back(f);
                v.warnings.push_back("notch at " + std::to_string(f) +
                                     " Hz is outside (0, Nyquist=" + std::to_string(nyquist) +
                                     ") Hz and was dropped");
            }
        }

        // Bandwidth sanity: the -3 dB width of an RBJ notch is about f0/q.  A
        // width wider than the spacing between adjacent centres means the
        // sections overlap into a broad band-stop that will also eat genuine
        // motion at those frequencies.
        for (std::size_t i = 0; i < v.accepted_freq_hz.size(); ++i)
        {
            const double bw = v.accepted_freq_hz[i] / opt.q;
            for (std::size_t j = i + 1; j < v.accepted_freq_hz.size(); ++j)
            {
                const double gap = std::fabs(v.accepted_freq_hz[j] - v.accepted_freq_hz[i]);
                if (gap < 0.5 * (bw + v.accepted_freq_hz[j] / opt.q))
                    v.warnings.push_back("notches at " + std::to_string(v.accepted_freq_hz[i]) +
                                         " and " + std::to_string(v.accepted_freq_hz[j]) +
                                         " Hz overlap (bandwidth " + std::to_string(bw) +
                                         " Hz): the cascade removes a band wider than intended");
            }
        }

        v.ok = !v.accepted_freq_hz.empty();
        if (!v.ok)
            v.errors.push_back("no usable notch frequency after validation");
        return v;
    }

    GaitNotchFilter() = default;

    // Builds the cascade.  Returns validate().ok, i.e. false when no section
    // survived; the object is then left inert (filter() is a pass-through).
    bool configure(const Options &opt)
    {
        m_opt = opt;
        m_sections.clear();
        m_masks = Masks{};
        m_valid = false;

        const Validation v = validate(opt);
        m_validation = v;
        if (!v.ok)
            return false;

        for (double f : v.accepted_freq_hz)
        {
            BiquadCoeffs c;
            if (makeNotch(f, opt.q, opt.sample_rate_hz, c))
            {
                Section s;
                s.c = c;
                m_sections.push_back(s);
            }
        }
        if (m_sections.empty())
            return false;

        m_masks.gyro[0] = opt.filter_gyro_x;
        m_masks.gyro[1] = opt.filter_gyro_y;
        m_masks.gyro[2] = opt.filter_gyro_z;
        m_masks.accel[0] = opt.filter_accel_x;
        m_masks.accel[1] = opt.filter_accel_y;
        m_masks.accel[2] = opt.filter_accel_z;

        m_valid = true;
        return true;
    }

    // True once a usable cascade is installed.  When false every filter*() call
    // returns its input unchanged.
    bool valid() const { return m_valid; }

    int sectionCount() const { return static_cast<int>(m_sections.size()); }

    const Validation &validation() const { return m_validation; }

    // Drops the integrator state.  Call on stream discontinuity (out-of-order
    // message, buffer flush, re-initialization): the state describes the sample
    // sequence that just ended, not the one that follows.
    void reset()
    {
        for (Section &s : m_sections)
            for (int a = 0; a < kAxes; ++a)
                s.state[a] = BiquadState{};
    }

    Eigen::Vector3d filterGyro(const Eigen::Vector3d &in) { return apply(in, m_masks.gyro); }

    Eigen::Vector3d filterAccel(const Eigen::Vector3d &in) { return apply(in, m_masks.accel); }

    // Group delay [s] of the whole cascade at freq_hz, from the analytic phase
    // response (central difference).  Reported so the caller can state the time
    // skew it introduces instead of guessing: if gyro and accel are filtered
    // with the same coefficients their delays cancel and the relative skew is
    // zero; if only one of them is filtered, the skew is this value.
    double groupDelaySeconds(double freq_hz) const
    {
        if (!m_valid || !(m_opt.sample_rate_hz > 0.0) || !(freq_hz > 0.0) || !(freq_hz < 0.5 * m_opt.sample_rate_hz))
            return 0.0;

        const double w = 2.0 * M_PI * freq_hz / m_opt.sample_rate_hz;
        const double dw = 1e-6;
        double total = 0.0;
        for (const Section &s : m_sections)
            total += -(biquadPhase(s.c, w + dw) - biquadPhase(s.c, w - dw)) / (2.0 * dw);
        return total / m_opt.sample_rate_hz;
    }

    // Cascade magnitude response at freq_hz (1.0 = untouched).  Exposed for the
    // regression test and for the "did the notch actually notch?" check.
    double magnitudeAt(double freq_hz) const
    {
        if (!m_valid || !(m_opt.sample_rate_hz > 0.0) || freq_hz < 0.0 || freq_hz > 0.5 * m_opt.sample_rate_hz)
            return 1.0;
        const double w = 2.0 * M_PI * freq_hz / m_opt.sample_rate_hz;
        double mag = 1.0;
        for (const Section &s : m_sections)
            mag *= biquadMagnitude(s.c, w);
        return mag;
    }

private:
    struct Section
    {
        BiquadCoeffs c{};
        BiquadState state[kAxes] = {};
    };

    struct Masks
    {
        bool gyro[kAxes] = {false, false, false};
        bool accel[kAxes] = {false, false, false};
    };

    Eigen::Vector3d apply(const Eigen::Vector3d &in, const bool *mask)
    {
        if (!m_valid)
            return in;
        Eigen::Vector3d out = in;
        for (int a = 0; a < kAxes; ++a)
        {
            if (!mask[a])
                continue; // pass-through: state untouched, no phase added
            double x = in[a];
            for (Section &s : m_sections)
            {
                BiquadState &st = s.state[a];
                const double y = s.c.b0 * x + st.s1;
                st.s1 = s.c.b1 * x - s.c.a1 * y + st.s2;
                st.s2 = s.c.b2 * x - s.c.a2 * y;
                x = y;
            }
            out[a] = x;
        }
        return out;
    }

    Options m_opt{};
    std::vector<Section> m_sections;
    Masks m_masks{};
    Validation m_validation{};
    bool m_valid = false;
};

} // namespace gait
