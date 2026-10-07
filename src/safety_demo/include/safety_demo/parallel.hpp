// OpenMP shim — keeps the directives readable at the call site and lets the package
// still build (serially) on a toolchain without OpenMP.
#ifndef SAFETY_DEMO_PARALLEL_HPP
#define SAFETY_DEMO_PARALLEL_HPP

#ifdef _OPENMP
#include <omp.h>
#endif

namespace safety_demo {
namespace parallel {

/**
 * @brief Emit an OpenMP directive, or nothing when built without OpenMP.
 *
 * Writing the pragmas through this macro avoids -Wunknown-pragmas noise in a
 * non-OpenMP build (the package compiles with -Wall) and keeps the serial
 * fallback compiling from the same source.
 *
 * Variadic so that clauses separated by top-level commas survive being passed
 * through; _Pragma needs a string literal, hence the stringify step.
 */
#ifdef _OPENMP
#define SAFETY_DEMO_OMP_STRINGIFY(...) #__VA_ARGS__
#define SAFETY_DEMO_OMP(...) _Pragma(SAFETY_DEMO_OMP_STRINGIFY(omp __VA_ARGS__))
#else
#define SAFETY_DEMO_OMP(...)
#endif

/**
 * @brief Point count below which a scan sweep stays on the calling thread.
 *
 * Scans arrive tens of milliseconds apart, so between callbacks the OpenMP team
 * has long since stopped spinning and gone to sleep. Every parallel region then
 * starts by waking it, which measured at roughly 190 us on a 12-core desktop —
 * far more than a 1081-point sweep costs in the first place (~15 us).
 *
 * Measured cost per scan at a 40 Hz cadence, 80% of beams inside zone reach,
 * 1 thread vs 4 (GCC 15, default OMP_WAIT_POLICY):
 *
 *    1081 pts:  114 us -> 249 us   parallel 2.2x SLOWER
 *    4000 pts:  400 us -> 352 us   break-even
 *    8000 pts:  780 us -> 439 us   parallel 1.8x faster
 *   32000 pts: 2820 us -> 1169 us  parallel 2.4x faster
 *
 * So the gate sits at the break-even point: ordinary 2D safety scanners (360 to
 * 1081 beams) run serially and pay nothing, while a dense cloud gets the team.
 *
 * If a deployment does run large scans continuously and has cores to spare,
 * setting OMP_WAIT_POLICY=active with GOMP_SPINCOUNT=infinite keeps the team hot
 * and cuts the wake-up entirely — at the price of every worker thread burning a
 * core full time, which is usually the wrong trade on a robot controller.
 */
inline constexpr int kMinPointsForParallelScan = 4096;

/**
 * @brief Fix the OpenMP team size used by the scan loops.
 * @param threads Team size; values <= 0 leave the runtime default in place.
 *
 * The scan loop runs inside a ROS callback, so an unbounded team (one thread per
 * core by default) both pays more fork/join overhead and competes with the
 * executor for cores.
 */
inline void setNumThreads(int threads) {
#ifdef _OPENMP
    if (threads > 0) {
        omp_set_num_threads(threads);
    }
#else
    (void)threads;
#endif
}

/**
 * @brief Team size the parallel loops will actually use (1 without OpenMP).
 */
inline int maxThreads() {
#ifdef _OPENMP
    return omp_get_max_threads();
#else
    return 1;
#endif
}

}  // namespace parallel
}  // namespace safety_demo

#endif  // SAFETY_DEMO_PARALLEL_HPP
