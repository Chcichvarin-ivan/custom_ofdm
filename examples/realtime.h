/*!
 * \file realtime.h
 * \brief Small helper to boost the calling thread to realtime scheduling.
 *
 * Primary path: uhd::set_thread_priority_safe(priority, realtime=true).
 *   This is the same mechanism UHD itself uses internally for its RX/TX
 *   worker threads. It attempts SCHED_FIFO via pthread and returns false
 *   (instead of throwing) if the OS refuses.
 *
 * Fallback path: raw pthread_setschedparam(SCHED_FIFO) + mlockall().
 *   Used if for some reason <uhd/utils/thread.hpp> isn't available.
 *
 * Non-root users need permission to raise the priority. Either run with
 * sudo (as the fun_ofdm examples already do), or add to
 * /etc/security/limits.conf:
 *
 *     @usrp - rtprio 99
 *     @usrp - memlock unlimited
 *
 * and add your user to the `usrp` group, then log out and back in.
 */

#ifndef VIDEO_REALTIME_H
#define VIDEO_REALTIME_H

#include <iostream>

// Try the UHD helper first -- it's what fun_ofdm ultimately depends on
// anyway, so the header is guaranteed to be installed.
#if __has_include(<uhd/utils/thread.hpp>)
#  include <uhd/utils/thread.hpp>
#  define VIDEO_HAS_UHD_THREAD 1
#elif __has_include(<uhd/utils/thread_priority.hpp>)
#  include <uhd/utils/thread_priority.hpp>
#  define VIDEO_HAS_UHD_THREAD 1
#else
#  define VIDEO_HAS_UHD_THREAD 0
#endif

#if !VIDEO_HAS_UHD_THREAD
#  include <pthread.h>
#  include <sched.h>
#  include <sys/mman.h>
#  include <cstring>
#  include <cerrno>
#endif

/*!
 * \brief Boost the calling thread to realtime priority.
 *
 * \param priority Normalized priority in [-1.0, +1.0]; 1.0 = max.
 *                 UHD maps this to SCHED_FIFO priorities internally.
 * \return true on success, false otherwise (a warning is printed either way).
 */
inline bool set_realtime_priority(float priority = 1.0f)
{
#if VIDEO_HAS_UHD_THREAD
    // UHD's safe variant: returns false instead of throwing on failure and
    // internally logs a warning explaining how to grant the permission.
    bool ok = uhd::set_thread_priority_safe(priority, /*realtime=*/true);
    if (ok) {
        std::cout << "[realtime] thread boosted to SCHED_FIFO "
                  << "(priority=" << priority << ")\n";
    } else {
        std::cerr << "[realtime] could not enable realtime scheduling -- "
                     "running with normal priority. See README for how to "
                     "grant CAP_SYS_NICE / rtprio to your user.\n";
    }
    return ok;
#else
    // Fallback: plain pthreads. Map [-1, 1] onto the SCHED_FIFO range.
    int min_prio = sched_get_priority_min(SCHED_FIFO);
    int max_prio = sched_get_priority_max(SCHED_FIFO);
    if (priority < -1.0f) priority = -1.0f;
    if (priority > +1.0f) priority = +1.0f;
    int target_prio = static_cast<int>(
        min_prio + (priority + 1.0f) * 0.5f * (max_prio - min_prio));

    struct sched_param sp;
    std::memset(&sp, 0, sizeof(sp));
    sp.sched_priority = target_prio;

    int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (rc != 0) {
        std::cerr << "[realtime] pthread_setschedparam failed: "
                  << std::strerror(rc)
                  << " -- running with normal priority.\n";
        return false;
    }

    // Lock pages into RAM so page faults don't stall the audio/radio path.
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "[realtime] mlockall failed: "
                  << std::strerror(errno)
                  << " (non-fatal).\n";
    }

    std::cout << "[realtime] SCHED_FIFO priority " << target_prio
              << " (range " << min_prio << ".." << max_prio << ")\n";
    return true;
#endif
}

#endif  // VIDEO_REALTIME_H
