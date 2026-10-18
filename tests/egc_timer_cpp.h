#ifndef EGC_TESTS_EGC_TIMER_CPP_H
#define EGC_TESTS_EGC_TIMER_CPP_H

#include <cstddef>
#include <functional>

#include "embedded-game-controller/egc_timer.h"

/* C++ wrapper for the embedded-game-controller API. Used for testing, but we
 * might make it part of embedded-game-controller, if someone needs it. */
namespace Egc {

class Timer {
public:
    using Callback = std::function<void(Timer &)>;
    Timer(u32 periodUs, Callback callback)
    {
        m_callback = callback;
        egc_timer_start(&m_timer, periodUs, &Timer::cCallback);
    }
    ~Timer()
    {
        stop();
    }

    void stop()
    {
        egc_timer_stop(&m_timer);
    }

private:
    static void cCallback(egc_timer_t *timer)
    {
        std::size_t offset = offsetof(Timer, m_timer);
        Timer *self = reinterpret_cast<Timer *>(reinterpret_cast<char *>(timer) - offset);
        self->m_callback(*self);
    }

private:
    egc_timer_t m_timer;
    Callback m_callback;
};

} /* namespace Egc */

#endif /* EGC_TESTS_EGC_TIMER_CPP_H */
