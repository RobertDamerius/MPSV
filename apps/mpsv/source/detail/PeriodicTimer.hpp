#pragma once


#include <Common.hpp>


/**
 * @brief This class represents a periodic timer that waits for timer interrupts. The periodic time is set when calling the @ref Start
 * member function. On windows, the lowest possible value is 1 millisecond.
 */
class PeriodicTimer {
    public:
        /**
         * @brief Create a new periodic timer object.
         */
        PeriodicTimer();

        /**
         * @brief Start the periodic timer.
         * @param[in] sampletime The sampletime in seconds.
         * @return True if success, false otherwise.
         * @details On Windows, the sampletime must not be less than 0.001!
         */
        bool Start(double sampletime);

        /**
         * @brief Stop the periodic timer.
         */
        void Stop(void);

        /**
         * @brief Wait for a tick event of the timer.
         * @return True if timer event was received successfully, false otherwise.
         * @details If timer is not created or was destroyed, false will be returned immediately.
         */
        bool WaitForTick(void);

    private:
        #ifdef _WIN32
        HANDLE hTimer;   // [Windows] Handle of internal timer object.
        #else
        int fdTimer;     // [Linux] File descriptor of internal timer object.
        #endif
};

