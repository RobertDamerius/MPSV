#pragma once


#include <mpsv/common/Common.hpp>


namespace mpsv {


namespace core {


/**
 * @brief The event class can be used to wait within a thread for an event that is notified by another thread.
 */
class Event {
    public:
        /**
         * @brief Create a new event object.
         */
        Event() noexcept {
            #if defined(_WIN32)
            notified = false;
            #elif defined(__linux__)
            sem_init(&sem, 0, 0);
            #endif
        }

        ~Event() noexcept {
            #if defined(__linux__)
            sem_destroy(&sem);
            #endif
        }

        /**
         * @brief Notify a thread waiting for this event.
         */
        void Notify(void) noexcept {
            #if defined(_WIN32)
            std::unique_lock<std::mutex> lock(mtx);
            notified = true;
            cv.notify_one();
            #elif defined(__linux__)
            sem_post(&sem);
            #endif
        }

        /**
         * @brief Wait for a notification event. A thread calling this function waits until @ref Notify is called.
         */
        void Wait(void) noexcept {
            #if defined(_WIN32)
            std::unique_lock<std::mutex> lock(mtx);
            cv.wait(lock, [this](){ return this->notified; });
            notified = false;
            #elif defined(__linux__)
            while(sem_wait(&sem) == -1){
                if(errno != EINTR){
                    return;
                }
            }
            while(sem_trywait(&sem) == 0);
            #endif
        }

        /**
         * @brief Wait for a notification event or for a timeout. A thread calling this function waits until @ref Notify is called or the wait timed out.
         * @param[in] timeoutMs Timeout in milliseconds.
         */
        void WaitFor(uint32_t timeoutMs) noexcept {
            #if defined(_WIN32)
            std::unique_lock<std::mutex> lock(mtx);
            cv.wait_for(lock, std::chrono::milliseconds(timeoutMs), [this](){ return this->notified; });
            notified = false;
            #elif defined(__linux__)
            struct timespec ts;
            if (clock_gettime(CLOCK_MONOTONIC, &ts) == -1) {
                return;
            }
            ts.tv_sec += timeoutMs / 1000;
            ts.tv_nsec += (timeoutMs % 1000) * 1000000;
            if(ts.tv_nsec >= 1000000000){
                ts.tv_sec += 1;
                ts.tv_nsec -= 1000000000;
            }
            while(sem_clockwait(&sem, CLOCK_MONOTONIC, &ts) == -1){
                if(errno == EINTR){
                    continue;
                }
                if(errno == ETIMEDOUT){
                    return;
                }
                return;
            }
            while (sem_trywait(&sem) == 0);
            #endif
        }

        /**
         * @brief Clear a notified event.
         */
        void Clear(void) noexcept {
            #if defined(_WIN32)
            std::unique_lock<std::mutex> lock(mtx);
            notified = false;
            #elif defined(__linux__)
            while(sem_trywait(&sem) == 0);
            #endif
        }

        /* Make this class non-copyable */
        Event(const Event&) = delete;
        Event& operator=(const Event&) = delete;
        Event(Event&&) = delete;
        Event& operator=(Event&&) = delete;

    protected:
        #if defined(_WIN32)
        std::mutex mtx;               // The mutex used for event notification.
        std::condition_variable cv;   // The condition variable used for event notification.
        bool notified;                // A boolean flag to prevent spurious wakeups.
        #elif defined(__linux__)
        sem_t sem;                    // The semaphore used for event notification.
        #endif
};


} /* namespace: core */


} /* namespace: mpsv */

