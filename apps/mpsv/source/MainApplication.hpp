#pragma once


#include <Common.hpp>
#include <Configuration.hpp>
#include <NetworkManager.hpp>
#include <Planner.hpp>
#include <mpsv/mpsv.hpp>


/**
 * @brief Represents the main application.
 */
class MainApplication {
    public:
        Configuration configuration;     // The configuration for the application.
        NetworkManager networkManager;   // The network manager that handles input and output messages.
        Planner planner;                 // The planner that solves motion planning problems.

        /**
         * @brief Run the main application.
         * @param[in] argc Number of arguments passed to the application.
         * @param[in] argv List of arguments passed to the application.
         */
        void Run(int argc, char** argv);

    private:
        mpsv::core::Event interruptEvent;   // Event that is being notified, if the application receives a corresponding interrupt signal.

        /**
         * @brief Print system information to the output.
         */
        void PrintSystemInfo(void);

        /**
         * @brief Print arguments that where passed to the application.
         * @param[in] argc Number of arguments passed to the application.
         * @param[in] argv List of arguments passed to the application.
         */
        void PrintArguments(int argc, char** argv);

        /**
         * @brief Redirect console prints (stdout, stderr) to a file.
         */
        void RedirectPrintsToFile(void);

        /**
         * @brief Make a protocol directory for prints.
         * @return True if success, false otherwise.
         */
        bool MakeProtocolDirectory(void);

        /**
         * @brief Keep the N latest protocol files.
         * @param[in] N Number of protocol files to keep.
         */
        void KeepNLatestProtocolFiles(uint32_t N);

        /**
         * @brief The signal handler that handles signals from OS.
         * @param[in] signal Signal value.
         */
        static void SignalHandler(int signal);

        /**
         * @brief Global termination function that is called if exception handling fails.
         * @details On linux, a backtrace to the error is printed if debugging is enabled.
         */
        static void TerminateHandler(void);
};


extern MainApplication mainApplication;

