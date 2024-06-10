#pragma once


#include <Common.hpp>


/**
 * @brief Represents the arguments passed to the application.
 */
class ApplicationArguments {
    public:
        bool console;   // True if prints should be displayed in the console instead of redirecting them to a protocol file.
        bool help;      // True if help command was found and help page has been printed.

        /**
         * @brief Construct a new application arguments object and parse the arguments.
         * @param[in] argc Number of arguments passed to the application.
         * @param[in] argv Array of arguments passed to the application.
         */
        ApplicationArguments(int argc, char** argv);

    private:
        /**
         * @brief Print help text to the standard output.
         */
        void PrintHelp(void);
};

