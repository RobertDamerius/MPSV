#pragma once


#include <Common.hpp>
#include <NetworkConfiguration.hpp>
#include <PlannerConfiguration.hpp>


/**
 * @brief Represents the configuration file of the application.
 */
class Configuration {
    public:
        NetworkConfiguration network;   // The network configuration.
        PlannerConfiguration planner;   // The planner configuration.

        /**
         * @brief Construct a new configuration and set default values (zero).
         */
        Configuration();

        /**
         * @brief Clear the configuration and set default values (zero).
         */
        void Clear(void);

        /**
         * @brief Read the configuration from the configuration file.
         * @param[in] filename The filename of the configuration file.
         * @return True if configuration file was read successfully, false otherwise.
         */
        bool ReadFromFile(std::string filename);
};

