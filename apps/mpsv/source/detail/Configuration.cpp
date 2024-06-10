#include <Configuration.hpp>
#include <nlohmann/json.hpp>


Configuration::Configuration(){
    Clear();
}

void Configuration::Clear(void){
    network.Clear();
    planner.Clear();
}

bool Configuration::ReadFromFile(std::string filename){
    // set default values
    Clear();

    // parse JSON file
    nlohmann::json jsonData;
    try{
        std::ifstream configurationFile(filename);
        jsonData = nlohmann::json::parse(configurationFile);
    }
    catch(const std::exception& e){
        PrintE("Failed to parse configuration file \"%s\": %s\n", filename.c_str(), e.what());
        return false;
    }

    // set configuration values
    bool success = true;
    try{ network.localPort                = jsonData.at("network").at("localPort");                } catch(const std::exception& e){ success = false; PrintE("Error in configuration file \"%s\": %s\n", filename.c_str(), e.what()); }
    try{ network.destinationAddress       = jsonData.at("network").at("destinationAddress");       } catch(const std::exception& e){ success = false; PrintE("Error in configuration file \"%s\": %s\n", filename.c_str(), e.what()); }
    try{ network.destinationPort          = jsonData.at("network").at("destinationPort");          } catch(const std::exception& e){ success = false; PrintE("Error in configuration file \"%s\": %s\n", filename.c_str(), e.what()); }
    try{ network.interfaceAddress         = jsonData.at("network").at("interfaceAddress");         } catch(const std::exception& e){ success = false; PrintE("Error in configuration file \"%s\": %s\n", filename.c_str(), e.what()); }
    try{ network.deviceName               = jsonData.at("network").at("deviceName");               } catch(const std::exception& e){ success = false; PrintE("Error in configuration file \"%s\": %s\n", filename.c_str(), e.what()); }
    try{ network.socketPriority           = jsonData.at("network").at("socketPriority");           } catch(const std::exception& e){ success = false; PrintE("Error in configuration file \"%s\": %s\n", filename.c_str(), e.what()); }
    try{ network.allowBroadcast           = jsonData.at("network").at("allowBroadcast");           } catch(const std::exception& e){ success = false; PrintE("Error in configuration file \"%s\": %s\n", filename.c_str(), e.what()); }
    try{ network.socketErrorRetryTimeMs   = jsonData.at("network").at("socketErrorRetryTimeMs");   } catch(const std::exception& e){ success = false; PrintE("Error in configuration file \"%s\": %s\n", filename.c_str(), e.what()); }
    try{ network.receiverThreadPriority   = jsonData.at("network").at("receiverThreadPriority");   } catch(const std::exception& e){ success = false; PrintE("Error in configuration file \"%s\": %s\n", filename.c_str(), e.what()); }
    try{ planner.pathMaxNumNodes          = jsonData.at("planner").at("pathMaxNumNodes");          } catch(const std::exception& e){ success = false; PrintE("Error in configuration file \"%s\": %s\n", filename.c_str(), e.what()); }
    try{ planner.pathMaxNumSamples        = jsonData.at("planner").at("pathMaxNumSamples");        } catch(const std::exception& e){ success = false; PrintE("Error in configuration file \"%s\": %s\n", filename.c_str(), e.what()); }
    try{ planner.motionMaxNumNodes        = jsonData.at("planner").at("motionMaxNumNodes");        } catch(const std::exception& e){ success = false; PrintE("Error in configuration file \"%s\": %s\n", filename.c_str(), e.what()); }
    try{ planner.motionMaxNumSamples      = jsonData.at("planner").at("motionMaxNumSamples");      } catch(const std::exception& e){ success = false; PrintE("Error in configuration file \"%s\": %s\n", filename.c_str(), e.what()); }
    try{ planner.threadPriorityPlanner    = jsonData.at("planner").at("threadPriorityPlanner");    } catch(const std::exception& e){ success = false; PrintE("Error in configuration file \"%s\": %s\n", filename.c_str(), e.what()); }
    try{ planner.threadPriorityUpdater    = jsonData.at("planner").at("threadPriorityUpdater");    } catch(const std::exception& e){ success = false; PrintE("Error in configuration file \"%s\": %s\n", filename.c_str(), e.what()); }
    try{ planner.ompNumThreads            = jsonData.at("planner").at("ompNumThreads");            } catch(const std::exception& e){ success = false; PrintE("Error in configuration file \"%s\": %s\n", filename.c_str(), e.what()); }
    try{ planner.ompDynamic               = jsonData.at("planner").at("ompDynamic");               } catch(const std::exception& e){ success = false; PrintE("Error in configuration file \"%s\": %s\n", filename.c_str(), e.what()); }
    try{ planner.updatePeriod             = jsonData.at("planner").at("updatePeriod");             } catch(const std::exception& e){ success = false; PrintE("Error in configuration file \"%s\": %s\n", filename.c_str(), e.what()); }
    return success;
}

