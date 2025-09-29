#pragma once


#include <mpsv/mpsv.hpp>
#include <ExampleAsyncOnlinePlanning2.hpp>


// define a custom class that inherits from the asynchronous online planner
class ExamplePlanner: protected mpsv::planner::AsyncOnlinePlanner {
    public:
        /* make some member function public again */
        using mpsv::planner::AsyncOnlinePlanner::SetParameter;
        using mpsv::planner::AsyncOnlinePlanner::SetInput;

        /**
         * @brief Start the custom example planner.
         * @return True if success, false otherwise.
         */
        bool Start(void){
            constexpr int16_t pathMaxNumNodes = 200;
            constexpr uint32_t pathMaxNumSamples = 1000000;
            constexpr int16_t motionMaxNumNodes = 200;
            constexpr uint32_t motionMaxNumSamples = 1000000;
            constexpr int32_t threadPriority = 20;
            int32_t ompNumThreads = -1; // ignore
            int32_t ompDynamic = -1; // ignore
            std::vector<int32_t> cpuCoreIDs; // ignore
            return Initialize(pathMaxNumNodes, pathMaxNumSamples, motionMaxNumNodes, motionMaxNumSamples, threadPriority, ompNumThreads, ompDynamic, cpuCoreIDs);
        }

        /**
         * @brief Stop the custom example planner.
         */
        void Stop(void){
            Terminate();
        }

    protected:
        /**
         * @brief This function is called from inside the worker thread of the planner, if a planning problem has been solved and new
         * output data is available. The actual data is obtained via @ref GetOutput.
         */
        void CallbackNewOutputAvailable(void){
            mpsv::planner::AsyncOnlinePlannerOutput dataOut = GetOutput();
            std::cout << "threadState=" << mpsv::to_string(dataOut.threadState) <<
                     ", timestampInput=" << std::to_string(dataOut.timestampInput) <<
                     ", timestampParameter=" << std::to_string(dataOut.timestampParameter) <<
                     ", timeoutInput=" << dataOut.timeoutInput <<
                     ", validInput=" << dataOut.validInput <<
                     ", validParameter=" << dataOut.validParameter <<
                     ", error=" << dataOut.error <<
                     ", performedReset=" << dataOut.performedReset <<
                     ", timestamp=" << dataOut.timestamp <<
                     ", sampletime=" << dataOut.sampletime <<
                     ", trajectory=" << dataOut.trajectory.size() << " points\n";

            // write results to a file
            mpsv::core::DataLogFile f;
            std::string filename = example::GetDataFilename();
            constexpr bool appendData = true;
            f.Open(filename, appendData);
            dataOut.WriteToFile(f, "examplePlanner.");
            f.Close();
        }
};

