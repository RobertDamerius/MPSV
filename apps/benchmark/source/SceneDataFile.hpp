#pragma once


#include <mpsv/mpsv.hpp>
#include <string>
#include <fstream>
#include <Scene.hpp>


class SceneDataFile {
    public:
        std::vector<Scene> scenes;   ///< List of all scenes.

        /**
         * @brief Read the @ref scenes from a binary file.
         * @param [in] filename The name of the scene file to read.
         * @return True if file reading and deserialization was successfull, false otherwise.
         * @note The scenes are not checked for validity.
         */
        bool Read(std::string filename){
            std::ifstream file(filename, std::ifstream::binary);
            if(file){
                file.seekg(0, file.end);
                size_t length = file.tellg();
                file.seekg(0, file.beg);
                char* buffer = new char[length];
                file.read(buffer, length);
                bool success = static_cast<bool>(file);
                file.close();
                if(success){
                    success = Deserialize(buffer, length);
                }
                delete[] buffer;
                return success;
            }
            return false;
        }

        /**
         * @brief Check whether all scenes are valid.
         * @return True if all scenes are valid, false otherwise.
         */
        bool IsValid(void){
            bool valid = true;
            for(auto&& scene : scenes){
                valid &= scene.IsValid();
            }
            return valid;
        }

    protected:
        bool Deserialize(char* buffer, size_t length){
            std::vector<Scene> data;
            size_t i = 0;

            // numScenes (4 bytes)
            if((i + 4) > length) return false;
            uint32_t numScenes = *reinterpret_cast<uint32_t*>(&buffer[i]); i += 4;

            // Deserialize all scene data
            for(uint32_t s = 0; s < numScenes; ++s){
                data.push_back(Scene());

                // initialPose (3*8 bytes)
                if((i + 24) > length) return false;
                data.back().initialStateAndInput[0] = *reinterpret_cast<double*>(&buffer[i]); i += 8;
                data.back().initialStateAndInput[1] = *reinterpret_cast<double*>(&buffer[i]); i += 8;
                data.back().initialStateAndInput[2] = *reinterpret_cast<double*>(&buffer[i]); i += 8;

                // finalPose (3*8 bytes)
                if((i + 24) > length) return false;
                data.back().finalPose[0] = *reinterpret_cast<double*>(&buffer[i]); i += 8;
                data.back().finalPose[1] = *reinterpret_cast<double*>(&buffer[i]); i += 8;
                data.back().finalPose[2] = *reinterpret_cast<double*>(&buffer[i]); i += 8;

                // numStaticObstacles (4 bytes)
                if((i + 4) > length) return false;
                uint32_t numStaticObstacles = *reinterpret_cast<uint32_t*>(&buffer[i]); i += 4;

                // Deserialize all static obstacles
                for(uint32_t p = 0; p < numStaticObstacles; ++p){
                    // numVertices (4 bytes)
                    if((i + 4) > length) return false;
                    uint32_t numVertices = *reinterpret_cast<uint32_t*>(&buffer[i]); i += 4;

                    // Deserialize all vertices
                    std::vector<std::array<double,2>> vertices;
                    for(uint32_t v = 0; v < numVertices; ++v){
                        vertices.push_back(std::array<double,2>());

                        // vertex (2*8 bytes)
                        if((i + 16) > length) return false;
                        vertices.back()[0] = *reinterpret_cast<double*>(&buffer[i]); i += 8;
                        vertices.back()[1] = *reinterpret_cast<double*>(&buffer[i]); i += 8;
                    }
                    data.back().staticObstacles.push_back(mpsv::geometry::StaticObstacle(vertices));
                }
            }
            data.swap(scenes);
            return true;
        }
};

