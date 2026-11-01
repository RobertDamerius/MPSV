#pragma once


#include <mpsv/mpsv.hpp>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <fstream>
#include <Scene.hpp>


class SceneDataFile {
    public:
        std::vector<Scene> scenes;   // List of all scenes.

        /**
         * @brief Read the @ref scenes from a binary file.
         * @param [in] filename The name of the scene file to read.
         * @return True if file reading and deserialization was successful, false otherwise.
         * @note The scenes are not checked for validity.
         */
        bool Read(std::string filename){
            std::ifstream file(filename, std::ifstream::binary);
            if(file){
                file.seekg(0, file.end);
                if(!file) return false;
                std::streampos endPosition = file.tellg();
                if(endPosition == std::streampos(-1)) return false;
                std::streamoff fileSize = endPosition;
                if(fileSize < 0 ||
                   static_cast<std::uintmax_t>(fileSize) > std::numeric_limits<size_t>::max() ||
                   static_cast<std::uintmax_t>(fileSize) > std::numeric_limits<std::streamsize>::max()){
                    return false;
                }
                size_t length = static_cast<size_t>(fileSize);
                file.seekg(0, file.beg);
                if(!file) return false;
                std::vector<char> buffer(length);
                file.read(buffer.data(), static_cast<std::streamsize>(length));
                if(!file) return false;
                return Deserialize(buffer.data(), length);
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
            uint32_t numScenes;
            std::memcpy(&numScenes, buffer + i, sizeof(numScenes)); i += sizeof(numScenes);

            // Deserialize all scene data
            for(uint32_t s = 0; s < numScenes; ++s){
                data.push_back(Scene());

                // initialPose (3*8 bytes)
                if((i + 24) > length) return false;
                std::memcpy(&data.back().initialStateAndInput[0], buffer + i, sizeof(double)); i += sizeof(double);
                std::memcpy(&data.back().initialStateAndInput[1], buffer + i, sizeof(double)); i += sizeof(double);
                std::memcpy(&data.back().initialStateAndInput[2], buffer + i, sizeof(double)); i += sizeof(double);

                // finalPose (3*8 bytes)
                if((i + 24) > length) return false;
                std::memcpy(&data.back().finalPose[0], buffer + i, sizeof(double)); i += sizeof(double);
                std::memcpy(&data.back().finalPose[1], buffer + i, sizeof(double)); i += sizeof(double);
                std::memcpy(&data.back().finalPose[2], buffer + i, sizeof(double)); i += sizeof(double);

                // numStaticObstacles (4 bytes)
                if((i + 4) > length) return false;
                uint32_t numStaticObstacles;
                std::memcpy(&numStaticObstacles, buffer + i, sizeof(numStaticObstacles)); i += sizeof(numStaticObstacles);

                // Deserialize all static obstacles
                for(uint32_t p = 0; p < numStaticObstacles; ++p){
                    // numVertices (4 bytes)
                    if((i + 4) > length) return false;
                    uint32_t numVertices;
                    std::memcpy(&numVertices, buffer + i, sizeof(numVertices)); i += sizeof(numVertices);

                    // Deserialize all vertices
                    std::vector<std::array<double,2>> vertices;
                    for(uint32_t v = 0; v < numVertices; ++v){
                        vertices.push_back(std::array<double,2>());

                        // vertex (2*8 bytes)
                        if((i + 16) > length) return false;
                        std::memcpy(&vertices.back()[0], buffer + i, sizeof(double)); i += sizeof(double);
                        std::memcpy(&vertices.back()[1], buffer + i, sizeof(double)); i += sizeof(double);
                    }
                    data.back().staticObstacles.push_back(mpsv::geometry::StaticObstacle(vertices));
                }
            }
            data.swap(scenes);
            return true;
        }
};
