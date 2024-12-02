#include "gravity_io.h"

#include <fstream>

namespace glomap {
    void ReadGravity(const std::string& gravity_path,
                     std::unordered_map<image_t, Image>& images) {
        std::unordered_map<std::string, image_t> name_idx;
        for (const auto& [image_id, image] : images)
        {
            name_idx[image.file_name] = image_id;
        }

        std::ifstream file(gravity_path);

        // Read in the file list
        std::string line, item;
        Eigen::Vector3d gravity;
        int counter = 0;
        while (std::getline(file, line))
        {
            std::stringstream line_stream(line);

            // file_name
            std::string name;
            std::getline(line_stream, name, ' ');

            // Gravity
            for (double i = 0; i < 3; i++)
            {
                std::getline(line_stream, item, ' ');
                gravity[i] = std::stod(item);
            }

            // Check whether the image present
            auto ite = name_idx.find(name);
            if (ite != name_idx.end())
            {
                counter++;
                images[ite->second].gravity_info.SetGravity(gravity);
                // Make sure the initialization is aligned with the gravity
                images[ite->second].cam_from_world.rotation =
                    images[ite->second].gravity_info.GetRAlign().transpose();
            }
        }
       // LOG(INFO) << counter << " images are loaded with gravity" << std::endl;
    }

} // namespace glomap