#pragma once
#include <filesystem>
#include <sstream>
#include <string>


std::string generateUniqueFilename(const std::string& directory,
                                   const std::string& base_name,
                                   const std::string& extension)
{
    namespace fs = std::filesystem;
    int index = 0;

    std::string full_path = directory + "/" + base_name + "_" + std::to_string(index) + extension;
    while (fs::exists(full_path))
    {
        std::ostringstream oss;
        oss << directory << "/" << base_name << "_" << index << extension;
        full_path = oss.str();
        index++;
    }
    return full_path;
}
