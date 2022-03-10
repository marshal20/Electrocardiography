#pragma once
#include <string>

std::string open_file_dialog(const std::string& initial_name, const std::string& filter);

std::string save_file_dialog(const std::string& initial_name, const std::string& filter);
