#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

namespace file_utils
{
    std::string resolve_file_path(const std::string & input_path, const std::string & package_name)
    {
        std::string resolved_path = input_path;
        if (!input_path.empty() && !std::filesystem::path(input_path).is_absolute()) {
            try {
                const auto share_dir = ament_index_cpp::get_package_share_directory(package_name);
                const auto share_candidate = std::filesystem::path(share_dir) / input_path;
                const auto cwd_candidate = std::filesystem::current_path() / input_path;
                if (std::filesystem::exists(share_candidate)) {
                    resolved_path = share_candidate.string();
                } else if (std::filesystem::exists(cwd_candidate)) {
                    resolved_path = cwd_candidate.string();
                } else {
                    resolved_path = share_candidate.string();
                    RCLCPP_WARN(
                        rclcpp::get_logger("file_utils"),
                        "File path '%s' is relative and was not found under share or CWD; using '%s'.",
                        input_path.c_str(),
                        resolved_path.c_str());
                }
            } catch (const std::exception & e) {
                const auto cwd_candidate = std::filesystem::current_path() / input_path;
                resolved_path = cwd_candidate.string();
                RCLCPP_WARN(
                    rclcpp::get_logger("file_utils"),
                    "Failed to resolve package share for file path '%s' (%s); using '%s'.",
                    input_path.c_str(),
                    e.what(),
                    resolved_path.c_str());
            }
        }
        return resolved_path;
    }



}