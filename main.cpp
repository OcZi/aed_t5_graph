#include "gui.h"
#include <filesystem>
#include <string>

namespace fs = std::filesystem;


const std::string project_name = "aed_t5_graph";

// ends_with in c++ 17
bool ends_with(const std::string& main_str, const std::string& suffix) {
    if (main_str.length() < suffix.length()) {
        return false;
    }
    return main_str.compare(main_str.length() - suffix.length(), suffix.length(), suffix) == 0;
}

int main() {
    // cmake's debug folder support:
    // removes all subfolders of execution to handle .csv of the project.
    auto p = fs::current_path();
    auto p_str = p.string();

    auto i = p_str.find(project_name);
    if (i != std::string::npos)
    {
        // Substring from 0 to project_name's end
        fs::current_path(p_str.substr(0, i + project_name.size()));
    }

    GUI gui("nodes.csv", "edges.csv");
    gui.main_loop();
    return 0;
}
