#include "gui.h"
#include <filesystem>
#include <string>

namespace fs = std::filesystem;


// ends_with in c++ 17
bool ends_with(const std::string& main_str, const std::string& suffix) {
    if (main_str.length() < suffix.length()) {
        return false;
    }
    return main_str.compare(main_str.length() - suffix.length(), suffix.length(), suffix) == 0;
}

int main() {
    // CLion run support:
    // It detects if it's running inside cmake-build-debug folder
    // and changes it
    auto p = fs::current_path();
    if (ends_with(p, "cmake-build-debug"))
    {
        p = p.parent_path();
        fs::current_path(p);
    }

    GUI gui("nodes.csv", "edges.csv");
    gui.main_loop();
    return 0;
}
