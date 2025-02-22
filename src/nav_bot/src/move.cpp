#include <iostream>
#include <cstdlib>
#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <thread>

int main() {
    double x = 0.0;
    bool increasing = true;  // Direction flag

    while (true) {
        // Format x to one decimal place
        //std::ostringstream oss;
        //oss << std::fixed << std::setprecision(1) << x;
        //std::string x_str = oss.str();

        //std::string x_str = std::to_string(x);

        // Build the command with the current x value
        // std::string capsule = "ign service -s /world/empty/set_pose "
        //                       "--reqtype ignition.msgs.Pose "
        //                       "--reptype ignition.msgs.Boolean "
        //                       "--timeout 1 "
        //                       "--req 'name: \"capsule\" id: 80 position { x: " + x_str +
        //                       " y: 0.0 z: 0.5 } orientation { w: 1.0 x: 0.0 y: 0.0 z: 0.0 }'";


        std::string capsule = "ign service -s /world/empty/set_pose "
                              "--reqtype ignition.msgs.Pose "
                              "--reptype ignition.msgs.Boolean "
                              "--timeout 1 "
                              "--req 'name: \"capsule\" id: 80 position { x: 2.3970"
                              " y: " + std::to_string(4.4767 - x) + " z: 0.0 } orientation { w: 1.0 x: 0.0 y: 0.0 z: 0.0 }'";

        std::string capsule_0 = "ign service -s /world/empty/set_pose "
                              "--reqtype ignition.msgs.Pose "
                              "--reptype ignition.msgs.Boolean "
                              "--timeout 1 "
                              "--req 'name: \"capsule_0\" id: 84 position { x: 4.4101" 
                              " y: " + std::to_string(-0.5764 - x) + " z: 0.0 } orientation { w: 1.0 x: 0.0 y: 0.0 z: 0.0 }'";

        // Execute command
        int ret = std::system(capsule.c_str());
        int ret_0 = std::system(capsule_0.c_str());
        if (ret != 0) {
            std::cerr << "Command failed with exit code " << ret << std::endl;
        }
        if (ret_0 != 0) {
            std::cerr << "Command failed with exit code " << ret << std::endl;
        }
        
        // Delay of 1 second
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Adjust x value
        if (increasing) {
            x += 0.1;
            if (x >= 3.0) {
                increasing = false;
            }
        } else {
            x -= 0.1;
            if (x <= 0.0) {
                increasing = true;
            }
        }
    }
    return 0;
}


/*
#include <iostream>
#include <cstdlib> // For std::system
#include <string>

int main() {
    for (int i = 0; i < 10; ++i) { // Example loop
        std::string command = "ign service -s /world/empty/set_pose "
                              "--reqtype ignition.msgs.Pose "
                              "--reptype ignition.msgs.Boolean "
                              "--timeout 1 "
                              "--req 'name: \"cylinder\" id: 8 position { x: 2.0 y: 0.0 z: 1.0 } "
                              "orientation { w: 1.0 x: 0.0 y: 0.0 z: 0.0 }'";

        int ret = std::system(command.c_str());

        if (ret != 0) {
            std::cerr << "Command failed with exit code " << ret << std::endl;
        }
    }
    return 0;
}
*/


// capsule id 80
// capsule_0 id 84