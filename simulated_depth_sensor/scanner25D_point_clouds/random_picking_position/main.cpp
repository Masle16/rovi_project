/* cpp file for generating random positions
 *  on the picking area
 *
 *  x = [-0.3, ..., 0.3]
 *  y = [0.37, ..., 0.53]
 *  roll = [0, ..., 360]
 */

#include <iostream>
#include <rw/rw.hpp>
#include <fstream>

#define DATA_FILE "../data.txt"
#define WC_FILE "../../../workcell/Scene.wc.xml"

int main(int argv, char** argc) {
    std::cout << "\nProgram started\n" << std::endl;

    rw::math::Math::seed();
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(WC_FILE);
    if (wc == NULL) {
        std::cerr << "Workcell not found!" << std::endl;
        return -1;
    }
    rw::kinematics::Frame *frame = wc->findFrame("Duck");
    if (frame == NULL) {
        std::cerr << "Duck frame not found!" << std::endl;
        return -1;
    }
    rw::kinematics::State state = wc->getDefaultState();

    std::ofstream myFile;
    myFile.open(DATA_FILE);
    myFile << "area:\n";
    myFile << "x = [-0.3, ..., 0.3]\n";
    myFile << "y = [0.37, ..., 0.53]\n";
    myFile << "roll = [0, ..., 360]\n";

    std::string x, y, roll;
    for (size_t i = 0; i < 30; i++) {
        x = std::to_string(rw::math::Math::ran(-0.3, 0.3));
        y = std::to_string(rw::math::Math::ran(0.37, 0.53));
        roll = std::to_string(rw::math::Math::ran(0, 360.0));
        std::cout << "x: " << x << " y: " << y << " roll: " << roll << std::endl;
        myFile << "<RPY>"
               << roll
               << " 0 0</RPY> <Pos>"
               << x << " "
               << y << " 0.13275</Pos>\n";
    }

    myFile.close();

    std::cout << "\nProgram ended\n" << std::endl;
    return 0;
}
