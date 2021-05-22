#include "3rd_party/Turtle/Turtle.h"
#include "3rd_party/Turtle/Turtle.cpp"
#include <string>

int main() {
    std::string inputPath = "D:/Documents/Uni/GEO1101_Synthesis_Project/materials/test_1_alphen/tree_zoom_lsystem.json";
    std::string outputPath = "../export.xyz";
    std::string outputPath2 = "../export.ply";

    Turtle turtle;
    turtle.setDebug(false);
    turtle.set2Degrees();
    turtle.readFile(inputPath);
    turtle.writeToXYZ(outputPath);
    turtle.writeToPly(outputPath2);
}
