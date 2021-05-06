#include <iostream>
#include <vector>
#include <string>
#include "easy3d/core/vec.h"
#include "easy3d/core/mat.h"


class Turtle {
public:
    /// default constructor ///
    explicit Turtle (double x = 0, double y =0, double z =0) {
        loc = {x, y, z};
        plane.set_row(0, easy3d::Vec<3, double>{1,0,0});
        plane.set_row(1, easy3d::Vec<3, double>{0,1,0});
        plane.set_row(2, easy3d::Vec<3, double>{0,0,1});
    }

    /// construct turtle based on another turtle location ///
    explicit Turtle (easy3d::Vec<3, double> p){
        loc = p;
        plane.set_row(0, easy3d::Vec<3, double>{1,0,0});
        plane.set_row(1, easy3d::Vec<3, double>{0,1,0});
        plane.set_row(2, easy3d::Vec<3, double>{0,0,1});

    }

    Turtle (Turtle const &other){
        loc = other.loc;
        plane = other.plane;
        debug = other.debug;
    }

    /// output the current location of the turtle ///
    void printLocation() const{
        std::cout <<"current location of turtle is: (";
        std::cout << loc.x << ", " << loc.y << ", " << loc.z << ")" << std::endl;
    }

    void setDebug(){
        debug = true;
    }


    /// initializer override from crooked stems (do not use while walking)///
    void setRotation(double angle, double roll = 0){
        // roll is executed before the angle
        rollPlane(roll);
        rotatePlane(angle);
    }

    void readLine(std::string line){
        for(int i = 0; i < line.size(); ++i) {
            double override = 0;
            int j = 0;

            // find override values;
            if (line[i + 1] == '('){
                j = 2;
                std::string sValue;
                while (line[i + j] != ')'){
                    sValue += line[i + j];
                    j++;
                }
                override = std::stod(sValue);
            }

            // execute normal stings
            if(line[i] == '['){
                Turtle turtle(*this);

                turtle.readLine(line.substr(i + 1));
                while (line[i] != ']'){i ++;}
                while (line[i] == ']'){i ++;}
                i--;
                continue;
            }

            if (line[i] == 'F') {
                if (override == 0){
                    stepForward(5);
                }
                else {
                    stepForward(override);
                }
                if (debug){printLocation();}

            } else if (line[i] == '+') {
                if (override == 0){
                    rotatePlane(10);
                }
                else {
                    rotatePlane(override);
                }
            } else if (line[i] == '-') {
                if (override == 0){
                    rotatePlane(-10);
                }
                else {
                    rotatePlane(- override);
                }
            } else if (line[i] == '>') {
                if (override == 0){
                    rollPlane(10);
                }
                else {
                    rollPlane(override);
                }
            } else if (line[i] == '<') {
                if (override == 0){
                    rollPlane(-10);
                }
                else {
                    rollPlane(-override);
                }
            } else if (line[i] == ']'){
                //terminate execution if end of nesting is found
                break;
            }
            i += j;
        }
    }


private:
    // location is a 3d coordinate
    easy3d::Vec<3, double> loc;

    // plane is a 2d plane in a 3d space
    easy3d::Mat<3,3, double> plane;

    //debug prints location at every step
    bool debug = false;

    /// step forward ///
    void stepForward(double distance){
        loc.x += distance * plane.row(2).x;
        loc.y += distance * plane.row(2).y;
        loc.z += distance * plane.row(2).z;
    }

    /// rotate ///
    void rotatePlane(double angle){
        angle = angle * M_PI/180;

        easy3d::Vec<3, double> uAxis = plane.row(0);
        easy3d::Vec<3, double> vAxis = plane.row(2);

        easy3d::Vec<3, double> uAxisT = uAxis*cos(angle) - vAxis*sin(angle);
        easy3d::Vec<3, double> vAxisT = uAxis*sin(angle) + vAxis*cos(angle);

        uAxisT.normalize();
        vAxisT.normalize();

        plane.set_row(0, uAxisT);
        plane.set_row(2, vAxisT);
    }

    /// roll ///
    void rollPlane(double rollAngle){
        rollAngle = rollAngle * M_PI/180;

        easy3d::Vec<3, double> uAxis = plane.row(0);
        easy3d::Vec<3, double> wAxis = plane.row(1);

        easy3d::Vec<3, double> uAxisT = uAxis*cos(rollAngle) - wAxis*sin(rollAngle);
        easy3d::Vec<3, double> wAxisT = uAxis*sin(rollAngle) + wAxis*cos(rollAngle);

        uAxisT.normalize();
        wAxisT.normalize();

        plane.set_row(0, uAxisT);
        plane.set_row(1, wAxisT);

    }

    //TODO custom read
    //TODO store
    //TODO write
    //TODO nesting
};


int main() {
    std::string line = "+(90)[F(10)+(90)F(15)[F(10)[F(1)F(1)]]]F(20)";
    Turtle turtle;
    turtle.setDebug();
    turtle.readLine(line);
    //turtle.printLocation();

}
