#include <iostream>
#include <vector>
#include "easy3d/core/vec.h"
#include "easy3d/core/mat.h"


class Turtle {
public:
    /// default constructor ///
    explicit Turtle (double x = 0, double y =0, double z =0) {
        loc = {x, y, z};
        plane.set_row(0, easy3d::Vec<3, double>{1,0,0});
        plane.set_row(1, easy3d::Vec<3, double>{0,0,1});
    }

    /// construct turtle based on another turtle location ///
    explicit Turtle (easy3d::Vec<3, double> p){
        loc = p;
        plane.set_row(0, easy3d::Vec<3, double>{1,0,0});
        plane.set_row(1, easy3d::Vec<3, double>{0,0,1});
    }

    /// initializer override from crooked stems (do not use while walking)///
    //TODO fix this
    void setRotation(double angle, double roll = 0){
        rollPlane(roll);
        rotatePlane(angle);
    }

    /// output the current location of the turtle ///
    void printLocation() const{
        std::cout <<"current location of turtle is: (";
        std::cout << loc.x << ", " << loc.y << ", " << loc.z << ")" << std::endl;
    }

    /// step forward ///
    void stepForward(double distance){
        loc.x += distance * plane.row(1).x;
        loc.y += distance * plane.row(1).y;
        loc.z += distance * plane.row(1).z;
    }

    /// rotate ///
    void rotatePlane(double angle){
        angle = angle * M_PI/180;
        
    }

    /// roll ///
    void rollPlane(double rollAngle){

    }

    //TODO read
    //TODO store
    //TODO write
    //TODO angle right and left
    //TODO orientate right and left
    //TODO custom overrides
    //TODO nesting

    // get the current position of the turtle
    easy3d::Vec<3, double> location(){
        return loc;
    }

    // get the current rotation of the turtle
    easy3d::Vec<3, double> orientation(){
        return plane.row(1);
    }

private:
    // location is a 3d coordinate
    easy3d::Vec<3, double> loc;

    // plane is a 2d plane in a 3d space
    easy3d::Mat<2,3, double> plane;
};


int main() {
    Turtle turtle;
    turtle.printLocation();
    turtle.setRotation(10);
    turtle.stepForward(5);
    turtle.stepForward(5);
    turtle.printLocation();


}
