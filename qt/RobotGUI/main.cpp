#include "RobotGUI.h"

#include <QtWidgets>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    RobotGUI * gui = new RobotGUI();

    return a.exec();
}

// TODO Fix the buffer amount to direct images to center of view
// TODO Potentially draw a grid that will show approximate RobotPositions
// TODO Draw 4 lines that center the view
// TODO Determine how to have signals from the robot execute the functions
