#include "RobotGUI.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    RobotGUI w;
    w.show();

    return a.exec();
}
