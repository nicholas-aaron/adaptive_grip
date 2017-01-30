#include "TargetItem.h"
#include "ItemsImage.h"

#include "math.h"

#define SCREEN_RATIO 5.0

TargetItem::TargetItem(QString n, RobotPosition pos, QGraphicsScene * scene)
    : name(n)
{
    updateCoordinates(pos);
    ItemsImage * item = new ItemsImage(n);
    this->image = item;
    item->setPos(x, y);
    scene->addItem(item);
}

/*
    Function    : convertCoordinates
    Arguments   : RobotPosition pos, qreal x, qreal y
    Purpose     : Takes a given robot position, and converts it to the x and y components
                  that are useable by the GUI's coordinate system.
                  Note: The GUI coordinate system is approximately 1/5th of the robot's
                  position limits. Furthermore, the y-axis is inverted, and therefore the
                  downward direction is positive. x (0, 600), y(0, 360), origin at top left
*/
void TargetItem::updateCoordinates(RobotPosition pos)
{
    this->x = ceil(pos.x / SCREEN_RATIO);
    this->y = ceil(pos.y / -SCREEN_RATIO);
    this->position = pos;
}

