#ifndef TARGETITEM_H
#define TARGETITEM_H

#include "RobotPosition.h"
#include "ItemsImage.h"

#include <QString>
#include <QGraphicsScene>

class TargetItem
{
public:
    TargetItem(QString n, RobotPosition pos, QGraphicsScene * scene);

    void updateCoordinates(RobotPosition pos);
    QString         name;
    ItemsImage *    image;
    qreal           x;
    qreal           y;

private:
    RobotPosition   position;

};

#endif // TARGETITEM_H
