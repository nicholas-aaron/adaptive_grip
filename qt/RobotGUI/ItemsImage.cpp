#include "ItemsImage.h"

#include <QGraphicsScene>
#include <QPainter>
#include <QStyleOption>

#include <math.h>

ItemsImage::ItemsImage(QString name)
    : color (qrand() % 256, qrand() % 256, qrand() % 256),
      name (name)
{
}

QRectF ItemsImage::boundingRect() const
{
    return QRectF(0, 0, 25, 25);
}

void ItemsImage::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
    painter->setBrush(color);
    painter->drawEllipse(0, 0, 15, 15);
    painter->drawText(0, 25, name);
    painter->setBrush(Qt::NoBrush);
}
