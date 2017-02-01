#include "ItemsImage.h"

#include <QGraphicsScene>
#include <QPainter>
#include <QStyleOption>

#include <math.h>

ItemsImage::ItemsImage(int id)
    : color (qrand() % 256, qrand() % 256, qrand() % 256),
      id (id)
{
}

QRectF ItemsImage::boundingRect() const
{
    return QRectF(0, 0, 15, 15);
}

void ItemsImage::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
    painter->setBrush(color);
    painter->setPen(QColor(255, 0, 0, 127));
    painter->drawRect((boundingRect()));
    painter->drawEllipse(0, 0, 15, 15);
    painter->drawText(0, 25, id);
    painter->setBrush(Qt::NoBrush);
}

void ItemsImage::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    qDebug() << "I clicked on a graphics item";
}
