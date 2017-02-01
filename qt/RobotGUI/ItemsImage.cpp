#include "ItemsImage.h"

#include <QGraphicsScene>
#include <QPainter>
#include <QStyleOption>

#include <math.h>

ItemsImage::ItemsImage(int id, int r, int g, int b)
    : color (r, g, b),
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
    painter->drawText(0, 25, (QString) id);
    painter->setBrush(Qt::NoBrush);
}

void ItemsImage::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    qDebug() << "I clicked this item and text popped up";
}
