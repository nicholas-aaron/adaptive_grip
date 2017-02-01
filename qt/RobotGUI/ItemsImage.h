#ifndef ITEMSIMAGE_H
#define ITEMSIMAGE_H

#include <QGraphicsObject>
#include <QDebug>

class ItemsImage : public QGraphicsObject
{
public:
    ItemsImage(int id);

    QRectF boundingRect() const Q_DECL_OVERRIDE;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) Q_DECL_OVERRIDE;

public slots:
    void mousePressEvent(QGraphicsSceneMouseEvent * event);

private:
    QColor      color;
    int         id;
};

#endif // ITEMSIMAGE_H
