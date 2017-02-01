#ifndef ITEMSIMAGE_H
#define ITEMSIMAGE_H

#include <QGraphicsObject>
#include <QDebug>

class ItemsImage : public QGraphicsItem
{
    // Q_OBJECT

public:
    ItemsImage(int id, int r, int g, int b);

    QRectF boundingRect() const Q_DECL_OVERRIDE;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) Q_DECL_OVERRIDE;

public slots:
    void mousePressEvent(QGraphicsSceneMouseEvent * event) Q_DECL_OVERRIDE;

private:
    QColor      color;
    int         id;
};

#endif // ITEMSIMAGE_H
