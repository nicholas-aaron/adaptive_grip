#ifndef ITEMSIMAGE_H
#define ITEMSIMAGE_H

#include <QGraphicsItem>

class ItemsImage : public QGraphicsItem
{
public:
    ItemsImage(QString name);

    QRectF boundingRect() const Q_DECL_OVERRIDE;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) Q_DECL_OVERRIDE;

private:
    QColor      color;
    QString     name;
};

#endif // ITEMSIMAGE_H
