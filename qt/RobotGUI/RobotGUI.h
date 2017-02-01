#ifndef ROBOTGUIV2_H
#define ROBOTGUIV2_H

#include "../../headers/WSObject.h"

#include <QObject>
#include <QGraphicsObject>
#include <QGraphicsView>
#include <QList>

class RobotGUIVTwo : public QGraphicsView
{
    Q_OBJECT

public:
    RobotGUIVTwo();

private:
    QGraphicsScene *        myScene;
    QGraphicsView *         myView;
    QList<WSObject> *       currentList;


public slots:
    void updateWSObjects(QList<WSObject> * newList);
    void redrawItems();

signals:
    void wsobjectsUpdated(QList<WSObject> * newList);

};

#endif // ROBOTGUIV2_H
