#ifndef ROBOTGUI_H
#define ROBOTGUI_H

#include "../../headers/WSObject.h"

#include <QWidget>
#include <QList>
#include <QGraphicsScene>
#include <QGraphicsView>

class RobotGUI : public QWidget
{
    Q_OBJECT

public:
    RobotGUI();

private:
    QGraphicsScene *        myScene;
    QGraphicsView *         myView;
    QList<WSObject> *       currentList;

    QList<WSObject> generateWSObject(int n);

private slots:
    void updateWSObjects(QList<WSObject> * newList);
    void redrawItems();

signals:
    void wsobjectsUpdated(QList<WSObject> * newList);

};

#endif // ROBOTGUI_H
