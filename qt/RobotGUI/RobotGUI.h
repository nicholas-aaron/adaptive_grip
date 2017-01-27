#ifndef ROBOTGUI_H
#define ROBOTGUI_H

#include "TargetItem.h"

#include <QApplication>
#include <QList>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QTimer>

class RobotGUI : public QGraphicsView
{
public:
    RobotGUI();

    QGraphicsScene * scene;

    void addNewItem(QString s, RobotPosition pos, QGraphicsScene * scene, QList<TargetItem> &list);
    void removeAnItem(QString s, QGraphicsScene * scene, QList<TargetItem> &list);
    void moveAnItem(QString s, RobotPosition newPos, QList<TargetItem> &list);
    bool checkPosition (RobotPosition pos);

    // TEST FUNCTIONS
    void testAddItems(int n, QGraphicsScene * scene, QList<TargetItem> &list);
    void testRmvItems(QGraphicsScene * scene, QList<TargetItem> &list);
    RobotPosition generatePos();
};

#endif // ROBOTGUI_H
