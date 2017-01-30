#include "RobotGUI.h"
#include "RobotLimits.h"

#include "math.h"

#define SCENE_WIDTH 600
#define SCENE_HEIGHT 360

#define BUFFER_AMOUNT 25

RobotGUI::RobotGUI()
{
    // Initialize the list of items
    QList<TargetItem> listOfItems;

    // Creates the scene to display the items
    QGraphicsScene * scene = new QGraphicsScene;
    scene->setSceneRect(0, 0, SCENE_WIDTH + BUFFER_AMOUNT, SCENE_HEIGHT + BUFFER_AMOUNT);

    // Testing the GUI
    // testAddItems(100, scene, listOfItems);
    testRmvItems(scene, listOfItems);

    // Visualize the scene
    QGraphicsView * view = new QGraphicsView;
    view->setScene(scene);
    view->resize(SCENE_WIDTH + BUFFER_AMOUNT, SCENE_HEIGHT + BUFFER_AMOUNT);
    view->setWindowTitle("Gantry Robot GUI");

    // Display the view
    view->show();
}

/*
    Function    : addNewItem
    Arguments   : QString s, RobotPosition pos, QGraphicsScene *scene, QList list
    Purpose     : Checks if the RobotPosition pos is valid, and adds an item to
                  the GUI if it is. Furthermore, saves the item's pointer in a list
*/
void RobotGUI::addNewItem(QString s, RobotPosition pos, QGraphicsScene * scene, QList<TargetItem> &list)
{
    TargetItem newItem(s, pos, scene);
    list.append(newItem);
}

/*
    Function    : removeAnItem
    Arguments   : QString s, QList<TargetItem> list
    Purpose     : Remove items specified by the user. Removing the item from the scene
                  does not delete the item. Therefore, it must be done manually by the
                  user.
*/
void RobotGUI::removeAnItem(QString s, QGraphicsScene *scene, QList<TargetItem> &list)
{
    for(int i = 0; i < list.size(); i++) {
        if (list[i].name == s) {
            scene->removeItem(list[i].image);
            delete list[i].image;
        }
    }
}

/*
    Function    : moveAnItem
    Arguments   : QString s, RobotPosition newPos, QGraphicsScene *scene, QList<TargetItem> &list
    Purpose     : If an item has been moved by the claw, update its position and redisplays on the GUI
*/
void RobotGUI::moveAnItem(QString s, RobotPosition newPos, QList<TargetItem> &list)
{
    for (int i = 0; i < list.size(); i++) {
        if (list[i].name == s) {
            list[i].updateCoordinates(newPos);
            list[i].image->setPos(list[i].x, list[i].y);
        }
    }
}

/*
    Function    : checkPosition
    Arguments   : RobotPosition pos
    Purpose     : Determines if a given RobotPosition is within the RobotLimits
                  using the predefined posWithin() function in RobotLimits.h
*/
bool RobotGUI::checkPosition(RobotPosition pos)
{
    RobotLimits lim;
    bool isPositionValid = lim.posWithin(pos);

    return isPositionValid;
}

// TEST FUNCTIONS

void RobotGUI::testAddItems(int n, QGraphicsScene * scene, QList<TargetItem> &list)
{
    for(int i = 0; i < n; i++) {
        RobotPosition pos(qrand() % 3000, qrand() % 1795*(-1), 0, 0, 0, 0);
        RobotGUI::addNewItem("Item", pos, scene, list);
    }
}

void RobotGUI::testRmvItems(QGraphicsScene * scene, QList<TargetItem> &list)
{
    RobotPosition testPos;

    testPos = generatePos();
    RobotGUI::addNewItem("Item 1", testPos, scene, list);

    testPos = generatePos();
    RobotGUI::addNewItem("Item 2", testPos, scene, list);

    testPos = generatePos();
    RobotGUI::addNewItem("Item 3", testPos, scene, list);

    RobotGUI::removeAnItem("Item 2", scene, list);
}

RobotPosition RobotGUI::generatePos()
{
    RobotPosition pos(qrand() % 3000, qrand() % 1795 * (-1), 0, 0, 0, 0);
    return pos;
}



