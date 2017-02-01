#include "RobotGUI.h"
#include "ItemsImage.h"

#define SCREEN_RATIO 5.0
#define SCENE_WIDTH 600
#define SCENE_HEIGHT 360

#define BUFFER_AMOUNT 25

RobotGUI::RobotGUI() :
    currentList(0)
{
    // Initialize both the view and the scene
    QGraphicsScene * scene = new QGraphicsScene;
    QGraphicsView * view = new QGraphicsView;

    // Set the scene, and the scene's size
    myScene = scene;
    myScene->setSceneRect(0, 0, SCENE_WIDTH + BUFFER_AMOUNT, SCENE_HEIGHT+BUFFER_AMOUNT);

    // If the anything about the list of WSObjects changes, update the current list in the GUI and redraw the items
    connect(this, SIGNAL(wsobjectsUpdated(QList<WSObject>*)), this, SLOT(updateWSObjects(QList<WSObject>*)));
    connect(this, SIGNAL(wsobjectsUpdated(QList<WSObject>*)), this, SLOT(redrawItems()));

    // Set the view, and resize the view to the dimensions of its parent scene
    myView = view;
    myView->setScene(myScene);
    myView->resize(myScene->width(), myScene->height());
    myView->setWindowTitle("Gantry Robot GUI");

    myView->show();
}

/**
 * Updates the list of WSObjects that is currently stored within the RobotGUI
 * @param newList is an updated list of WSObjects that is passed by a user
 */
void RobotGUI::updateWSObjects(QList<WSObject> * newList)
{
    QList<WSObject> nList = *newList;
    QList<WSObject> cList = *currentList;

    // The list has been updated. Their sizes are different
    if (newList->size() != cList.size()) {
        delete currentList;
        currentList = newList;
        emit wsobjectsUpdated(newList);
    }

    // If any of the WSObjects have changed, signal that the lists have updated
    for (int i = 0; i < newList->size(); i++) {
        bool areEqual = (nList[i] == cList[i]);
        if (!areEqual) {
            delete currentList;
            currentList = newList;
            emit wsobjectsUpdated(newList);
        }
    }
}

/**
 * This should only be called after having called updateWSObjects
 * This function will redraw all items into the scene based on the information
 * given by the WSObjects in the list
 */
void RobotGUI::redrawItems()
{
    QList<WSObject> cList = *currentList;

    myScene->clear();

    for (int i = 0; i < currentList->size(); i++) {
        ItemsImage * item = new ItemsImage(cList[i].id, cList[i].r_display, cList[i].g_display, cList[i].b_display);
        item->setPos(cList[i].x_position / SCREEN_RATIO, cList[i].y_position / SCREEN_RATIO);
        myScene->addItem(item);
    }
}




