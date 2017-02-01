#include "RobotGUI.h"
#include "ItemsImage.h"
#include "../../headers/WSObject.h"

#define SCENE_WIDTH 600
#define SCENE_HEIGHT 360

#define BUFFER_AMOUNT 25

RobotGUI::RobotGUI() :
    currentList(NULL)
{
    // Can be edited to deal with an already created view
    myScene->setSceneRect(0, 0, SCENE_WIDTH + BUFFER_AMOUNT, SCENE_HEIGHT + BUFFER_AMOUNT);

    // Ask Aaron how he intends to pass the signal
    connect(this, SIGNAL(wsobjectsUpdated(QList<WSObject>*)), this, SLOT(updateWSObjects(QList<WSObject>*));
    connect(this, SIGNAL(wsobjectsUpdated(QList<WSObject>*)), this, SLOT(redrawItems());

    myView->setScene(myScene);
    myView->resize(myScene->width(), myScene->height());
    myView->setWindowTitle("Gantry Robot GUI");

    myView->show();
}

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
            currentList = newList;
            emit wsobjectsUpdated(newList);
        }
    }
}

void RobotGUI::redrawItems()
{
    QList<WSObject> cList = *currentList;

    myScene->clear();
    myView->update();

    for (int i = 0; i < currentList->size(); i++) {
        ItemsImage * item = new ItemsImage;

        item->x = cList[i].x_position / 5.0;
        item->y = cList[i].y_position / 5.0;
        item->setPos(item->x, item->y);

        myScene->addItem(item);
    }
}

void RobotGUI::wsobjectsUpdated(QList<WSObject> * newList)
{
}


