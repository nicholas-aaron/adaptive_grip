#ifndef ITEMDISPLAYWINDOW_H
#define ITEMDISPLAYWINDOW_H

#include <QWidget>
#include <QPainter>
#include <QPen>
#include <QtDesigner/QtDesigner>
#include "WSObject.h"

#include "RobotLimits.h"



class QDESIGNER_WIDGET_EXPORT ItemDisplayWindow : public QWidget
{
    Q_OBJECT
public:
    explicit ItemDisplayWindow(QWidget *parent = 0);

    void                      init_display();
    std::vector<WSObject> *   m_objects;

    int                       get_index();

protected:

    void     paintEvent(QPaintEvent *);

private:

    RobotLimits               m_limits;


    float                     x_scale_factor;
    float                     y_scale_factor;

    int                       select_index;


    int win_width, win_height;
    
    RobotLimits               limits;

signals:

public slots:

   void     cycle();
};

#endif // ITEMDISPLAYWINDOW_H
