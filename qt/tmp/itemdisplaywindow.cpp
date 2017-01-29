#include "itemdisplaywindow.h"
#include <iostream>
ItemDisplayWindow::ItemDisplayWindow(QWidget *parent) : QWidget(parent)
{
      setAutoFillBackground(true);
      select_index = -1;
}

int
ItemDisplayWindow::get_index()
{
   return select_index;
}

void
ItemDisplayWindow::cycle()
{
    bool forward = true;
   if (!m_objects) {
      select_index = -1;
   } else if (m_objects->size() == 0) {
      select_index = -1;
   } else if (select_index == -1) {
      select_index = 0;
   } else {
      select_index = (forward) ? select_index + 1 : select_index - 1;
      select_index %= m_objects->size();
   }

   using std::cout;
   using std::endl;
   cout << "select_index = " << select_index << endl;
   update();

}

void
ItemDisplayWindow::paintEvent(QPaintEvent * event)
{
   QPainter painter(this);
   QRectF background(0.0, 0.0, size().width(), size().height());
   painter.fillRect(background, Qt::white);

   // If the m_objects array hasn't been allocated, return.
   if (!m_objects) {
      select_index = -1;
      return;
   }

   QPen     defaultPen(Qt::red);
   painter.setPen(defaultPen);

   const float radius = 20.0;

   typedef std::vector<WSObject>::const_iterator Iterator;


   // define scale_factor here.
   x_scale_factor = (limits.max().x - limits.min().x) / ((float) size().width());
   y_scale_factor = (limits.max().y - limits.min().y) / ((float) size().height());

   float min_object_x = limits.min().x;
   float min_object_y = limits.min().y;
   
   for (Iterator i = m_objects->begin(); i != m_objects->end(); i++)
   {

      // Change the pen color using the WSObject's r_display, g_display, b_display..
      painter.setPen(QPen(QColor(i->r_display, i->g_display, i->b_display)));

      float draw_at_x = (((*i).x_position - min_object_x) / x_scale_factor) - radius;
      float draw_at_y = (((*i).y_position - min_object_y) / y_scale_factor) - radius;
      painter.drawEllipse(draw_at_x, draw_at_y, radius, radius);
   }

   if (select_index >= 0)
   {
      float radius_big = radius * 1.15;
      painter.setPen(QPen(Qt::red));
      float draw_at_x = (((*m_objects)[select_index].x_position - min_object_x) / x_scale_factor) - radius_big;
      float draw_at_y = (((*m_objects)[select_index].y_position - min_object_y) / y_scale_factor) - radius_big;
      painter.drawEllipse(draw_at_x, draw_at_y, radius_big, radius_big);
   }

}
