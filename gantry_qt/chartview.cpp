#include "chartview.h"


ChartView::ChartView(QWidget *parent) :
    QChartView(new QChart(), parent)
{
    /*
    QScatterSeries *series0 = new QScatterSeries();

    series0->setName("test1");
    series0->append(0, 3);
    series0->append(3, 0);
    series0->append(3, 3);

    chart()->addSeries(series0);
    chart()->setTitle("Test Title");
    chart()->createDefaultAxes();
    chart()->setDropShadowEnabled(false);
    */
}

void
ChartView::updateSeries()
{
    // read m_series


	 typedef SurfaceMap::Coordinate							Coordinate;
	 typedef std::vector<Coordinate>::const_iterator	CoordIt;

     QScatterSeries * points_2d = new QScatterSeries();

    for (CoordIt it = surface->coordinates.begin(); it != surface->coordinates.end(); ++it)
	 {
		// Fill a series
		points_2d->append(it->x, it->y);
	 }

	 chart()->addSeries(points_2d);
	 chart()->setTitle("2D Points");
	 chart()->createDefaultAxes();
	 chart()->setDropShadowEnabled(false);
}


