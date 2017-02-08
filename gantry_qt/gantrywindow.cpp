#include "gantrywindow.h"
#include "ui_gantrywindow.h"

//#define DEBUG_LIVE_VIEWER

using std::cout;
using std::endl;
using pcl::OpenNIGrabber;

const int DECREASE_X = 1;
const int DECREASE_Y = 2;
const int DECREASE_Z = 3;
const int DECREASE_J4 = 4;
const int DECREASE_J5 = 5;
const int DECREASE_J6 = 6;
const int INCREASE_X = 101;
const int INCREASE_Y = 102;
const int INCREASE_Z = 103;
const int INCREASE_J4 = 104;
const int INCREASE_J5 = 105;
const int INCREASE_J6 = 106;


GantryWindow::GantryWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::GantryWindow),
    viewer(new pcl::visualization::PCLVisualizer("Viewer", false)) // false = no interactor
{

    ui->setupUi(this);

    m_logger = new QLogger(ui->statusDisplay);
//  eng = new Engine3(viewer, m_logger); 
//  eng = new Engine2(viewer, m_logger);
    ui->itemDisplay->m_objects = eng->m_objects;

   live_viewer = new LiveViewer(ui->qvtkWidgetLive, viewer, eng->vp_calibration_axes);
    ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();



    // CHECKED_CONNECTs
    CHECKED_CONNECT(ui->homeButton, SIGNAL(pressed()), this, SLOT(home()));
    CHECKED_CONNECT(ui->loadCalButton, SIGNAL(pressed()), this, SLOT(load_calibration()));
    CHECKED_CONNECT(ui->moveButton, SIGNAL(pressed()), this, SLOT(move_to_location()));
    CHECKED_CONNECT(ui->scanButton, SIGNAL(pressed()), this, SLOT(find_objects()));
    CHECKED_CONNECT(ui->positionButton, SIGNAL(pressed()), this, SLOT(update_position()));
    CHECKED_CONNECT(ui->prevItemButton, SIGNAL(pressed()), ui->itemDisplay, SLOT(cycle()));
    CHECKED_CONNECT(ui->nextItemButton, SIGNAL(pressed()), ui->itemDisplay, SLOT(cycle()));
    CHECKED_CONNECT(ui->centerItemButton, SIGNAL(pressed()), this, SLOT(center_item()));
    CHECKED_CONNECT(ui->cal_x_spin, SIGNAL(valueChanged(double)), this, SLOT(changeXCal(double)));
    CHECKED_CONNECT(ui->cal_y_spin, SIGNAL(valueChanged(double)), this, SLOT(changeYCal(double)));
    CHECKED_CONNECT(ui->clearObjectsButton, SIGNAL(pressed()), this, SLOT(clear_objects()));
    CHECKED_CONNECT(ui->startLiveButton, SIGNAL(pressed()), this, SLOT(startLiveFeed()));
    CHECKED_CONNECT(ui->stopLiveButton, SIGNAL(pressed()), this, SLOT(stopLiveFeed()));

    ui->cal_y_spin->setMinimum(-100.0);
    ui->cal_x_spin->setMinimum(-100.0);

    // Signal-mapping the slots for the "move" "here" buttons
    QSignalMapper * buttonMapper = new QSignalMapper(this);
    buttonMapper->setMapping(decreasexButton, DECREASE_X);
    buttonMapper->setMapping(decreaseyButton, DECREASE_Y);
    buttonMapper->setMapping(decreasezButton, DECREASE_Z);
    buttonMapper->setMapping(decreasej4Button, DECREASE_J4);
    buttonMapper->setMapping(decreasej5Button, DECREASE_J5);
    buttonMapper->setMapping(decreasej6Button, DECREASE_J6);
    buttonMapper->setMapping(increasexButton, INCREASE_X);
    buttonMapper->setMapping(increaseyButton, INCREASE_Y);
    buttonMapper->setMapping(increasezButton, INCREASE_Z);
    buttonMapper->setMapping(increasej4Button, INCREASE_J4);
    buttonMapper->setMapping(increasej5Button, INCREASE_J5);
    buttonMapper->setMapping(increasej6Button, INCREASE_J6);
    CHECKED_CONNECT(decreasexButton, SIGNAL(pressed()), signalMapper, SLOT(map()));
    CHECKED_CONNECT(decreaseyButton, SIGNAL(pressed()), signalMapper, SLOT(map()));
    CHECKED_CONNECT(decreasezButton, SIGNAL(pressed()), signalMapper, SLOT(map()));
    CHECKED_CONNECT(decreasej4Button, SIGNAL(pressed()), signalMapper, SLOT(map()));
    CHECKED_CONNECT(decreasej5Button, SIGNAL(pressed()), signalMapper, SLOT(map()));
    CHECKED_CONNECT(decreasej6Button, SIGNAL(pressed()), signalMapper, SLOT(map()));
    CHECKED_CONNECT(increasexButton, SIGNAL(pressed()), signalMapper, SLOT(map()));
    CHECKED_CONNECT(increaseyButton, SIGNAL(pressed()), signalMapper, SLOT(map()));
    CHECKED_CONNECT(increasezButton, SIGNAL(pressed()), signalMapper, SLOT(map()));
    CHECKED_CONNECT(increasej4Button, SIGNAL(pressed()), signalMapper, SLOT(map()));
    CHECKED_CONNECT(increasej5Button, SIGNAL(pressed()), signalMapper, SLOT(map()));
    CHECKED_CONNECT(increasej6Button, SIGNAL(pressed()), signalMapper, SLOT(map()));
    QObject::connect(buttonMapper, SIGNAL(mapped(int)), this, SLOT(manualMove(int)));
    
    // Thread
    et = new EngThread(eng, eng->m_robot->currentPos());
    QObject::connect(et, SIGNAL(finished()), et, SLOT(quit()));
}

void GantryWindow::startLiveFeed() { 
   live_viewer->start(); 
}
void GantryWindow::stopLiveFeed() { live_viewer->stop(); }

void
GantryWindow::home()
{
   eng->m_robot->runCmd("RUN HOME5");
   update_display();
}

void 
GantryWindow::center_item()
{
   eng->move_to_object(ui->itemDisplay->get_index());
   update_display();
}

void 
GantryWindow::load_calibration()
{
   eng->load();
   RobotPosition new_pos = eng->m_robot->currentPos();
   update_display();
}


void
GantryWindow::move_to_location()
{
// QString  editor_text = ui->positionInput->toPlainText();
   std::string editor_text = ui->positionInput->toPlainText().toUtf8().constData();

   RobotPosition new_pos = eng->m_robot->currentPos();

   using stringmanip::arg_list;
   arg_list args = stringmanip::split(editor_text);

   while (!args.empty()) {
      std::string axis = args.front();
      float value;
      args.pop_front();
      if (args.empty()) {
         cout << "Illegal command." << endl;
         return;
      } else {
         value = atof(args.front().c_str());
         args.pop_front();
      }

      if      (axis == "x")      new_pos.x = value;
      else if (axis == "y")      new_pos.y = value;
      else if (axis == "z")      new_pos.z = value;
      else if (axis == "j4")     new_pos.j4 = value;
      else if (axis == "j5")     new_pos.j5 = value;
      else if (axis == "j6")     new_pos.j6 = value;
      else {
         cout << "Unrecognized argument: <" << axis << ">" << endl;
         return;
      }
   }

   std::cout << "New Position: " << std::endl;
   std::cout << new_pos << std::endl;

// eng->m_robot->moveTo(new_pos);
  
// Need to be updating the other view in another thread
// eng->moveTo(new_pos);

   et->setPos(new_pos);
   et->start();

// std::string reply;
// eng->m_robot->controller >> reply;

// std::cout << "REPLY: " << reply << endl;
// 
// update_position();
   update_display();
}

void GantryWindow::find_objects()
{
// eng->locate(true);
   eng->scan();
   update_display();

   // Trigger a paint event?
}

void GantryWindow::update_position()
{
   const RobotPosition curr = eng->getPosition();

   ui->x_pte->setPlainText(QString::number(curr.x));
   ui->y_pte->setPlainText(QString::number(curr.y));
   ui->z_pte->setPlainText(QString::number(curr.z));
   ui->j4_pte->setPlainText(QString::number(curr.j4));
   ui->j5_pte->setPlainText(QString::number(curr.j5));
   ui->j6_pte->setPlainText(QString::number(curr.j6));

   ui->itemDisplay->update_robot_position(curr);
   update_display();
}

void GantryWindow::update_display()
{
   ui->itemDisplay->update();
}

GantryWindow::~GantryWindow()
{
    delete ui;
}

void GantryWindow::changeXCal(double d) {
   current_x_cal = d;
   cout << "current_x_cal = " << current_x_cal << endl;
   eng->move_claw_line(current_x_cal, current_y_cal, 0);
   clear_objects();
}

void GantryWindow::changeYCal(double d) {
   current_y_cal = d;
   cout << "current_y_cal = " << current_y_cal << endl;
   eng->move_claw_line(current_x_cal, current_y_cal, 0);
   clear_objects();
}

void GantryWindow::clear_objects()
{
   eng->clear_objects();
   update_display();
}

void GantryWindow::manualMove(int direction)
{
   // degrees: X01, X01, X12
   // angles: X10, X11, X12
   // increase: 0XX
   // decrease: 1XX
   bool decreasing = (direction >= 100);
   bool angles     = ((direction % 100) > 3);

   if (direction % 100 == 1) {

   } else if (direction % 100 == 2) {
   } else if (direction % 100 == 3) {
   } else if (direction % 100 == 4) {
   } else if (direction % 100 == 5) {
   } else if (direction % 100 == 6) {
}
