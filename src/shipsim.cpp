#include <QApplication>

#include <rclcpp/rclcpp.hpp>

#include "shipsim/ship_frame.h"

class ShipApp : public QApplication
{
public:
  rclcpp::Node::SharedPtr nh_;

  explicit ShipApp(int &argc, char **argv)
      : QApplication(argc, argv)
  {
    rclcpp::init(argc, argv);
    nh_ = rclcpp::Node::make_shared("shipsim");
  }

  ~ShipApp()
  {
    rclcpp::shutdown();
  }

  int exec()
  {
    shipsim::ShipFrame frame(nh_);
    frame.show();

    return QApplication::exec();
  }
};

int main(int argc, char **argv)
{
  ShipApp app(argc, argv);
  return app.exec();
}
