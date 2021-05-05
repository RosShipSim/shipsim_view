#ifndef SHIPSIM_SHIP_H
#define SHIPSIM_SHIP_H

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <shipsim/action/rotate_absolute.hpp>
#include <shipsim/msg/color.hpp>
#include <shipsim/msg/pose.hpp>
#include <shipsim/srv/set_pen.hpp>
#include <shipsim/srv/teleport_absolute.hpp>
#include <shipsim/srv/teleport_relative.hpp>
#endif

#include <QImage>
#include <QPainter>
#include <QPen>
#include <QPointF>

#define PI 3.14159265
#define TWO_PI 2.0 * PI

namespace shipsim
{

  class Ship
  {
  public:
    using RotateAbsoluteGoalHandle = rclcpp_action::ServerGoalHandle<shipsim::action::RotateAbsolute>;

    Ship(rclcpp::Node::SharedPtr &nh, const std::string &real_name, const QImage &ship_image, const QPointF &pos, float orient);

    bool update(double dt, QPainter &path_painter, const QImage &path_image, qreal canvas_width, qreal canvas_height);
    void paint(QPainter &painter);

  private:
    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr vel);
    bool setPenCallback(const shipsim::srv::SetPen::Request::SharedPtr, shipsim::srv::SetPen::Response::SharedPtr);
    bool teleportRelativeCallback(const shipsim::srv::TeleportRelative::Request::SharedPtr, shipsim::srv::TeleportRelative::Response::SharedPtr);
    bool teleportAbsoluteCallback(const shipsim::srv::TeleportAbsolute::Request::SharedPtr, shipsim::srv::TeleportAbsolute::Response::SharedPtr);
    void rotateAbsoluteAcceptCallback(const std::shared_ptr<RotateAbsoluteGoalHandle>);

    void rotateImage();

    rclcpp::Node::SharedPtr nh_;

    QImage ship_image_;
    QImage ship_rotated_image_;

    QPointF pos_;
    qreal orient_;

    qreal lin_vel_x_;
    qreal lin_vel_y_;
    qreal ang_vel_;
    bool pen_on_;
    QPen pen_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    rclcpp::Publisher<shipsim::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Publisher<shipsim::msg::Color>::SharedPtr color_pub_;
    rclcpp::Service<shipsim::srv::SetPen>::SharedPtr set_pen_srv_;
    rclcpp::Service<shipsim::srv::TeleportRelative>::SharedPtr teleport_relative_srv_;
    rclcpp::Service<shipsim::srv::TeleportAbsolute>::SharedPtr teleport_absolute_srv_;
    rclcpp_action::Server<shipsim::action::RotateAbsolute>::SharedPtr rotate_absolute_action_server_;

    std::shared_ptr<RotateAbsoluteGoalHandle> rotate_absolute_goal_handle_;
    std::shared_ptr<shipsim::action::RotateAbsolute::Feedback> rotate_absolute_feedback_;
    std::shared_ptr<shipsim::action::RotateAbsolute::Result> rotate_absolute_result_;
    qreal rotate_absolute_start_orient_;

    rclcpp::Time last_command_time_;

    float meter_;

    struct TeleportRequest
    {
      TeleportRequest(float x, float y, qreal _theta, qreal _linear, bool _relative)
          : pos(x, y), theta(_theta), linear(_linear), relative(_relative)
      {
      }

      QPointF pos;
      qreal theta;
      qreal linear;
      bool relative;
    };
    typedef std::vector<TeleportRequest> V_TeleportRequest;
    V_TeleportRequest teleport_requests_;
  };
  typedef std::shared_ptr<Ship> ShipPtr;

}

#endif
