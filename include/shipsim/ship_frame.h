#include <QFrame>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QVector>

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rcl_interfaces/msg/parameter_event.hpp>
#include <std_srvs/srv/empty.hpp>
#include <shipsim/srv/spawn.hpp>
#include <shipsim/srv/kill.hpp>
#include <map>

#include "ship.h"
#endif

namespace shipsim
{

  class ShipFrame : public QFrame
  {
    Q_OBJECT
  public:
    ShipFrame(rclcpp::Node::SharedPtr &node_handle, QWidget *parent = 0, Qt::WindowFlags f = Qt::WindowFlags());
    ~ShipFrame();

    std::string spawnShip(const std::string &name, float x, float y, float angle);
    std::string spawnShip(const std::string &name, float x, float y, float angle, size_t index);

  protected:
    void paintEvent(QPaintEvent *event);

  private slots:
    void onUpdate();

  private:
    void updateShips();
    void clear();
    bool hasShip(const std::string &name);

    bool clearCallback(const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr);
    bool resetCallback(const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr);
    bool spawnCallback(const shipsim::srv::Spawn::Request::SharedPtr, shipsim::srv::Spawn::Response::SharedPtr);
    bool killCallback(const shipsim::srv::Kill::Request::SharedPtr, shipsim::srv::Kill::Response::SharedPtr);

    void parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr);

    rclcpp::Node::SharedPtr nh_;

    QTimer *update_timer_;
    QImage path_image_;
    QPainter path_painter_;

    uint64_t frame_count_;

    rclcpp::Time last_ship_update_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
    rclcpp::Service<shipsim::srv::Spawn>::SharedPtr spawn_srv_;
    rclcpp::Service<shipsim::srv::Kill>::SharedPtr kill_srv_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

    typedef std::map<std::string, ShipPtr> M_Ship;
    M_Ship ships_;
    uint32_t id_counter_;

    QVector<QImage> ship_images_;

    float meter_;
    float width_in_meters_;
    float height_in_meters_;
  };

}
