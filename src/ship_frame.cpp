#include "shipsim/ship_frame.h"

#include <QPointF>

#include <cstdlib>
#include <ctime>

#define DEFAULT_BG_R 0x45
#define DEFAULT_BG_G 0x56
#define DEFAULT_BG_B 0xff

namespace shipsim
{

  ShipFrame::ShipFrame(rclcpp::Node::SharedPtr &node_handle, QWidget *parent, Qt::WindowFlags f)
      : QFrame(parent, f), path_image_(1200, 900, QImage::Format_ARGB32), path_painter_(&path_image_), frame_count_(0), id_counter_(0)
  {
    setFixedSize(1200, 900);
    setWindowTitle("ShipSim");

    srand(time(NULL));

    update_timer_ = new QTimer(this);
    update_timer_->setInterval(16);
    update_timer_->start();

    connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

    nh_ = node_handle;
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = 0;
    range.step = 1;
    range.to_value = 255;
    rcl_interfaces::msg::ParameterDescriptor background_r_descriptor;
    background_r_descriptor.description = "Red channel of the background color";
    background_r_descriptor.integer_range.push_back(range);
    rcl_interfaces::msg::ParameterDescriptor background_g_descriptor;
    background_g_descriptor.description = "Green channel of the background color";
    background_g_descriptor.integer_range.push_back(range);
    rcl_interfaces::msg::ParameterDescriptor background_b_descriptor;
    background_b_descriptor.description = "Blue channel of the background color";
    background_b_descriptor.integer_range.push_back(range);
    nh_->declare_parameter("background_r", rclcpp::ParameterValue(DEFAULT_BG_R), background_r_descriptor);
    nh_->declare_parameter("background_g", rclcpp::ParameterValue(DEFAULT_BG_G), background_g_descriptor);
    nh_->declare_parameter("background_b", rclcpp::ParameterValue(DEFAULT_BG_B), background_b_descriptor);

    QVector<QString> ships;
    ships.append("ship_brown.png");

    QString images_path = (ament_index_cpp::get_package_share_directory("shipsim") + "/images/").c_str();
    for (int i = 0; i < ships.size(); ++i)
    {
      QImage img;
      img.load(images_path + ships[i]);
      ship_images_.append(img);
    }

    meter_ = ship_images_[0].height();

    clear();

    clear_srv_ = nh_->create_service<std_srvs::srv::Empty>("clear", std::bind(&ShipFrame::clearCallback, this, std::placeholders::_1, std::placeholders::_2));
    reset_srv_ = nh_->create_service<std_srvs::srv::Empty>("reset", std::bind(&ShipFrame::resetCallback, this, std::placeholders::_1, std::placeholders::_2));
    spawn_srv_ = nh_->create_service<shipsim::srv::Spawn>("spawn", std::bind(&ShipFrame::spawnCallback, this, std::placeholders::_1, std::placeholders::_2));
    kill_srv_ = nh_->create_service<shipsim::srv::Kill>("kill", std::bind(&ShipFrame::killCallback, this, std::placeholders::_1, std::placeholders::_2));

    rclcpp::QoS qos(rclcpp::KeepLast(100), rmw_qos_profile_sensor_data);
    parameter_event_sub_ = nh_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events", qos, std::bind(&ShipFrame::parameterEventCallback, this, std::placeholders::_1));

    RCLCPP_INFO(nh_->get_logger(), "Starting shipsim with node name %s", nh_->get_node_names()[0].c_str());

    width_in_meters_ = (width() - 1) / meter_;
    height_in_meters_ = (height() - 1) / meter_;
    spawnShip("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);

    // spawn all available ship types
    if (false)
    {
      for (int index = 0; index < ships.size(); ++index)
      {
        QString name = ships[index];
        name = name.split(".").first();
        name.replace(QString("-"), QString(""));
        spawnShip(name.toStdString(), 1.0f + 1.5f * (index % 7), 1.0f + 1.5f * (index / 7), static_cast<float>(PI) / 2.0f, index);
      }
    }
  }

  ShipFrame::~ShipFrame()
  {
    delete update_timer_;
  }

  bool ShipFrame::spawnCallback(const shipsim::srv::Spawn::Request::SharedPtr req, shipsim::srv::Spawn::Response::SharedPtr res)
  {
    std::string name = spawnShip(req->name, req->x, req->y, req->theta);
    if (name.empty())
    {
      RCLCPP_ERROR(nh_->get_logger(), "A ship named [%s] already exists", req->name.c_str());
      return false;
    }

    res->name = name;

    return true;
  }

  bool ShipFrame::killCallback(const shipsim::srv::Kill::Request::SharedPtr req, shipsim::srv::Kill::Response::SharedPtr)
  {
    M_Ship::iterator it = ships_.find(req->name);
    if (it == ships_.end())
    {
      RCLCPP_ERROR(nh_->get_logger(), "Tried to kill ship [%s], which does not exist", req->name.c_str());
      return false;
    }

    ships_.erase(it);
    update();

    return true;
  }

  void ShipFrame::parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
  {
    // only consider events from this node
    if (event->node == nh_->get_fully_qualified_name())
    {
      // since parameter events for this even aren't expected frequently just always call update()
      update();
    }
  }

  bool ShipFrame::hasShip(const std::string &name)
  {
    return ships_.find(name) != ships_.end();
  }

  std::string ShipFrame::spawnShip(const std::string &name, float x, float y, float angle)
  {
    return spawnShip(name, x, y, angle, rand() % ship_images_.size());
  }

  std::string ShipFrame::spawnShip(const std::string &name, float x, float y, float angle, size_t index)
  {
    std::string real_name = name;
    if (real_name.empty())
    {
      do
      {
        std::stringstream ss;
        ss << "ship" << ++id_counter_;
        real_name = ss.str();
      } while (hasShip(real_name));
    }
    else
    {
      if (hasShip(real_name))
      {
        return "";
      }
    }

    ShipPtr t = std::make_shared<Ship>(nh_, real_name, ship_images_[static_cast<int>(index)], QPointF(x, height_in_meters_ - y), angle);
    ships_[real_name] = t;
    update();

    RCLCPP_INFO(nh_->get_logger(), "Spawning ship [%s] at x=[%f], y=[%f], theta=[%f]", real_name.c_str(), x, y, angle);

    return real_name;
  }

  void ShipFrame::clear()
  {
    // make all pixels fully transparent
    path_image_.fill(qRgba(255, 255, 255, 0));
    update();
  }

  void ShipFrame::onUpdate()
  {
    if (!rclcpp::ok())
    {
      close();
      return;
    }

    rclcpp::spin_some(nh_);

    updateShips();
  }

  void ShipFrame::paintEvent(QPaintEvent *)
  {
    QPainter painter(this);

    int r = DEFAULT_BG_R;
    int g = DEFAULT_BG_G;
    int b = DEFAULT_BG_B;
    nh_->get_parameter("background_r", r);
    nh_->get_parameter("background_g", g);
    nh_->get_parameter("background_b", b);
    QRgb background_color = qRgb(r, g, b);
    painter.fillRect(0, 0, width(), height(), background_color);

    painter.drawImage(QPoint(0, 0), path_image_);

    M_Ship::iterator it = ships_.begin();
    M_Ship::iterator end = ships_.end();
    for (; it != end; ++it)
    {
      it->second->paint(painter);
    }
  }

  void ShipFrame::updateShips()
  {
    if (last_ship_update_.nanoseconds() == 0)
    {
      last_ship_update_ = nh_->now();
      return;
    }

    bool modified = false;
    M_Ship::iterator it = ships_.begin();
    M_Ship::iterator end = ships_.end();
    for (; it != end; ++it)
    {
      modified |= it->second->update(0.001 * update_timer_->interval(), path_painter_, path_image_, width_in_meters_, height_in_meters_);
    }
    if (modified)
    {
      update();
    }

    ++frame_count_;
  }

  bool ShipFrame::clearCallback(const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr)
  {
    RCLCPP_INFO(nh_->get_logger(), "Clearing shipsim.");
    clear();
    return true;
  }

  bool ShipFrame::resetCallback(const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr)
  {
    RCLCPP_INFO(nh_->get_logger(), "Resetting shipsim.");
    ships_.clear();
    id_counter_ = 0;
    spawnShip("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);
    clear();
    return true;
  }

}
