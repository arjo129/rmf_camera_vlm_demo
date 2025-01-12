//! [registerSampleSystem]
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/common/Image.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>

#include <random>

namespace random_camera_poser {
  /// \brief Simple plugin that generates multiple poses for a camera.
  /// Moves camera around the world.
  class RandomCameraPoser:
    // This class is a system.
    public gz::sim::System,
    // This class also implements the ISystemPostUpdate interface.
    public gz::sim::ISystemPreUpdate,
    //
    public gz::sim::ISystemConfigure
  {
    public: RandomCameraPoser();

    public: ~RandomCameraPoser() override;

    public: void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &/*_eventMgr*/) override;

    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    private: gz::sim::Model model;

    private: std::chrono::steady_clock::duration lastTimeReposed{0};

    private: double minX, minY, minZ, maxX, maxY, maxZ;

    private: gz::math::Pose3<double> currPose;

    private: /// \brief Holds data to set as the next image
    std::optional<gz::msgs::Image> imageMsg;

    /// \brief Node for communication.
    public: gz::transport::Node node;

    private: void OnImageMsg(const gz::msgs::Image &_msg) {
      this->imageMsg = _msg;
      std::stringstream ss;
      ss << "data/rgb_image_"<< currPose.Pos().X() << "_"
        << currPose.Pos().Y() << "_"
        << currPose.Pos().Z() << "_"
        << currPose.Rot().Euler().X() << "_"
        << currPose.Rot().Euler().Y() << "_"
        << currPose.Rot().Euler().Z() << ".png";

      gz::common::Image img;
      gz::common::Image::PixelFormatType pixelFormat =
            gz::common::Image::ConvertPixelFormat(
            gz::msgs::ConvertPixelFormatType(
            this->imageMsg->pixel_format_type()));

      img.SetFromData((const unsigned char*)_msg.data().c_str(), _msg.width(), _msg.height(), pixelFormat);
      img.SavePNG(ss.str());
    }
  };
}

// Include a line in your source file for each interface implemented.
GZ_ADD_PLUGIN(
    random_camera_poser::RandomCameraPoser,
    gz::sim::System,
    random_camera_poser::RandomCameraPoser::ISystemPreUpdate,
    random_camera_poser::RandomCameraPoser::ISystemConfigure)
//! [registerSampleSystem]
//! [implementSampleSystem]
using namespace random_camera_poser;

double getRandomDouble(double min, double max) {
    std::random_device rd;
    std::mt19937 generator(rd()); // Mersenne Twister engine
    std::uniform_real_distribution<> distribution(min, max);
    return distribution(generator);
}

RandomCameraPoser::RandomCameraPoser()
{
}

RandomCameraPoser::~RandomCameraPoser()
{
}

void RandomCameraPoser::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &/*_eventMgr*/)
{
  this->model = gz::sim::Model(_entity);
  this->minX = _sdf->Get<double>("min_x", 0.0).first;
  this->maxX = _sdf->Get<double>("max_x", 0.0).first;
  this->minY = _sdf->Get<double>("min_y", 0.0).first;
  this->maxY = _sdf->Get<double>("max_y", 0.0).first;
  this->minZ = _sdf->Get<double>("min_z", 0.0).first;
  this->maxZ = _sdf->Get<double>("max_z", 0.0).first;

  this->node.Subscribe("camera", &RandomCameraPoser::OnImageMsg,
      this);
}
void RandomCameraPoser::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if(_info.paused)
    return;

  using namespace std::chrono_literals;
  if((_info.simTime - this->lastTimeReposed) > 500ms) {
    gz::math::Vector3<double> randPos(
        getRandomDouble(this->minX, this->maxX),
        getRandomDouble(this->minY, this->maxY),
        getRandomDouble(this->minZ, this->maxZ)
    );

    gz::math::Quaternion<double> randOrientation(
        getRandomDouble(-M_PI/4, M_PI/4),
        getRandomDouble(-M_PI/4, M_PI/4),
        getRandomDouble(-M_PI, M_PI));

    gz::math::Pose3<double> randPose(
        randPos, randOrientation
    );
    currPose = randPose;
    this->model.SetWorldPoseCmd(_ecm, randPose);
    this->imageMsg = std::nullopt;
  }
}