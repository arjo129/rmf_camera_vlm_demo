//! [registerSampleSystem]
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
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

    this->model.SetWorldPoseCmd(_ecm, randPose);
  }
}