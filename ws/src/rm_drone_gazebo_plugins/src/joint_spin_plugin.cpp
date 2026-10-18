// 让关节匀速旋转（前哨站中部装甲、能量机关）。
// random_direction=true 时方向由环境变量 RM_SIM_SEED 决定，同一 seed 下所有实例方向一致，
// 满足“红蓝双方前哨站旋转方向一致”的规则；mirror_joint 以相反方向旋转（能量机关共轴反转）。
#include <cstdlib>
#include <functional>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace rm_drone
{

class JointSpinPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    joint_ = model->GetJoint(sdf->Get<std::string>("joint"));
    if (sdf->HasElement("mirror_joint")) {
      mirror_ = model->GetJoint(sdf->Get<std::string>("mirror_joint"));
    }
    if (!joint_) {
      gzerr << "joint_spin_plugin: joint not found in " << model->GetName() << "\n";
      return;
    }
    speed_ = sdf->HasElement("velocity") ? sdf->Get<double>("velocity") : 1.0;
    stop_after_ = sdf->HasElement("stop_after") ? sdf->Get<double>("stop_after") : 0.0;
    ramp_ = sdf->HasElement("ramp_time") ? sdf->Get<double>("ramp_time") : 0.0;
    if (sdf->HasElement("random_direction") && sdf->Get<bool>("random_direction")) {
      const char * seed = std::getenv("RM_SIM_SEED");
      const size_t h = std::hash<std::string>{}(seed ? seed : "rmuc2027");
      if (h % 2) {speed_ = -speed_;}
    }
    conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&JointSpinPlugin::OnUpdate, this));
  }

private:
  void Drive(gazebo::physics::JointPtr j, double v)
  {
    j->SetParam("fmax", 0, 500.0);
    j->SetParam("vel", 0, v);
  }

  void OnUpdate()
  {
    const double t = model_->GetWorld()->SimTime().Double();
    double v = speed_;
    if (ramp_ > 0.0 && t < ramp_) {v *= t / ramp_;}
    if (stop_after_ > 0.0 && t > stop_after_) {v = 0.0;}
    Drive(joint_, v);
    if (mirror_) {Drive(mirror_, -v);}
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::JointPtr joint_, mirror_;
  gazebo::event::ConnectionPtr conn_;
  double speed_{1.0}, stop_after_{0.0}, ramp_{0.0};
};

GZ_REGISTER_MODEL_PLUGIN(JointSpinPlugin)

}  // namespace rm_drone
