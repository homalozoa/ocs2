#include <ocs2_anymal_bear/AnymalBearInterface.h>
#include <ocs2_anymal_bear/AnymalBearPyBindings.h>

#include <ocs2_quadruped_interface/QuadrupedSlqMpc.h>

namespace anymal {

AnymalBearPyBindings::AnymalBearPyBindings(std::string taskName, bool async) : Base(async), taskName_(std::move(taskName)) {
  auto anymalBearInterface = getAnymalBearInterface(getTaskFileFolderBear(taskName_));
  ocs2::mpc::Settings mpcSettings = ocs2::mpc::loadSettings(getTaskFilePathBear(taskName_));
  ocs2::ddp::Settings ddpSettings = ocs2::ddp::loadSettings(getTaskFilePathBear(taskName_));

  init(*anymalBearInterface, switched_model::getMpc(*anymalBearInterface, mpcSettings, ddpSettings));

  penalty_.reset(new ocs2::RelaxedBarrierPenalty(ddpSettings.ddpSettings_.inequalityConstraintMu_,
                                                 ddpSettings.ddpSettings_.inequalityConstraintDelta_));
}

void AnymalBearPyBindings::visualizeTrajectory(const scalar_array_t& t, const state_vector_array_t& x, const input_vector_array_t& u,
                                               double speed) {
  if (!visualizer_) {
    auto anymalBearInterface = getAnymalBearInterface(taskName_);
    int fake_argc = 1;
    char arg0[] = "no_name";
    char* fake_argv[] = {arg0};
    ros::init(fake_argc, fake_argv, "anymal_visualization_node");
    ros::NodeHandle n;
    visualizer_.reset(new visualizer_t(anymalBearInterface->getKinematicModel(), anymalBearInterface->getComModel(), n));
  }

  assert(t.size() == x.size());
  visualizer_t::system_observation_array_t observations(t.size());

  const bool inputProvided = !u.empty();

  for (int i = 0; i < t.size(); i++) {
    observations[i].time() = t[i];
    observations[i].state() = x[i];
    if (inputProvided) {
      observations[i].input() = u[i];
    }
  }

  visualizer_->publishTrajectory(observations, speed);
}

}  // namespace anymal
