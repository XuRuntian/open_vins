#include <cfenv>
#include <iostream>
#include <stdexcept>
#include <string>

#include "feat/Feature.h"
#include "feat/FeatureInitializer.h"
#include "state/State.h"
#include "update/UpdaterMSCKF.h"
#include "update/UpdaterSLAM.h"

int main(int argc, char **argv) {
  const bool slam = argc > 1 && std::string(argv[1]) == "slam";
  ov_msckf::StateOptions state_options;
  state_options.num_cameras = 1;
  auto state = std::make_shared<ov_msckf::State>(state_options);
  state->_clones_IMU[0.] = std::make_shared<ov_type::PoseJPL>();
  state->_clones_IMU[1.] = std::make_shared<ov_type::PoseJPL>();
  auto feature = std::make_shared<ov_core::Feature>();
  feature->featid = 1000;
  feature->to_delete = false;
  feature->p_FinA.setZero();
  feature->p_FinG.setZero();
  feature->timestamps[0] = {0., 1.};
  feature->uvs_norm[0] = {Eigen::Vector2f(0., 0.), Eigen::Vector2f(0.2, 0.1)};
  feature->uvs = feature->uvs_norm;

  ov_core::FeatureInitializerOptions initializer_options;
  initializer_options.triangulate_1d = argc > 2 && std::string(argv[2]) == "1d";
  initializer_options.refine_features = true;
  ov_core::FeatureInitializer initializer(initializer_options);
  std::unordered_map<size_t, std::unordered_map<double, ov_core::FeatureInitializer::ClonePose>> clones;
  clones[0][0.] = ov_core::FeatureInitializer::ClonePose();
  clones[0][1.] = ov_core::FeatureInitializer::ClonePose();
  const bool triangulated = initializer_options.triangulate_1d
      ? initializer.single_triangulation_1d(feature, clones)
      : initializer.single_triangulation(feature, clones);
  if (triangulated) throw std::runtime_error("Fixture must fail triangulation");

  ov_msckf::UpdaterOptions options;
  ov_msckf::UpdaterMSCKF msckf(options, initializer_options);
  ov_msckf::UpdaterSLAM slam_updater(options, options, initializer_options);
  std::vector<std::shared_ptr<ov_core::Feature>> features{feature};
  std::feclearexcept(FE_ALL_EXCEPT);
  // Optional Linux trap mode reproduces the captured SIGFPE at the real call site.
  if (argc > 3 && std::string(argv[3]) == "trap") feenableexcept(FE_INVALID);
  if (slam) slam_updater.delayed_init(state, features);
  else msckf.update(state, features);
  if (std::fetestexcept(FE_INVALID | FE_DIVBYZERO))
    throw std::runtime_error("Failed triangulation reached invalid inverse-depth arithmetic");
  if (!features.empty() || !feature->to_delete)
    throw std::runtime_error("Failed feature must be rejected");
  std::cout << "PASS: failed triangulation rejected without invalid arithmetic\n";
}
