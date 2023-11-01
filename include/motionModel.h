#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include <BayesFilters/LinearStateModel.h>

class MotionModel : public bfl::LinearStateModel
{
public:
    MotionModel(const double& sample_time, const double& power_spectral_density);

    Eigen::MatrixXd getStateTransitionMatrix() override;
    Eigen::MatrixXd getNoiseCovarianceMatrix();

    bfl::VectorDescription getStateDescription() override;

    /* Useful in case the user wants to set properties of the model online. */
    bool setProperty(const std::string& property) override;

private:
    /*
     * Implement x+1 = Fx + noise
     * noise ~ Normal with covariance Q
     * where x = [bbox_x, bbox_y, bbox_x_dot, bbox_y_dot]
     * and (bbox_x, bbox_y) represents the center of the bounding box, for the sake of simplicity.
     */
    Eigen::MatrixXd F_;
    Eigen::MatrixXd Q_;

    /**
     * Define state and output description.
     * - state: 4, 0, 4 means 4 linear components, 0 circular components and 4 noise components as the state is noisy
     * - output: already provided by bfl for a LinearStateModel
     *
     * @warning: noise components are actually only required if using sigma point-based filters and a generic non-linear measurement model is used
     */
    bfl::VectorDescription state_des_ = bfl::VectorDescription(4, 0, 4);
};

#endif /* MOTION_MODEL_H */
