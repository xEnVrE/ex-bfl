#ifndef MEASUREMENT_MODEL_H
#define MEASUREMENT_MODEL_H

#include <BayesFilters/LinearMeasurementModel.h>

class MeasurementModel : public bfl::LinearMeasurementModel
{
public:
    MeasurementModel(const double& noise_covariance);

    std::pair<bool, Eigen::MatrixXd> getNoiseCovarianceMatrix() const override;
    Eigen::MatrixXd getMeasurementMatrix() const override;
    bool freeze(const bfl::Data&) override;
    std::pair<bool, bfl::Data> measure(const bfl::Data&) const override;

    bfl::VectorDescription getInputDescription() const;
    bfl::VectorDescription getMeasurementDescription() const;

private:
    /**
     * Implement z = Hx + noise
     * noise ~ Normal with covariance R
     * where x = [bbox_x, bbox_y, bbox_x_dot, bbox_y_dot]
     * and (bbox_x, bbox_y) represents the center of the bounding box, for the sake of simplicity.
     */
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;

    /**
     * Define input and measurement, i.e. output, descriptions.
     * - input: 4, 0, 2 means 4 linear components, 0 circular components and 2 noise components as the state is noisy
     * - output: 2 means 2 linear components
     *
     * @warning: noise components are actually only required if using sigma point-based filters and a generic non-linear measurement model is used
     */
    bfl::VectorDescription input_des_ = bfl::VectorDescription(4, 0, 2);
    bfl::VectorDescription output_des_ = bfl::VectorDescription(2);

    /* The actual measurement from the sensor. */
    Eigen::MatrixXd meas_;
};

#endif
