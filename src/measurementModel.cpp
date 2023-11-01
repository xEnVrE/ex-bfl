#include <measurementModel.h>


MeasurementModel::MeasurementModel(const double& noise_covariance)
{
    /* The measurement matrix picks only the positional part out of the whole state. */
    H_.resize(2, 4);
    H_ << 1, 0, 0, 0,
          0, 1, 0, 0;

    R_.resize(2, 2);
    R_ << noise_covariance, 0,
          0,                noise_covariance;
}


std::pair<bool, Eigen::MatrixXd> MeasurementModel::getNoiseCovarianceMatrix() const
{
    return std::make_pair(true, R_);
}


Eigen::MatrixXd MeasurementModel::getMeasurementMatrix() const
{
    return H_;
}


bool MeasurementModel::freeze(const bfl::Data&)
{
    /**
     * Here is where data should be either:
     * - received from a sensor using their SDK, e.g. a RealSense;
     * - received from the network using a middleware, e.g. ROS 2;
     * - simulated;
     * - etc.
     */

    /* Let's assume we get a constant measurement in (10, 10). */
    meas_.resize(2, 1);
    meas_(0) = 10.0;
    meas_(1) = 10.0;

    return true;
}


std::pair<bool, bfl::Data> MeasurementModel::measure(const bfl::Data&) const
{
    /* Always return the last *frozen* measurement, in case multiple evaluations are required. */
    return std::make_pair(true, meas_);
}


bfl::VectorDescription MeasurementModel::getInputDescription() const
{
    return input_des_;
}


bfl::VectorDescription MeasurementModel::getMeasurementDescription() const
{
    return output_des_;
}
