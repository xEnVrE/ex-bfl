#include <motionModel.h>


MotionModel::MotionModel(const double& dt, const double& psd)
{
    /**
     *  Implement a constant velocity model.
     *  bbox_x+1 = bbox_x + bbox_x_dot * dt
     *  bbox_y+1 = bbox_y + bbox_y_dot * dt
     *  bbox_x_dot+1 = bbox_x_dot
     *  bbox_y_dot+1 = bbox_y_dot
     */
    F_.resize(4, 4);
    F_ << 1, 0, dt, 0,
          0, 1,  0, dt,
          0, 0,  1, 0,
          0, 0,  0, 1;

    /**
     * We use a discretized version of the continous time variant of the constant velocity model.
     *
     * Q = (dt ** 3 / 3 * psd, dt ** 2 / 2 * psd
     *      dt ** 2 / 2 * psd, dt * psd         )
     */
    Q_.resize(4, 4);
    Q_.topLeftCorner<2, 2>() = Eigen::Matrix2d::Identity() * std::pow(dt, 3) / 3;
    Q_.topRightCorner<2, 2>() = Eigen::Matrix2d::Identity() * std::pow(dt, 2) / 3;
    Q_.bottomLeftCorner<2, 2>() = Q_.topRightCorner<2, 2>();
    Q_.bottomRightCorner<2, 2>() = Eigen::Matrix2d::Identity() * dt;
    Q_ *= psd;
}


Eigen::MatrixXd MotionModel::getStateTransitionMatrix()
{
    return F_;
}


Eigen::MatrixXd MotionModel::getNoiseCovarianceMatrix()
{
    return Q_;
}


bfl::VectorDescription MotionModel::getStateDescription()
{
    return state_des_;
}


bool MotionModel::setProperty(const std::string& property)
{
    /* Does nothing in our case. */
    return true;
}
