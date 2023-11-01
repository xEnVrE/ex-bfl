#include <bboxFilter.h>


BboxFilter::BboxFilter(const double& sample_time, const std::size_t& max_iterations, std::unique_ptr<bfl::GaussianPrediction> prediction, std::unique_ptr<bfl::GaussianCorrection> correction) :
    bfl::GaussianFilter(std::move(prediction), std::move(correction)), sample_time_(sample_time), max_iterations_(max_iterations)
{}


bool BboxFilter::initialization_step()
{
    /* We need to initialize the corrected belif, at least, as it is the input to the very first prediction step. */
    Eigen::VectorXd mean_0 = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd covariance_0 = Eigen::MatrixXd::Identity(4, 4) * 0.1;

    corr_ = bfl::Gaussian(4);
    corr_.mean() = mean_0;
    corr_.covariance() = covariance_0;

    pred_ = bfl::Gaussian(4);

    return true;
}


void BboxFilter::filtering_step()
{
    std::cout << "Current state: " << corr_.mean().transpose() << std::endl;

    /* Prediction step. */
    prediction().predict(corr_, pred_);

    /* Freeze measurements. */
    correction().freeze_measurements();

    /* Correction step. */
    correction().correct(pred_, corr_);

    std::this_thread::sleep_for(std::chrono::milliseconds(int(sample_time_ * 1000)));
}


bool BboxFilter::run_condition()
{
    if (step_number() > max_iterations_)
        return false;

    return true;
}


