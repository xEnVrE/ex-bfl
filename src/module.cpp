#include <module.h>

#include <BayesFilters/UKFPrediction.h>
#include <BayesFilters/UKFCorrection.h>

#include <measurementModel.h>
#include <motionModel.h>


Module::Module(const double& sample_time, const std::size_t& max_iterations, const double& power_spectral_density, const double& noise_covariance)
{
    /* Initialize motion model. */
    std::unique_ptr<bfl::AdditiveStateModel> motion_model = std::make_unique<MotionModel>(sample_time, power_spectral_density);

    /* Initialize measurement model .*/
    std::unique_ptr<bfl::AdditiveMeasurementModel> meas_model = std::make_unique<MeasurementModel>(noise_covariance);

    /* Unscented transform parameters. */
    double alpha = 1.0;
    double beta = 2.0;
    double kappa = 0.0;

    /* Initialize UKF prediction algorithm. */
    std::unique_ptr<bfl::UKFPrediction> prediction = std::make_unique<bfl::UKFPrediction>(std::move(motion_model), alpha, beta, kappa);

    /* Initialize UKF correction algorithm. */
    std::unique_ptr<bfl::UKFCorrection> correction = std::make_unique<bfl::UKFCorrection>(std::move(meas_model), alpha, beta, kappa);

    /* Initialize the filter. */
    filter_ = std::make_unique<BboxFilter>(sample_time, max_iterations, std::move(prediction), std::move(correction));
}


bool Module::start()
{
    /* Boot filter. */
    filter_->boot();

    /* Start internal thread - this is optional as one might call filter_->filteringStep() from another thread. */
    filter_->run();

    /* Join thread. */
    return filter_->wait();
}

