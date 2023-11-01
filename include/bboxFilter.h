#ifndef BBOX_FILTER_H
#define BBOX_FILTER_H

#include <BayesFilters/GaussianFilter.h>
#include <BayesFilters/Gaussian.h>

#include <memory>


class BboxFilter : public bfl::GaussianFilter
{
public:
    BboxFilter(const double& sample_time, const std::size_t& max_iterations, std::unique_ptr<bfl::GaussianPrediction> prediction, std::unique_ptr<bfl::GaussianCorrection> correction);

    bool initialization_step() override;
    void filtering_step() override;
    bool run_condition() override;

private:
    /* Storage for predicted and corrected beliefs. */
    bfl::Gaussian pred_;
    bfl::Gaussian corr_;

    /* Sample time. */
    const double sample_time_;

    /* Max number of iterations. */
    std::size_t max_iterations_;
};

#endif /* BBOX_FILTER_H */
