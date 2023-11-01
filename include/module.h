#ifndef MODULE_H
#define MODULE_H

#include <memory>

#include <bboxFilter.h>


class Module
{
public:
    Module(const double& sample_time, const std::size_t& number_iterations, const double& power_spectral_density, const double& noise_covariance);

    bool start();

private:
    std::unique_ptr<bfl::GaussianFilter> filter_;
};

#endif /* MODULE_H */
