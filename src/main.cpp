#include <cstdlib>
#include <chrono>

#include <module.h>

using namespace std::literals::chrono_literals;


int main(int argc, char** argv)
{

	const auto make_run = [](const double& psd, const double& noise) -> bool
	{
		/* Parameters. */
		double sample_time = 1.0 / 30.0;
		double power_spectral_density = psd;
		double noise_covariance = noise;
		std::size_t max_iterations = 30;

		/* Initialize module. */
		Module module(sample_time, max_iterations, power_spectral_density, noise_covariance);

		/* Start the filter. */
		if (!module.start())
		{
			std::cerr << "Cannot run the filter." << std::endl;

			return false;
		}

		return true;
	};

	/* Run 1. */
	std::cout << "Run 1" << std::endl;
	if (!make_run(1.0, 0.1))
		return EXIT_FAILURE;

	std::cout << std::endl;
	std::this_thread::sleep_for(3s);

	std::cout << "Run 2" << std::endl << std::endl;
	if (!make_run(0.001, 1.0))
		return EXIT_FAILURE;

	return EXIT_SUCCESS;
}
