#include "action_potential.h"
#include "network/serializer.h"
#include "file_io.h"
#include "math.h"


Real action_potential_value(Real t, const ActionPotentialParameters& params)
{
	const Real depolarization_slope_duration = 0.08;
	Real mixing_percentage = 1;

	if (t < params.depolarization_time-depolarization_slope_duration*(params.repolarization_time-params.depolarization_time))
	{
		mixing_percentage = 1;
	}
	else if (t < params.depolarization_time)
	{
		mixing_percentage = (params.depolarization_time-t)/(depolarization_slope_duration*(params.repolarization_time-params.depolarization_time));
	}
	else if (params.depolarization_time <= t && t <= params.repolarization_time)
	{
		mixing_percentage = (exp(4*(t-params.depolarization_time)/(params.repolarization_time-params.depolarization_time)-2) - exp(-2))/(-exp(-2)+exp(2))*0.75;
	}
	else if (t > params.repolarization_time)
	{
		mixing_percentage = 1-0.25*exp(-(t-params.repolarization_time)*10/(params.repolarization_time-params.depolarization_time));
	}

	return params.peak_potential - (params.peak_potential-params.resting_potential)*mixing_percentage;
}

/*
Real action_potential_value_2(Real t, const ActionPotentialParameters& params)
{
	const Real depolarization_slope_duration = 0.020;
	const Real repolarization_slope_duration = 0.050;
	Real mixing_percentage = 0;

	if (t < params.depolarization_time-depolarization_slope_duration)
	{
		mixing_percentage = 0;
	}
	else if (t < params.depolarization_time)
	{
		mixing_percentage = s_3rd_order_curve_transition((t-(params.depolarization_time-depolarization_slope_duration))/depolarization_slope_duration);
	}
	else if (params.depolarization_time <= t && t <= params.repolarization_time)
	{
		mixing_percentage = 1;
	}
	else if (t > params.repolarization_time && t <= params.repolarization_time+repolarization_slope_duration)
	{
		mixing_percentage = 1-s_3rd_order_curve_transition((t-params.repolarization_time)/repolarization_slope_duration);
	}
	else
	{
		mixing_percentage = 0;
	}

	return params.resting_potential + (params.peak_potential-params.resting_potential)*mixing_percentage;
}
*/

/*
Real action_potential_value_2(Real t, const ActionPotentialParameters& params)
{
	const Real depolarization_slope_duration = 0.020;
	const Real repolarization_slope_duration = 0.050;
	Real mixing_percentage = 0;

	if (t < params.depolarization_time)
	{
		mixing_percentage = 0;
	}
	else if (t < params.depolarization_time+depolarization_slope_duration)
	{
		mixing_percentage = s_3rd_order_curve_transition((t-params.depolarization_time)/depolarization_slope_duration);
	}
	else if (params.depolarization_time+depolarization_slope_duration <= t && t <= params.repolarization_time)
	{
		mixing_percentage = 1;
	}
	else if (t > params.repolarization_time && t <= params.repolarization_time+repolarization_slope_duration)
	{
		mixing_percentage = 1-s_3rd_order_curve_transition((t-params.repolarization_time)/repolarization_slope_duration);
	}
	else
	{
		mixing_percentage = 0;
	}

	return params.resting_potential + (params.peak_potential-params.resting_potential)*mixing_percentage;
}
*/

Real action_potential_value_2(Real t, const ActionPotentialParameters& params, Real depolarization_slope_duration, Real repolarization_slope_duration)
{
	Real mixing_percentage = 0;

	if (t < params.depolarization_time)
	{
		mixing_percentage = 0;
	}
	else if (t < params.depolarization_time+depolarization_slope_duration)
	{
		mixing_percentage = s_3rd_order_curve_transition((t-params.depolarization_time)/depolarization_slope_duration);
	}
	else if (params.depolarization_time+depolarization_slope_duration <= t && t <= params.repolarization_time)
	{
		mixing_percentage = 1;
	}
	else if (t > params.repolarization_time && t <= params.repolarization_time+repolarization_slope_duration)
	{
		mixing_percentage = 1-s_3rd_order_curve_transition((t-params.repolarization_time)/repolarization_slope_duration);
	}
	else
	{
		mixing_percentage = 0;
	}

	return params.resting_potential + (params.peak_potential-params.resting_potential)*mixing_percentage;
}


bool import_action_potential_parameters(const std::string& file_name, std::vector<ActionPotentialParameters>& params)
{
	size_t contents_size;
	uint8_t* contents = file_read(file_name.c_str(), &contents_size);
	if (!contents)
	{
		return false;
	}

	Deserializer des(contents, contents_size);

	// parse

	std::vector<ActionPotentialParameters> new_params;

	// vertex count
	uint32_t vertex_count = des.parse_u32();
	if (vertex_count != params.size())
	{
		printf("vertex count doesn't match\n");
		free(contents);
		return false;
	}
	new_params.reserve(vertex_count);

	for (uint32_t i = 0; i < vertex_count; i++)
	{
		ActionPotentialParameters param;
		param.resting_potential = des.parse_double();
		param.peak_potential = des.parse_double();
		param.depolarization_time = des.parse_double();
		param.repolarization_time = des.parse_double();
		new_params.push_back(param);
	}

	// set parameters
	params = new_params;

	free(contents);
	return true;
}

bool export_action_potential_parameters(const std::string& file_name, const std::vector<ActionPotentialParameters>& params)
{
	Serializer ser;

	// write

	// vertex count
	ser.push_u32(params.size());

	for (const ActionPotentialParameters& param : params)
	{
		ser.push_double(param.resting_potential);
		ser.push_double(param.peak_potential);
		ser.push_double(param.depolarization_time);
		ser.push_double(param.repolarization_time);
	}

	return file_write(file_name.c_str(), ser.get_data());
}

