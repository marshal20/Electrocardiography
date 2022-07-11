#pragma once
#include "math.h"
#include <vector>


#define ACTION_POTENTIAL_RESTING_POTENTIAL -80e-3
#define ACTION_POTENTIAL_PEAK_POTENTIAL 15e-3
#define ACTION_POTENTIAL_DEPOLARIZATION_TIME 150e-3
#define ACTION_POTENTIAL_REPOLARIZATION_TIME 500e-3


struct ActionPotentialParameters
{
	Real resting_potential;
	Real peak_potential;
	Real depolarization_time;
	Real repolarization_time;
};

Real action_potential_value(Real t, const ActionPotentialParameters& params);
Real action_potential_value_2(Real t, const ActionPotentialParameters& params, Real depolarization_slope_duration = 0.020, Real repolarization_slope_duration = 0.050);
Real action_potential_value_with_hyperdepolarizaton(Real t, const ActionPotentialParameters& params, Real depolarization_slope_duration = 0.020, Real repolarization_slope_duration = 0.050, Real hyperdepolarization_percentage = 0.1, Real amplitude = 1);
Real action_potential_value_with_hyperdepolarizaton_new(Real t, const ActionPotentialParameters& params, Real depolarization_slope_duration = 0.020, Real repolarization_slope_duration = 0.050, Real hyperdepolarization_percentage = 0.1, Real amplitude = 1);

Real extracellular_potential(Real t, Real dt, const ActionPotentialParameters& params, Real depolarization_slope_duration = 0.020, Real repolarization_slope_duration = 0.050);
Real extracellular_potential_negative_t_wave(Real t, const ActionPotentialParameters& params, Real depolarization_slope_duration = 0.020, Real repolarization_slope_duration = 0.050);
Real extracellular_potential_positive_t_wave(Real t, const ActionPotentialParameters& params, Real depolarization_slope_duration = 0.020, Real repolarization_slope_duration = 0.050);
Real extracellular_potential_positive_t_wave_with_over_depolarization(Real t, const ActionPotentialParameters& params, Real depolarization_slope_duration = 0.020, Real repolarization_slope_duration = 0.050);


bool import_action_potential_parameters(const std::string& file_name, std::vector<ActionPotentialParameters>& params);
bool export_action_potential_parameters(const std::string& file_name, const std::vector<ActionPotentialParameters>& params);
