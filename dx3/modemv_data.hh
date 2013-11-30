#ifndef MODEMV_DATA_HH
#define MODEMV_DATA_HH

#include <array>
#include "bool_decoder.hh"

enum intra_mbmode { DC_PRED, V_PRED, H_PRED, TM_PRED, B_PRED };
const unsigned int num_uv_modes = B_PRED, num_y_modes = num_uv_modes + 1;

enum intra_bmode { B_DC_PRED, B_TM_PRED, B_VE_PRED, B_HE_PRED, B_LD_PRED,
		   B_RD_PRED, B_VR_PRED, B_VL_PRED, B_HD_PRED, B_HU_PRED };
const unsigned int num_intra_b_modes = B_HU_PRED + 1;

const extern ProbabilityArray< num_y_modes > kf_y_mode_probs;
const extern ProbabilityArray< num_uv_modes > kf_uv_mode_probs;

const extern std::array< std::array< ProbabilityArray< num_intra_b_modes >,
				     num_intra_b_modes>,
			 num_intra_b_modes > kf_b_mode_probs;

const extern TreeArray< num_y_modes > kf_y_mode_tree;

const extern TreeArray< num_y_modes > y_mode_tree;

const extern TreeArray< num_uv_modes > uv_mode_tree;

const extern TreeArray< num_intra_b_modes > b_mode_tree;

const extern TreeArray< 8 > small_mv_tree;

const extern TreeArray< 5 > mv_ref_tree;

const extern TreeArray< 4 > submv_ref_tree;

const extern TreeArray< 4 > split_mv_tree;

const extern ProbabilityArray< num_intra_b_modes > default_b_mode_probs;

const extern std::array< std::array< Probability, 4 >, 6 > mv_counts_to_probs;

const extern ProbabilityArray< 4 > split_mv_probs;

const extern std::array< std::array< Probability, 3 >, 5 > submv_ref_probs2;

const extern std::array< std::array< Probability, 16 >, 4 > mv_partitions;

const int num_segments = 4;

const extern TreeArray< num_segments > segment_id_tree;

#endif /* MODEMV_DATA_HH */
