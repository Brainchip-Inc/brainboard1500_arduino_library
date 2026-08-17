#include "model_metadata.h"

const char akida_model_path[] = "human_detection_model/runs/regular_frame96_gray/presence_regular_96_gray_akd1500.fbz";
const char akida_model_ip_version[] = "v1";
const char akida_model_akida_version[] = "2.5.0";
const char akida_selected_sequence_name[] = "HW/stem_conv-classifier";
const char akida_selected_sequence_backend[] = "Hardware";
const char akida_input_transport[] = "dense";
const char akida_first_layer_type[] = "InputConvolutional";
const bool akida_mapping_hw_only = false;
const bool akida_input_host_quantizer = true;
const int64_t akida_selected_sequence_index = 0;
const int64_t akida_sequence_count = 1;
const int64_t akida_layer_count = 6;
const int64_t akida_program_length_bytes = 50016;
const int64_t akida_input_rank = 3;
const int64_t akida_output_rank = 3;
const uint32_t akida_input_shape[] = {96, 96, 1};
const uint32_t akida_output_shape[] = {1, 1, 2};
const int64_t akida_first_layer_input_bits = 8;
const bool akida_first_layer_output_signed = false;
const int64_t akida_last_layer_input_bits = 4;
const bool akida_last_layer_output_signed = true;
