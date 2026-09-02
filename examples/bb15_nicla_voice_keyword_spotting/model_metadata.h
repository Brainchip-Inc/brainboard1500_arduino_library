#ifndef AKIDA_MODEL_METADATA_H_
#define AKIDA_MODEL_METADATA_H_

#include <cstddef>
#include <cstdint>

extern const char akida_model_path[];
extern const char akida_model_ip_version[];
extern const char akida_model_akida_version[];
extern const char akida_selected_sequence_name[];
extern const char akida_selected_sequence_backend[];
extern const char akida_input_transport[];
extern const char akida_first_layer_type[];
extern const bool akida_mapping_hw_only;
extern const bool akida_input_host_quantizer;
extern const int64_t akida_selected_sequence_index;
extern const int64_t akida_sequence_count;
extern const int64_t akida_layer_count;
extern const int64_t akida_program_length_bytes;
extern const int64_t akida_input_rank;
extern const int64_t akida_output_rank;
extern const uint32_t akida_input_shape[];
extern const uint32_t akida_output_shape[];
extern const int64_t akida_first_layer_input_bits;
extern const bool akida_first_layer_output_signed;
extern const int64_t akida_last_layer_input_bits;
extern const bool akida_last_layer_output_signed;

#endif  // AKIDA_MODEL_METADATA_H_
