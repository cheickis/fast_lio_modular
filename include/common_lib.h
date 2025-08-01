// common_lib.h - Temporary for backward compatibility
#pragma once

#warning "common_lib.h is deprecated. Please use fast_lio/common/Common.hpp instead"

#include "fast_lio/common/Common.hpp"

// Bring everything into global scope for backward compatibility
using namespace fast_lio;
using namespace fast_lio::types;
using namespace fast_lio::constants;

// Macros that were in the original file
#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
#define ARRAY_FROM_EIGEN(mat) mat.data(), mat.data() + mat.rows() * mat.cols()
#define STD_VEC_FROM_EIGEN(mat) std::vector<decltype(mat)::Scalar>(mat.data(), mat.data() + mat.rows() * mat.cols())
#define DEBUG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "Log/" + name))