/**
  ******************************************************************************
  * @file    network.c
  * @author  AST Embedded Analytics Research Platform
  * @date    Mon May 27 19:51:07 2019
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */



#include "network.h"

#include "ai_platform_interface.h"
#include "ai_math_helpers.h"

#include "core_common.h"
#include "layers.h"

#undef AI_TOOLS_VERSION_MAJOR
#undef AI_TOOLS_VERSION_MINOR
#undef AI_TOOLS_VERSION_MICRO
#define AI_TOOLS_VERSION_MAJOR 3
#define AI_TOOLS_VERSION_MINOR 3
#define AI_TOOLS_VERSION_MICRO 0

#undef AI_TOOLS_API_VERSION_MAJOR
#undef AI_TOOLS_API_VERSION_MINOR
#undef AI_TOOLS_API_VERSION_MICRO
#define AI_TOOLS_API_VERSION_MAJOR 1
#define AI_TOOLS_API_VERSION_MINOR 1
#define AI_TOOLS_API_VERSION_MICRO 0

#undef AI_NET_OBJ_INSTANCE
#define AI_NET_OBJ_INSTANCE g_network
 
#undef AI_NETWORK_MODEL_SIGNATURE
#define AI_NETWORK_MODEL_SIGNATURE     "f986d43058d90e31c0e661d48b8f5cf7"

#ifndef AI_TOOLS_REVISION_ID
#define AI_TOOLS_REVISION_ID     "(rev-)"
#endif

#undef AI_TOOLS_DATE_TIME
#define AI_TOOLS_DATE_TIME   "Mon May 27 19:51:07 2019"

#undef AI_TOOLS_COMPILE_TIME
#define AI_TOOLS_COMPILE_TIME    __DATE__ " " __TIME__

#undef AI_NETWORK_N_BATCHES
#define AI_NETWORK_N_BATCHES         (1)

/**  Forward network declaration section  *************************************/
AI_STATIC ai_network AI_NET_OBJ_INSTANCE;


/**  Forward network arrays declarations  *************************************/
AI_STATIC ai_array input_0_output_array;   /* Array #0 */
AI_STATIC ai_array conv2d_1_output_array;   /* Array #1 */
AI_STATIC ai_array separable_conv2d_1_output_array;   /* Array #2 */
AI_STATIC ai_array separable_conv2d_1_conv2d_output_array;   /* Array #3 */
AI_STATIC ai_array separable_conv2d_2_output_array;   /* Array #4 */
AI_STATIC ai_array separable_conv2d_2_conv2d_output_in_array;   /* Array #5 */
AI_STATIC ai_array separable_conv2d_2_conv2d_output_out_array;   /* Array #6 */
AI_STATIC ai_array dense_1_output_array;   /* Array #7 */
AI_STATIC ai_array dense_1_nl_output_array;   /* Array #8 */


/**  Forward network tensors declarations  ************************************/
AI_STATIC ai_tensor input_0_output;   /* Tensor #0 */
AI_STATIC ai_tensor conv2d_1_output;   /* Tensor #1 */
AI_STATIC ai_tensor separable_conv2d_1_output;   /* Tensor #2 */
AI_STATIC ai_tensor separable_conv2d_1_conv2d_output;   /* Tensor #3 */
AI_STATIC ai_tensor separable_conv2d_2_output;   /* Tensor #4 */
AI_STATIC ai_tensor separable_conv2d_2_conv2d_output_in;   /* Tensor #5 */
AI_STATIC ai_tensor separable_conv2d_2_conv2d_output_out;   /* Tensor #6 */
AI_STATIC ai_tensor dense_1_output;   /* Tensor #7 */
AI_STATIC ai_tensor dense_1_nl_output;   /* Tensor #8 */


/**  Forward network tensor chain declarations  *******************************/
AI_STATIC_CONST ai_tensor_chain conv2d_1_chain;   /* Chain #0 */
AI_STATIC_CONST ai_tensor_chain separable_conv2d_1_chain;   /* Chain #1 */
AI_STATIC_CONST ai_tensor_chain separable_conv2d_1_conv2d_chain;   /* Chain #2 */
AI_STATIC_CONST ai_tensor_chain separable_conv2d_2_chain;   /* Chain #3 */
AI_STATIC_CONST ai_tensor_chain separable_conv2d_2_conv2d_chain;   /* Chain #4 */
AI_STATIC_CONST ai_tensor_chain dense_1_chain;   /* Chain #5 */
AI_STATIC_CONST ai_tensor_chain dense_1_nl_chain;   /* Chain #6 */


/**  Subgraph network operators tensor chain declarations  *********************************/


/**  Subgraph network operators declarations  *********************************/


/**  Forward network layers declarations  *************************************/
AI_STATIC ai_layer_conv2d conv2d_1_layer; /* Layer #0 */
AI_STATIC ai_layer_conv2d separable_conv2d_1_layer; /* Layer #1 */
AI_STATIC ai_layer_conv2d_nl_pool separable_conv2d_1_conv2d_layer; /* Layer #2 */
AI_STATIC ai_layer_conv2d separable_conv2d_2_layer; /* Layer #3 */
AI_STATIC ai_layer_conv2d_nl_pool separable_conv2d_2_conv2d_layer; /* Layer #4 */
AI_STATIC ai_layer_dense dense_1_layer; /* Layer #5 */
AI_STATIC ai_layer_sm dense_1_nl_layer; /* Layer #6 */


/**  Arrays declarations section  *********************************************/
AI_ARRAY_OBJ_DECLARE(
  input_0_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 784,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  conv2d_1_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 10816,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  separable_conv2d_1_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 9216,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  separable_conv2d_1_conv2d_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 2304,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  separable_conv2d_2_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 1600,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  separable_conv2d_2_conv2d_output_in_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 400,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  separable_conv2d_2_conv2d_output_out_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 400,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  dense_1_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 47,
  AI_STATIC)
AI_ARRAY_OBJ_DECLARE(
  dense_1_nl_output_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 47,
  AI_STATIC)


/**  Activations tensors declaration section  *********************************/
AI_TENSOR_OBJ_DECLARE(
  input_0_output,
  AI_SHAPE_INIT(28, 28, 1, 1),
  AI_STRIDE_INIT(112, 4, 4, 4),
  &input_0_output_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  conv2d_1_output,
  AI_SHAPE_INIT(26, 26, 16, 1),
  AI_STRIDE_INIT(1664, 64, 4, 4),
  &conv2d_1_output_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  separable_conv2d_1_output,
  AI_SHAPE_INIT(24, 24, 16, 1),
  AI_STRIDE_INIT(1536, 64, 4, 4),
  &separable_conv2d_1_output_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  separable_conv2d_1_conv2d_output,
  AI_SHAPE_INIT(12, 12, 16, 1),
  AI_STRIDE_INIT(768, 64, 4, 4),
  &separable_conv2d_1_conv2d_output_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  separable_conv2d_2_output,
  AI_SHAPE_INIT(10, 10, 16, 1),
  AI_STRIDE_INIT(640, 64, 4, 4),
  &separable_conv2d_2_output_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  separable_conv2d_2_conv2d_output_in,
  AI_SHAPE_INIT(5, 5, 16, 1),
  AI_STRIDE_INIT(320, 64, 4, 4),
  &separable_conv2d_2_conv2d_output_in_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  separable_conv2d_2_conv2d_output_out,
  AI_SHAPE_INIT(1, 1, 400, 1),
  AI_STRIDE_INIT(1600, 1600, 4, 4),
  &separable_conv2d_2_conv2d_output_out_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  dense_1_output,
  AI_SHAPE_INIT(1, 1, 47, 1),
  AI_STRIDE_INIT(188, 188, 4, 4),
  &dense_1_output_array,
  AI_STATIC)
AI_TENSOR_OBJ_DECLARE(
  dense_1_nl_output,
  AI_SHAPE_INIT(1, 1, 47, 1),
  AI_STRIDE_INIT(188, 188, 4, 4),
  &dense_1_nl_output_array,
  AI_STATIC)





/* Layer #0: "conv2d_1" (Conv2D) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_1_weights_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 144,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  conv2d_1_weights,
  AI_SHAPE_INIT(16, 3, 3, 1),
  AI_STRIDE_INIT(36, 12, 4, 4),
  &conv2d_1_weights_array,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  conv2d_1_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 16,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  conv2d_1_bias,
  AI_SHAPE_INIT(1, 1, 16, 1),
  AI_STRIDE_INIT(64, 64, 4, 4),
  &conv2d_1_bias_array,
  AI_STATIC)


AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv2d_1_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&input_0_output),
  AI_TENSOR_LIST_ENTRY(&conv2d_1_output),
  AI_TENSOR_LIST_ENTRY(&conv2d_1_weights, &conv2d_1_bias),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv2d_1_layer, 0,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &separable_conv2d_1_layer, AI_STATIC,
  .tensors = &conv2d_1_chain, 
  .groups = 1, 
  .nl_func = nl_func_relu_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_2D_INIT(0, 0), 
)

/* Layer #1: "separable_conv2d_1" (Conv2D) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  separable_conv2d_1_weights_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 144,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  separable_conv2d_1_weights,
  AI_SHAPE_INIT(16, 3, 3, 1),
  AI_STRIDE_INIT(36, 12, 4, 4),
  &separable_conv2d_1_weights_array,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  separable_conv2d_1_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 16,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  separable_conv2d_1_bias,
  AI_SHAPE_INIT(1, 1, 16, 1),
  AI_STRIDE_INIT(64, 64, 4, 4),
  &separable_conv2d_1_bias_array,
  AI_STATIC)


AI_TENSOR_CHAIN_OBJ_DECLARE(
  separable_conv2d_1_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&conv2d_1_output),
  AI_TENSOR_LIST_ENTRY(&separable_conv2d_1_output),
  AI_TENSOR_LIST_ENTRY(&separable_conv2d_1_weights, &separable_conv2d_1_bias),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  separable_conv2d_1_layer, 1,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &separable_conv2d_1_conv2d_layer, AI_STATIC,
  .tensors = &separable_conv2d_1_chain, 
  .groups = 16, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_2D_INIT(0, 0), 
)

/* Layer #2: "separable_conv2d_1_conv2d" (Conv2D) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  separable_conv2d_1_conv2d_weights_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 256,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  separable_conv2d_1_conv2d_weights,
  AI_SHAPE_INIT(16, 1, 1, 16),
  AI_STRIDE_INIT(64, 64, 64, 4),
  &separable_conv2d_1_conv2d_weights_array,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  separable_conv2d_1_conv2d_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 16,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  separable_conv2d_1_conv2d_bias,
  AI_SHAPE_INIT(1, 1, 16, 1),
  AI_STRIDE_INIT(64, 64, 4, 4),
  &separable_conv2d_1_conv2d_bias_array,
  AI_STATIC)


AI_TENSOR_CHAIN_OBJ_DECLARE(
  separable_conv2d_1_conv2d_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&separable_conv2d_1_output),
  AI_TENSOR_LIST_ENTRY(&separable_conv2d_1_conv2d_output),
  AI_TENSOR_LIST_ENTRY(&separable_conv2d_1_conv2d_weights, &separable_conv2d_1_conv2d_bias),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  separable_conv2d_1_conv2d_layer, 1,
  OPTIMIZED_CONV2D_TYPE,
  conv2d_nl_pool, forward_conv2d_nl_pool,
  &AI_NET_OBJ_INSTANCE, &separable_conv2d_2_layer, AI_STATIC,
  .tensors = &separable_conv2d_1_conv2d_chain, 
  .groups = 1, 
  .nl_func = nl_func_relu_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_2D_INIT(0, 0), 
  .filter_pad_r = AI_SHAPE_2D_INIT(0, 0), 
  .pool_size = AI_SHAPE_2D_INIT(2, 2), 
  .pool_stride = AI_SHAPE_2D_INIT(2, 2), 
  .pool_pad = AI_SHAPE_2D_INIT(0, 0), 
  .pool_func = pool_func_mp_array_f32, 
)

/* Layer #3: "separable_conv2d_2" (Conv2D) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  separable_conv2d_2_weights_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 144,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  separable_conv2d_2_weights,
  AI_SHAPE_INIT(16, 3, 3, 1),
  AI_STRIDE_INIT(36, 12, 4, 4),
  &separable_conv2d_2_weights_array,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  separable_conv2d_2_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 16,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  separable_conv2d_2_bias,
  AI_SHAPE_INIT(1, 1, 16, 1),
  AI_STRIDE_INIT(64, 64, 4, 4),
  &separable_conv2d_2_bias_array,
  AI_STATIC)


AI_TENSOR_CHAIN_OBJ_DECLARE(
  separable_conv2d_2_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&separable_conv2d_1_conv2d_output),
  AI_TENSOR_LIST_ENTRY(&separable_conv2d_2_output),
  AI_TENSOR_LIST_ENTRY(&separable_conv2d_2_weights, &separable_conv2d_2_bias),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  separable_conv2d_2_layer, 4,
  CONV2D_TYPE,
  conv2d, forward_conv2d,
  &AI_NET_OBJ_INSTANCE, &separable_conv2d_2_conv2d_layer, AI_STATIC,
  .tensors = &separable_conv2d_2_chain, 
  .groups = 16, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_2D_INIT(0, 0), 
)

/* Layer #4: "separable_conv2d_2_conv2d" (Conv2D) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  separable_conv2d_2_conv2d_weights_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 256,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  separable_conv2d_2_conv2d_weights,
  AI_SHAPE_INIT(16, 1, 1, 16),
  AI_STRIDE_INIT(64, 64, 64, 4),
  &separable_conv2d_2_conv2d_weights_array,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  separable_conv2d_2_conv2d_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 16,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  separable_conv2d_2_conv2d_bias,
  AI_SHAPE_INIT(1, 1, 16, 1),
  AI_STRIDE_INIT(64, 64, 4, 4),
  &separable_conv2d_2_conv2d_bias_array,
  AI_STATIC)


AI_TENSOR_CHAIN_OBJ_DECLARE(
  separable_conv2d_2_conv2d_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&separable_conv2d_2_output),
  AI_TENSOR_LIST_ENTRY(&separable_conv2d_2_conv2d_output_in),
  AI_TENSOR_LIST_ENTRY(&separable_conv2d_2_conv2d_weights, &separable_conv2d_2_conv2d_bias),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  separable_conv2d_2_conv2d_layer, 4,
  OPTIMIZED_CONV2D_TYPE,
  conv2d_nl_pool, forward_conv2d_nl_pool,
  &AI_NET_OBJ_INSTANCE, &dense_1_layer, AI_STATIC,
  .tensors = &separable_conv2d_2_conv2d_chain, 
  .groups = 1, 
  .nl_func = nl_func_relu_array_f32, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_2D_INIT(0, 0), 
  .filter_pad_r = AI_SHAPE_2D_INIT(0, 0), 
  .pool_size = AI_SHAPE_2D_INIT(2, 2), 
  .pool_stride = AI_SHAPE_2D_INIT(2, 2), 
  .pool_pad = AI_SHAPE_2D_INIT(0, 0), 
  .pool_func = pool_func_mp_array_f32, 
)

/* Layer #5: "dense_1" (Dense) */
  

/* Weight tensor #1 */
AI_ARRAY_OBJ_DECLARE(
  dense_1_weights_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 18800,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  dense_1_weights,
  AI_SHAPE_INIT(1, 1, 47, 400),
  AI_STRIDE_INIT(75200, 75200, 1600, 4),
  &dense_1_weights_array,
  AI_STATIC)

/* Weight tensor #2 */
AI_ARRAY_OBJ_DECLARE(
  dense_1_bias_array, AI_DATA_FORMAT_FLOAT, 
  NULL, NULL, 47,
  AI_STATIC)

AI_TENSOR_OBJ_DECLARE(
  dense_1_bias,
  AI_SHAPE_INIT(1, 1, 47, 1),
  AI_STRIDE_INIT(188, 188, 4, 4),
  &dense_1_bias_array,
  AI_STATIC)


AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_1_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&separable_conv2d_2_conv2d_output_out),
  AI_TENSOR_LIST_ENTRY(&dense_1_output),
  AI_TENSOR_LIST_ENTRY(&dense_1_weights, &dense_1_bias),
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_1_layer, 8,
  DENSE_TYPE,
  dense, forward_dense,
  &AI_NET_OBJ_INSTANCE, &dense_1_nl_layer, AI_STATIC,
  .tensors = &dense_1_chain, 
)

/* Layer #6: "dense_1_nl" (Nonlinearity) */
  


AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_1_nl_chain, AI_STATIC_CONST, 
  AI_TENSOR_LIST_ENTRY(&dense_1_output),
  AI_TENSOR_LIST_ENTRY(&dense_1_nl_output),
  AI_TENSOR_LIST_EMPTY,
  AI_TENSOR_LIST_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_1_nl_layer, 8,
  SM_TYPE,
  sm, forward_sm,
  &AI_NET_OBJ_INSTANCE, &dense_1_nl_layer, AI_STATIC,
  .tensors = &dense_1_nl_chain, 
)


AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE,
  AI_BUFFER_OBJ_INIT(AI_BUFFER_FORMAT_U8,
                     1, 1, 79484, 1,
                     NULL),
  AI_BUFFER_OBJ_INIT(AI_BUFFER_FORMAT_U8,
                     1, 1, 80132, 1,
                     NULL),
  &input_0_output, &dense_1_nl_output,
  &conv2d_1_layer, 0)


AI_DECLARE_STATIC
ai_bool network_configure_activations(
  ai_network* net_ctx, const ai_buffer* activation_buffer)
{
  AI_ASSERT(net_ctx &&  activation_buffer && activation_buffer->data)

  ai_ptr activations = AI_PTR(AI_PTR_ALIGN(activation_buffer->data, 4));
  AI_ASSERT( activations )
  AI_FLAG_SET(net_ctx->flags, AI_NETWORK_FLAG_OUT_COPY);

  {
    /* Updating activations (byte) offsets */
    input_0_output_array.data = NULL;
  input_0_output_array.data_start = NULL;
  conv2d_1_output_array.data = activations + 0;
  conv2d_1_output_array.data_start = activations + 0;
  separable_conv2d_1_output_array.data = activations + 43264;
  separable_conv2d_1_output_array.data_start = activations + 43264;
  separable_conv2d_1_conv2d_output_array.data = activations + 0;
  separable_conv2d_1_conv2d_output_array.data_start = activations + 0;
  separable_conv2d_2_output_array.data = activations + 9280;
  separable_conv2d_2_output_array.data_start = activations + 9280;
  separable_conv2d_2_conv2d_output_in_array.data = activations + 0;
  separable_conv2d_2_conv2d_output_in_array.data_start = activations + 0;
  separable_conv2d_2_conv2d_output_out_array.data = activations + 0;
  separable_conv2d_2_conv2d_output_out_array.data_start = activations + 0;
  dense_1_output_array.data = activations + 3200;
  dense_1_output_array.data_start = activations + 3200;
  dense_1_nl_output_array.data = activations + 3200;
  dense_1_nl_output_array.data_start = activations + 3200;
  
  }
  return true;
}

AI_DECLARE_STATIC
ai_bool network_configure_weights(
  ai_network* net_ctx, const ai_buffer* weights_buffer)
{
  AI_ASSERT(net_ctx &&  weights_buffer && weights_buffer->data)

  ai_ptr weights = AI_PTR(weights_buffer->data);
  AI_ASSERT( weights )

  {
    /* Updating weights (byte) offsets */
    conv2d_1_weights_array.format |= AI_FMT_FLAG_CONST;
  conv2d_1_weights_array.data = weights + 0;
  conv2d_1_weights_array.data_start = weights + 0;
  conv2d_1_bias_array.format |= AI_FMT_FLAG_CONST;
  conv2d_1_bias_array.data = weights + 576;
  conv2d_1_bias_array.data_start = weights + 576;
  separable_conv2d_1_weights_array.format |= AI_FMT_FLAG_CONST;
  separable_conv2d_1_weights_array.data = weights + 640;
  separable_conv2d_1_weights_array.data_start = weights + 640;
  separable_conv2d_1_bias_array.format |= AI_FMT_FLAG_CONST;
  separable_conv2d_1_bias_array.data = weights + 1216;
  separable_conv2d_1_bias_array.data_start = weights + 1216;
  separable_conv2d_1_conv2d_weights_array.format |= AI_FMT_FLAG_CONST;
  separable_conv2d_1_conv2d_weights_array.data = weights + 1280;
  separable_conv2d_1_conv2d_weights_array.data_start = weights + 1280;
  separable_conv2d_1_conv2d_bias_array.format |= AI_FMT_FLAG_CONST;
  separable_conv2d_1_conv2d_bias_array.data = weights + 2304;
  separable_conv2d_1_conv2d_bias_array.data_start = weights + 2304;
  separable_conv2d_2_weights_array.format |= AI_FMT_FLAG_CONST;
  separable_conv2d_2_weights_array.data = weights + 2368;
  separable_conv2d_2_weights_array.data_start = weights + 2368;
  separable_conv2d_2_bias_array.format |= AI_FMT_FLAG_CONST;
  separable_conv2d_2_bias_array.data = weights + 2944;
  separable_conv2d_2_bias_array.data_start = weights + 2944;
  separable_conv2d_2_conv2d_weights_array.format |= AI_FMT_FLAG_CONST;
  separable_conv2d_2_conv2d_weights_array.data = weights + 3008;
  separable_conv2d_2_conv2d_weights_array.data_start = weights + 3008;
  separable_conv2d_2_conv2d_bias_array.format |= AI_FMT_FLAG_CONST;
  separable_conv2d_2_conv2d_bias_array.data = weights + 4032;
  separable_conv2d_2_conv2d_bias_array.data_start = weights + 4032;
  dense_1_weights_array.format |= AI_FMT_FLAG_CONST;
  dense_1_weights_array.data = weights + 4096;
  dense_1_weights_array.data_start = weights + 4096;
  dense_1_bias_array.format |= AI_FMT_FLAG_CONST;
  dense_1_bias_array.data = weights + 79296;
  dense_1_bias_array.data_start = weights + 79296;
  
  }

  return true;
}

/**  PUBLIC APIs SECTION  *****************************************************/

AI_API_ENTRY
ai_bool ai_network_get_info(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if ( report && net_ctx )
  {
    ai_network_report r = {
      .model_name        = AI_NETWORK_MODEL_NAME,
      .model_signature   = AI_NETWORK_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = {AI_TOOLS_API_VERSION_MAJOR, AI_TOOLS_API_VERSION_MINOR,
                            AI_TOOLS_API_VERSION_MICRO, 0x0},

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 328485,
      .n_inputs          = AI_NETWORK_IN_NUM,
      .inputs            = AI_BUFFER_OBJ_INIT(
                              AI_BUFFER_FORMAT_FLOAT,
                              28,
                              28,
                              1,
                              1, NULL),
      .n_outputs         = AI_NETWORK_OUT_NUM,
      .outputs           = AI_BUFFER_OBJ_INIT(
                              AI_BUFFER_FORMAT_FLOAT,
                              1,
                              1,
                              47,
                              1, NULL),
      .activations       = net_ctx->activations,
      .weights           = net_ctx->params,
      .n_nodes           = 0,
      .signature         = net_ctx->signature,
    };

    AI_FOR_EACH_NODE_DO(node, net_ctx->input_node)
    {
      r.n_nodes++;
    }

    *report = r;

    return ( r.n_nodes>0 ) ? true : false;
  }
  
  return false;
}

AI_API_ENTRY
ai_error ai_network_get_error(ai_handle network)
{
  return ai_platform_network_get_error(network);
}

AI_API_ENTRY
ai_error ai_network_create(
  ai_handle* network, const ai_buffer* network_config)
{
  return ai_platform_network_create(
    network, network_config, 
    &AI_NET_OBJ_INSTANCE,
    AI_TOOLS_API_VERSION_MAJOR, AI_TOOLS_API_VERSION_MINOR, AI_TOOLS_API_VERSION_MICRO);
}

AI_API_ENTRY
ai_handle ai_network_destroy(ai_handle network)
{
  return ai_platform_network_destroy(network);
}

AI_API_ENTRY
ai_bool ai_network_init(
  ai_handle network, const ai_network_params* params)
{
  ai_network* net_ctx = ai_platform_network_init(network, params);
  if ( !net_ctx ) return false;

  ai_bool ok = true;
  ok &= network_configure_weights(net_ctx, &params->params);
  ok &= network_configure_activations(net_ctx, &params->activations);
  
  return ok;
}


AI_API_ENTRY
ai_i32 ai_network_run(
  ai_handle network, const ai_buffer* input, ai_buffer* output)
{
  return ai_platform_network_process(network, input, output);
}

AI_API_ENTRY
ai_i32 ai_network_forward(ai_handle network, const ai_buffer* input)
{
  return ai_platform_network_process(network, input, NULL);
}

#undef AI_NETWORK_MODEL_SIGNATURE
#undef AI_NET_OBJ_INSTANCE
#undef AI_TOOLS_VERSION_MAJOR
#undef AI_TOOLS_VERSION_MINOR
#undef AI_TOOLS_VERSION_MICRO
#undef AI_TOOLS_API_VERSION_MAJOR
#undef AI_TOOLS_API_VERSION_MINOR
#undef AI_TOOLS_API_VERSION_MICRO
#undef AI_TOOLS_DATE_TIME
#undef AI_TOOLS_COMPILE_TIME

