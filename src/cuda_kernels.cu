#include "cuda_kernels.h"
#include <stdio.h>


__global__
void elementwiseMultiply(int dist,
                         int stride,
                         int n_samples,
                         int n_batches,
                         double* window_func,
                         cuDoubleComplex* data)
{
  int sample_idx = blockIdx.x * blockDim.x + threadIdx.x;
  int batch_idx = blockIdx.y * blockDim.y + threadIdx.y;

  if (sample_idx < n_samples && batch_idx < n_batches)
  {
    int data_idx = (batch_idx * dist) + (sample_idx * stride);
    data[data_idx] =
      cuCmul(make_cuDoubleComplex(window_func[sample_idx],0.0),
             data[data_idx]);
  }
}

__global__
void rearrangeMatrix(int n_range_bins,
                     int n_doppler_bins,
                     int n_tx,
                     int n_rx,
                     int n_az_beams,
                     int n_el_beams,
                     int n_virtual_elements,
                     int* virtual_array_map,
                     double* az_window,
                     double* el_window,
                     cuDoubleComplex* src_mat,
                     cuDoubleComplex* dest_mat)
{
  int range_idx = blockIdx.x * blockDim.x + threadIdx.x;
  int doppler_idx = blockIdx.y * blockDim.y + threadIdx.y;
  int virtual_idx = blockIdx.z * blockDim.z + threadIdx.z;

  if (range_idx < n_range_bins && doppler_idx < n_doppler_bins && virtual_idx < n_virtual_elements)
  {
    int az_idx = virtual_array_map[4*virtual_idx];
    int el_idx = virtual_array_map[(4*virtual_idx)+1];
    int rx_idx = virtual_array_map[(4*virtual_idx)+2];
    int tx_idx = virtual_array_map[(4*virtual_idx)+3];

    // truncate samples if num fft points is less than virtual array dims
    if (az_idx < n_az_beams && el_idx < n_el_beams)
    {
      int src_idx = range_idx + n_range_bins * (rx_idx + n_rx * (tx_idx + n_tx * doppler_idx));
      int dest_idx = el_idx + n_el_beams * (az_idx + n_az_beams * (range_idx + n_range_bins * doppler_idx));
      dest_mat[dest_idx] = src_mat[src_idx];
//       if (dest_idx < 100)
//         printf("rearrangeMatrix dest_idx: %d, src_idx: %d, value: (%f + i%f)\n", dest_idx, src_idx, cuCreal(dest_mat[dest_idx]), cuCimag(dest_mat[dest_idx]));
      /*
      dest_mat[dest_idx] = cuCmul(cuCmul(src_mat[src_idx],
                                  make_cuDoubleComplex(az_window[az_idx],0.0)),
                                  make_cuDoubleComplex(el_window[el_idx],0.0)); */
    }
  }

}
// mine
// rearrangeMatrix dest_idx: 0, src_idx: 0, value: (6067.363877 + i-2204.109097)
// rearrangeMatrix dest_idx: 32, src_idx: 128, value: (7138.991342 + i-6608.822645)
// rearrangeMatrix dest_idx: 64, src_idx: 256, value: (-18881.897466 + i-11790.785929)
// rearrangeMatrix dest_idx: 96, src_idx: 384, value: (-13489.109783 + i3531.592471)

// 2nd msg
// rearrangeMatrix dest_idx: 0, src_idx: 0, value: (6067.363877 + i-2204.109097)
// rearrangeMatrix dest_idx: 32, src_idx: 128, value: (1887.907818 + i-5370.057859)
// rearrangeMatrix dest_idx: 64, src_idx: 256, value: (-14937.764607 + i-1228.167782)
// rearrangeMatrix dest_idx: 96, src_idx: 384, value: (-4835.452407 + i4750.476440)

// first msg
// rearrangeMatrix dest_idx: 0, src_idx: 0, value: (-1471.871323 + i-32085.529897)
// rearrangeMatrix dest_idx: 32, src_idx: 128, value: (-9788.159382 + i-22641.801859)
// rearrangeMatrix dest_idx: 64, src_idx: 256, value: (-47238.116607 + i38964.421018)
// rearrangeMatrix dest_idx: 96, src_idx: 384, value: (6693.513993 + i29549.225240)



__global__
void removeAntennaCoupling(int num_range_bins,
                           int num_doppler_bins,
                           int num_antennas,
                           cuDoubleComplex* data,
                           cuDoubleComplex* coupling_signature)
{
  int antenna_idx = blockIdx.x * blockDim.x + threadIdx.x;
  int doppler_idx = blockIdx.y * blockDim.y + threadIdx.y;
  int range_idx = blockIdx.z * blockDim.z + threadIdx.z;

  if (antenna_idx < num_antennas
      && doppler_idx < num_doppler_bins
      && range_idx < num_range_bins)
  {
    int couple_idx = range_idx + num_range_bins * antenna_idx;
    int data_idx = range_idx + num_range_bins * antenna_idx + num_range_bins * num_antennas * doppler_idx;
    data[data_idx] = cuCsub(data[data_idx], coupling_signature[couple_idx]);
  }

}

__global__
void removeNegativeSpectrum(int n_range_bins,
                            int n_pos_range_bins,
                            int n_doppler_bins,
                            int n_antennas,
                            cuDoubleComplex* data_in,
                            cuDoubleComplex* data_out)
{
  int out_range_idx = blockIdx.x * blockDim.x + threadIdx.x;
  int ant_idx = blockIdx.y * blockDim.y + threadIdx.y;
  int doppler_idx = blockIdx.z * blockDim.z + threadIdx.z;

  if (out_range_idx < n_pos_range_bins
      && ant_idx < n_antennas
      && doppler_idx < n_doppler_bins)
  {
    int in_range_idx = out_range_idx + 1; // start at 1 to skip DC bin
    int in_idx = in_range_idx + n_range_bins
                  * (ant_idx + n_antennas * doppler_idx);
    int out_idx = out_range_idx + n_pos_range_bins
                  * (ant_idx + n_antennas * doppler_idx);
    data_out[out_idx] = data_in[in_idx];
  }
}

__global__
void assembleOutput(int n_range_bins,
                    int n_doppler_bins,
                    int n_az_beams,
                    int n_el_beams,
                    cuDoubleComplex* data_in,
                    float* data_out)
{
  int in_range_idx = blockIdx.x * blockDim.x + threadIdx.x;
  int in_doppler_idx = blockIdx.y * blockDim.y + threadIdx.y;
  int angle_idx = blockIdx.z * blockDim.z + threadIdx.z;

  if (in_range_idx < n_range_bins
      && in_doppler_idx < n_doppler_bins
      && angle_idx < n_az_beams * n_el_beams)
  {
    int out_range_idx = in_range_idx;
    int in_az_idx = int(angle_idx / n_el_beams);
    int in_el_idx = angle_idx - (in_az_idx * n_el_beams);
    int out_doppler_idx;
    int out_el_idx;
    int out_az_idx;

    // fftshift for doppler
    if (in_doppler_idx < n_doppler_bins / 2)
      out_doppler_idx = in_doppler_idx + (n_doppler_bins / 2);
    else
      out_doppler_idx = in_doppler_idx - (n_doppler_bins / 2);
    // fftshift for azimuth
    if (in_az_idx < n_az_beams / 2)
      out_az_idx = in_az_idx + (n_az_beams / 2);
    else
      out_az_idx = in_az_idx - (n_az_beams / 2);
    // fftshift for elevation
    if (in_el_idx < n_el_beams / 2)
      out_el_idx = in_el_idx + (n_el_beams / 2);
    else
      out_el_idx = in_el_idx - (n_el_beams / 2);

    int in_idx = in_el_idx + n_el_beams * (in_az_idx + n_az_beams
                  * (in_range_idx + n_range_bins * in_doppler_idx));
    /*
    int n_samples = n_range_bins * n_doppler_bins * n_az_beams * n_el_beams;
    if (in_idx >= n_samples)
      printf
    */
    int out_idx = out_range_idx + n_range_bins * (out_doppler_idx + n_doppler_bins
                  * (out_az_idx + n_az_beams * out_el_idx));

    double magnitude = sqrt(cuCreal(data_in[in_idx]) * cuCreal(data_in[in_idx])
                          + cuCimag(data_in[in_idx]) * cuCimag(data_in[in_idx]));

    data_out[out_idx] = (float)magnitude;
  }
}

__global__
void gaussianBlur1D(int n_range_bins,
                    int n_doppler_bins,
                    int n_antennas,
                    int direction,
                    cuDoubleComplex* data_in,
                    cuDoubleComplex* data_out)
{
  int range_idx = blockIdx.x * blockDim.x + threadIdx.x;
  int doppler_idx = blockIdx.y * blockDim.y + threadIdx.y;
  int antenna_idx = blockIdx.z * blockDim.z + threadIdx.z;

  if (range_idx < n_range_bins
      && doppler_idx < n_doppler_bins
      && antenna_idx < n_antennas)
  {
    double kernel[] = {1.0, 2.0, 1.0}; //{1.0, 4.0, 6.0, 4.0, 1.0};
    double kernel_sum = 4.0;
    int kernel_width = 3;
    int out_idx = range_idx + n_range_bins * doppler_idx
                  + n_range_bins * n_doppler_bins * antenna_idx;

    for (int k_idx = 0; k_idx < kernel_width; k_idx++)
    {
      int idx = 0;
      if (direction == 0)
      {
        int kernel_idx = range_idx - (kernel_width / 2) + k_idx;
        kernel_idx = max(min(kernel_idx,n_range_bins-1),0);
        idx = kernel_idx + n_range_bins * doppler_idx
                  + n_range_bins * n_doppler_bins * antenna_idx;
      }
      else
      {
        int kernel_idx = doppler_idx - (kernel_width / 2) + k_idx;
        kernel_idx = max(min(kernel_idx,n_doppler_bins-1),0);
        idx = range_idx + n_range_bins * kernel_idx
              + n_range_bins * n_doppler_bins * antenna_idx;
      }
      data_out[out_idx] = cuCadd(data_out[out_idx],cuCmul(data_in[idx], make_cuDoubleComplex(kernel[k_idx],0.0)));
    }
    data_out[out_idx] = cuCdiv(data_out[out_idx], make_cuDoubleComplex(kernel_sum,0.0));
  }
}
// 0 1 2 3 4 5 6 7 8 9
//       0 1 2 3 4
// 5
// 5/2 = 2
// 5 - 2 + 0 = 3
// 5 - 2 + 4 = 7

__global__
void getMaxDoppler(int n_range_bins,
                   int n_doppler_bins,
                   int n_angles,
                   float doppler_bin_width,
                   float* data_in,
                   float* data_out)
{
  int range_idx = blockIdx.x * blockDim.x + threadIdx.x;
  int angle_idx = blockIdx.y * blockDim.y + threadIdx.y;

  //printf("data out size: %f\n", float(sizeof(data_in)) / float(sizeof(data_out[0])));

  if (angle_idx < n_angles && range_idx < n_range_bins)
  {

    // get maximum doppler value in range bin
    float max_doppler = 0;
    for (int doppler_idx = 0; doppler_idx < n_doppler_bins; doppler_idx++)
    {
      int idx = range_idx + n_range_bins * doppler_idx
                + n_range_bins * n_doppler_bins * angle_idx;
      if (max_doppler < data_in[idx])
      {
        max_doppler = data_in[idx];
      }
    }
    // get softmax-weighted center of doppler activations
    float sum_weights = 0;
    float activation_center = 0;
    float temp = 1.0;
    for (int doppler_idx = 0; doppler_idx < n_doppler_bins; doppler_idx++)
    {
      int idx = range_idx + n_range_bins * doppler_idx
                + n_range_bins * n_doppler_bins * angle_idx;
      float softmax_weight = exp((data_in[idx] - max_doppler) / temp);
      activation_center += softmax_weight * doppler_idx;
      sum_weights += softmax_weight;
    }
    activation_center /= sum_weights;

    // interpolate intensity value at activation center
    int activation_idx = activation_center;
    int idx0 = range_idx + n_range_bins * activation_idx
                + n_range_bins * n_doppler_bins * angle_idx;
    int idx1 = 0;
    // if activation center is within the doppler indices
    if (activation_idx < n_doppler_bins - 1)
    {
      idx1 = range_idx + n_range_bins * (activation_idx + 1)
              + n_range_bins * n_doppler_bins * angle_idx;
    }
    else // wrap around to start of the doppler indices
    {
      idx1 = range_idx + n_range_bins * n_doppler_bins * angle_idx;
    }

    // interpolate sub-bin intensity value at activation center
    float r = activation_center - float(activation_idx);
    float intensity = (1.0 - r) * data_in[idx0] + r * data_in[idx1];

    int out_idx = 2 * (range_idx + n_range_bins * angle_idx);
    data_out[out_idx] = intensity / float(n_doppler_bins);
    data_out[out_idx+1] = (activation_center - float(n_doppler_bins / 2))
                          * doppler_bin_width;

  }
}

__global__
void setFrameDataKernel(int n_range_bins,
                        int n_doppler_bins,
                        int n_tx,
                        int n_rx,
                        int16_t* int_frame_data,
                        cuDoubleComplex* range_fft_data)
{
  int range_idx = blockIdx.x * blockDim.x + threadIdx.x;
  int doppler_idx = blockIdx.y * blockDim.y + threadIdx.y;
  int antenna_idx = blockIdx.z * blockDim.z + threadIdx.z;

  if (range_idx < n_range_bins
    && doppler_idx < n_doppler_bins
    && antenna_idx < (n_tx * n_rx))
  {
    int tx_idx = (int) (antenna_idx / n_rx);
    int rx_idx = antenna_idx - (tx_idx * n_rx);

    int in_idx = 2 * (range_idx + n_range_bins *
                     (doppler_idx + n_doppler_bins *
                     (rx_idx + n_rx * tx_idx)));

    int out_idx = range_idx + n_range_bins *
                  (rx_idx + n_rx *
                  (tx_idx + n_tx * doppler_idx));

    range_fft_data[out_idx] = make_cuDoubleComplex((double)int_frame_data[in_idx],
                                                    (double)int_frame_data[in_idx+1]);
  }
}

__global__
void applyPhaseFreqCalKernel(int n_range_bins,
                             int n_doppler_bins,
                             int n_tx,
                             int n_rx,
                             cuDoubleComplex* range_fft_data,
                             cuDoubleComplex* freq_calib_mat,
                             cuDoubleComplex* phase_calib_mat)
{
  int range_idx = blockIdx.x * blockDim.x + threadIdx.x;
  int doppler_idx = blockIdx.y * blockDim.y + threadIdx.y;
  int antenna_idx = blockIdx.z * blockDim.z + threadIdx.z;

  if (range_idx < n_range_bins
    && doppler_idx < n_doppler_bins
    && antenna_idx < (n_tx * n_rx))
  {
    int tx_idx = (int) (antenna_idx / n_rx);
    int rx_idx = antenna_idx - (tx_idx * n_rx);

    int phase_idx = rx_idx + n_rx * tx_idx;
    int freq_idx = range_idx + n_range_bins *
                   (rx_idx + n_rx * tx_idx);
    int data_idx = range_idx + n_range_bins *
                   (rx_idx + n_rx *
                   (tx_idx + n_tx * doppler_idx));

    range_fft_data[data_idx] = cuCmul(phase_calib_mat[phase_idx],
                                      range_fft_data[data_idx]);
    range_fft_data[data_idx] = cuCmul(freq_calib_mat[freq_idx],
                                      range_fft_data[data_idx]);

  }
}


void applyWindow(int dist,
                 int stride,
                 int n_samples,
                 int n_batches,
                 double* window_func,
                 cuDoubleComplex* data)
{
  dim3 threads_per_block(16,16);
  dim3 num_blocks(n_samples / threads_per_block.x,
                 n_batches / threads_per_block.y);
  elementwiseMultiply<<<num_blocks,threads_per_block>>>(dist,
                                                        stride,
                                                        n_samples,
                                                        n_batches,
                                                        window_func,
                                                        data);
  cudaDeviceSynchronize();
}

void rearrangeData(int num_range_bins,
                   int num_doppler_bins,
                   int num_tx,
                   int num_rx,
                   int num_az_beams,
                   int num_el_beams,
                   int num_virtual_elements,
                   int* virtual_array_map,
                   double* azimuth_window,
                   double* elevation_window,
                   cuDoubleComplex* src_mat,
                   cuDoubleComplex* dest_mat)
{
  dim3 threads_per_block(8,8,8);
  dim3 num_blocks(num_range_bins / threads_per_block.x,
                  num_doppler_bins / threads_per_block.y,
                  num_virtual_elements / threads_per_block.z);
  rearrangeMatrix<<<num_blocks, threads_per_block>>>(num_range_bins,
                                                     num_doppler_bins,
                                                     num_tx,
                                                     num_rx,
                                                     num_az_beams,
                                                     num_el_beams,
                                                     num_virtual_elements,
                                                     virtual_array_map,
                                                     azimuth_window,
                                                     elevation_window,
                                                     src_mat,
                                                     dest_mat);
    cudaError_t err = cudaDeviceSynchronize();
}


void removeCoupling(int num_range_bins,
                    int num_doppler_bins,
                    int num_antennas,
                    cuDoubleComplex* data,
                    cuDoubleComplex* coupling_signature)
{
  dim3 threads_per_block(4,16,4);
  dim3 num_blocks(num_antennas / threads_per_block.x,
                  num_doppler_bins / threads_per_block.y,
                  num_range_bins / threads_per_block.z);
  removeAntennaCoupling<<<num_blocks,threads_per_block>>>(num_range_bins,
                                                          num_doppler_bins,
                                                          num_antennas,
                                                          data,
                                                          coupling_signature);
  cudaDeviceSynchronize();
}

void removeNegSpectrum(int num_range_bins,
                       int num_pos_range_bins,
                       int num_doppler_bins,
                       int num_antennas,
                       cuDoubleComplex* data_in,
                       cuDoubleComplex* data_out)
{
  dim3 threads_per_block(8,8,8);
  dim3 num_blocks(num_pos_range_bins / threads_per_block.x,
                  num_antennas / threads_per_block.y,
                  num_doppler_bins / threads_per_block.z);
  removeNegativeSpectrum<<<num_blocks,threads_per_block>>>(num_range_bins,
                                                           num_pos_range_bins,
                                                           num_doppler_bins,
                                                           num_antennas,
                                                           data_in,
                                                           data_out);
  cudaDeviceSynchronize();
}

void assembleMsg(int n_range_bins,
                 int n_doppler_bins,
                 int n_az_beams,
                 int n_el_beams,
                 cuDoubleComplex* data_in,
                 float* data_out)
{
  dim3 threads_per_block(8,8,8);
  dim3 num_blocks(n_range_bins / threads_per_block.x,
                  n_doppler_bins / threads_per_block.y,
                  (n_az_beams * n_el_beams) / threads_per_block.z);
  assembleOutput<<<num_blocks,threads_per_block>>>(n_range_bins,
                                                   n_doppler_bins,
                                                   n_az_beams,
                                                   n_el_beams,
                                                   data_in,
                                                   data_out);
  cudaDeviceSynchronize();
}

void gaussianBlur(int n_range_bins,
                  int n_doppler_bins,
                  int n_antennas,
                  cuDoubleComplex* data)
{
  cuDoubleComplex* data_aux;
  cudaMalloc(&data_aux,
             sizeof(cuDoubleComplex)
             * n_range_bins
             * n_doppler_bins
             * n_antennas);
  cudaMemset(data_aux, 0,
             sizeof(cuDoubleComplex)
             * n_range_bins
             * n_doppler_bins
             * n_antennas);
  // blur in range
  dim3 threads_per_block(8,8,4);
  dim3 num_blocks(n_range_bins / threads_per_block.x,
                  n_doppler_bins / threads_per_block.y,
                  n_antennas / threads_per_block.z);
  gaussianBlur1D<<<num_blocks,threads_per_block>>>(n_range_bins,
                                                   n_doppler_bins,
                                                   n_antennas,
                                                   0,
                                                   data,
                                                   data_aux);


  cudaMemset(data, 0,
             sizeof(cuDoubleComplex)
             * n_range_bins
             * n_doppler_bins
             * n_antennas);

  // blur in doppler
  gaussianBlur1D<<<num_blocks,threads_per_block>>>(n_range_bins,
                                                   n_doppler_bins,
                                                   n_antennas,
                                                   1,
                                                   data_aux,
                                                   data);
  /*
  cudaMemcpy(data,
             data_aux,
             sizeof(float)
             * n_range_bins
             * n_doppler_bins
             * n_az_beams
             * n_el_beams,
             cudaMemcpyDefault);
  */
  cudaFree(data_aux);
  cudaDeviceSynchronize();
}

/*
          1
        1   1
      1   2   1
    1   3   3   1
  1   4   6   4   1
1   5  10   10  5   1
*/

void collapseDoppler(int n_range_bins,
                     int n_doppler_bins,
                     int n_angles,
                     float doppler_bin_width,
                     float* data_in,
                     float* data_out)
{

  dim3 threads(16,16);
  dim3 blocks(n_range_bins / threads.x,
              n_angles / threads.y);

  getMaxDoppler<<<blocks, threads>>>(n_range_bins,
                                      n_doppler_bins,
                                      n_angles,
                                      doppler_bin_width,
                                      data_in,
                                      data_out);
  cudaDeviceSynchronize();
}

void setFrameData(int n_range_bins,
                  int n_doppler_bins,
                  int n_tx,
                  int n_rx,
                  int16_t* int_frame_data,
                  cuDoubleComplex* range_fft_data)
{
  dim3 threads(8,8,8);
  dim3 blocks(n_range_bins / threads.x,
              n_doppler_bins / threads.y,
              (n_tx * n_rx) / threads.z);

  setFrameDataKernel<<<blocks, threads>>>(n_range_bins,
                                          n_doppler_bins,
                                          n_tx,
                                          n_rx,
                                          int_frame_data,
                                          range_fft_data);
  cudaDeviceSynchronize();
}

void applyPhaseFreqCal(int n_range_bins,
                       int n_doppler_bins,
                       int n_tx,
                       int n_rx,
                       cuDoubleComplex* range_fft_data,
                       cuDoubleComplex* freq_calib_mat,
                       cuDoubleComplex* phase_calib_mat)
{
  dim3 threads(8,8,8);
  dim3 blocks(n_range_bins / threads.x,
              n_doppler_bins / threads.y,
              (n_tx * n_rx) / threads.z);

  applyPhaseFreqCalKernel<<<blocks, threads>>>(n_range_bins,
                                               n_doppler_bins,
                                               n_tx,
                                               n_rx,
                                               range_fft_data,
                                               freq_calib_mat,
                                               phase_calib_mat);
  cudaDeviceSynchronize();
}