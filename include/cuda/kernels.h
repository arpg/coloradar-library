#include <cuda_runtime.h>
#include <vector>
#include "cuComplex.h"


void applyWindow(int dist,
                 int stride,
                 int n_samples,
                 int n_batches,
                 double* window_func,
                 cuDoubleComplex* data);

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
                  cuDoubleComplex* dest_mat);

void removeCoupling(int num_range_bins,
                    int num_doppler_bins,
                    int num_antennas,
                    cuDoubleComplex* data,
                    cuDoubleComplex* coupling_signature);

void removeNegSpectrum(int num_range_bins_,
                       int num_pos_range_bins_,
                       int num_doppler_bins_,
                       int num_antennas,
                       cuDoubleComplex* data_in,
                       cuDoubleComplex* data_out);

void assembleMsg(int n_range_bins,
                 int n_doppler_bins,
                 int n_az_beams,
                 int n_el_beams,
                 cuDoubleComplex* data_in,
                 float* data_out);

void gaussianBlur(int n_range_bins,
                  int n_doppler_bins,
                  int n_antennas,
                  cuDoubleComplex* data);

void collapseDoppler(int n_range_bins,
                     int n_doppler_bins,
                     int n_angles,
                     float doppler_bin_width,
                     float* data_in,
                     float* data_out);

void setFrameData(int n_range_bins,
                  int n_doppler_bins,
                  int n_tx,
                  int n_rx,
                  int16_t* int_frame_data,
                  cuDoubleComplex* range_fft_data);

void applyPhaseFreqCal(int n_range_bins,
                       int n_doppler_bins,
                       int n_tx,
                       int n_rx,
                       cuDoubleComplex* range_fft_data,
                       cuDoubleComplex* freq_calib_mat,
                       cuDoubleComplex* phase_calib_mat);
