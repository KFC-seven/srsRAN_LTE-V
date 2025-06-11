/**
 * Copyright 2013-2023 Software Radio Systems Limited
 *
 * This file is part of srsRAN.
 *
 * srsRAN is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsRAN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <signal.h>
#include <inttypes.h>
#include <complex.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <errno.h>

#include "srsran/phy/ch_estimation/chest_sl.h"
#include "srsran/phy/common/phy_common_sl.h"
#include "srsran/phy/dft/ofdm.h"
#include "srsran/phy/io/filesink.h"
#include "srsran/phy/phch/pscch.h"
#include "srsran/phy/phch/pssch.h"
#include "srsran/phy/phch/sci.h"
#include "srsran/phy/utils/debug.h"
#include "srsran/phy/utils/vector.h"
#include "srsran/phy/rf/rf.h"

#define MAX_SF_BUFFER_SIZE (15 * 2048)

static srsran_cell_sl_t cell = {.nof_prb = 6, .N_sl_id = 0, .tm = SRSRAN_SIDELINK_TM2, .cp = SRSRAN_CP_NORM};
static srsran_sl_comm_resource_pool_t sl_comm_resource_pool = {
    .period_length = 40,
    .prb_num = 6,
    .prb_start = 0,
    .prb_end = 5
};

static uint32_t        mcs_idx       = 4;
static uint32_t        prb_start_idx = 0;
static uint32_t        sf_n_samples  = 0;
static uint32_t        sf_n_re       = 0;
static cf_t*           sf_buffer     = NULL;
static srsran_ofdm_t   fft;
static srsran_rf_t     rf;
static srsran_pssch_t  pssch;
static srsran_pscch_t  pscch;
static srsran_sci_t    sci;
static srsran_chest_sl_t chest_sl;

static char*            output_file_name = NULL;
static srsran_filesink_t fsink;
static bool             go_exit = false;

void usage(char* prog)
{
  printf("Usage: %s [emptv]\n", prog);
  printf("\t-p nof_prb [Default %d]\n", cell.nof_prb);
  printf("\t-m mcs_idx [Default %d]\n", mcs_idx);
  printf("\t-e extended CP [Default normal]\n");
  printf("\t-t Sidelink transmission mode {1,2,3,4} [Default %d]\n", (cell.tm + 1));
  printf("\t-v [set srsran_verbose to debug, default none]\n");
}

void parse_args(int argc, char** argv)
{
  int opt;
  while ((opt = getopt(argc, argv, "emptv")) != -1) {
    switch (opt) {
      case 'p':
        cell.nof_prb = (uint32_t)strtol(argv[optind], NULL, 10);
        break;
      case 'm':
        mcs_idx = (uint32_t)strtol(argv[optind], NULL, 10);
        break;
      case 'e':
        cell.cp = SRSRAN_CP_EXT;
        break;
      case 't':
        cell.tm = (srsran_sl_tm_t)(strtol(argv[optind], NULL, 10) - 1);
        break;
      case 'v':
        srsran_verbose++;
        break;
      default:
        usage(argv[0]);
        exit(-1);
    }
  }
}

int base_init()
{
  sf_n_samples = srsran_symbol_sz(cell.nof_prb) * 15;
  sf_n_re      = SRSRAN_SF_LEN_RE(cell.nof_prb, cell.cp);

  // Initialize PSSCH
  if (srsran_pssch_init(&pssch, &cell, &sl_comm_resource_pool) != SRSRAN_SUCCESS) {
    ERROR("Error initializing PSSCH");
    return SRSRAN_ERROR;
  }

  // Initialize PSCCH
  if (srsran_pscch_init(&pscch, SRSRAN_MAX_PRB) != SRSRAN_SUCCESS) {
    ERROR("Error in PSCCH init");
    return SRSRAN_ERROR;
  }

  if (srsran_pscch_set_cell(&pscch, cell) != SRSRAN_SUCCESS) {
    ERROR("Error in PSCCH set cell");
    return SRSRAN_ERROR;
  }

  // Initialize SCI
  if (srsran_sci_init(&sci, &cell, &sl_comm_resource_pool) < SRSRAN_SUCCESS) {
    ERROR("Error in SCI init");
    return SRSRAN_ERROR;
  }

  // Initialize channel estimator
  if (srsran_chest_sl_init(&chest_sl, SRSRAN_SIDELINK_PSSCH, cell, &sl_comm_resource_pool) != SRSRAN_SUCCESS) {
    ERROR("Error in PSSCH DMRS init");
    return SRSRAN_ERROR;
  }

  // Initialize FFT
  if (srsran_ofdm_rx_init(&fft, cell.cp, sf_buffer, sf_buffer, cell.nof_prb)) {
    ERROR("Error creating FFT object");
    return SRSRAN_ERROR;
  }

  // Initialize RF
  if (srsran_rf_open(&rf, "uhd")) {
    ERROR("Error opening RF device");
    return SRSRAN_ERROR;
  }

  // Allocate buffers
  sf_buffer = srsran_vec_cf_malloc(sf_n_re);
  if (!sf_buffer) {
    ERROR("Error allocating memory");
    return SRSRAN_ERROR;
  }
  srsran_vec_cf_zero(sf_buffer, sf_n_re);

  return SRSRAN_SUCCESS;
}

void base_free()
{
  srsran_pssch_free(&pssch);
  srsran_pscch_free(&pscch);
  srsran_sci_free(&sci);
  srsran_chest_sl_free(&chest_sl);
  srsran_ofdm_rx_free(&fft);
  srsran_rf_close(&rf);

  if (sf_buffer) {
    free(sf_buffer);
  }
}

int main(int argc, char** argv)
{
  int ret = SRSRAN_ERROR;

  parse_args(argc, argv);

  // 设置信号处理
  signal(SIGINT, sig_int_handler);

  if (base_init() != SRSRAN_SUCCESS) {
    ERROR("Error in base initialization");
    goto clean_exit;
  }

  // Configure PSSCH
  srsran_pssch_cfg_t pssch_cfg = {
      .mcs_idx = mcs_idx,
      .prb_start_idx = prb_start_idx,
      .nof_prb = cell.nof_prb,
      .N_x_id = cell.N_sl_id,
      .sf_idx = 0
  };

  if (srsran_pssch_set_cfg(&pssch, pssch_cfg) != SRSRAN_SUCCESS) {
    ERROR("Error configuring PSSCH");
    goto clean_exit;
  }

  // Configure channel estimator
  srsran_chest_sl_cfg_t chest_sl_cfg = {
      .N_x_id = cell.N_sl_id,
      .sf_idx = 0,
      .prb_start_idx = prb_start_idx,
      .nof_prb = cell.nof_prb
  };

  if (srsran_chest_sl_set_cfg(&chest_sl, chest_sl_cfg) != SRSRAN_SUCCESS) {
    ERROR("Error configuring channel estimator");
    goto clean_exit;
  }

  // 主循环
  while (!go_exit) {
    // Receive signal
    if (srsran_rf_recv(&rf, sf_buffer, sf_n_samples, true) != SRSRAN_SUCCESS) {
      ERROR("Error receiving");
      goto clean_exit;
    }

    // Convert to frequency domain
    srsran_ofdm_rx_sf(&fft);

    // Estimate channel
    cf_t* ce = srsran_vec_cf_malloc(sf_n_re);
    if (!ce) {
      ERROR("Error allocating memory");
      goto clean_exit;
    }

    if (srsran_chest_sl_ls_estimate(&chest_sl, sf_buffer, ce) != SRSRAN_SUCCESS) {
      ERROR("Error estimating channel");
      goto clean_exit;
    }

    // Decode PSCCH
    uint8_t sci_msg[SRSRAN_SCI_MAX_LEN];
    if (srsran_pscch_decode(&pscch, sf_buffer, ce, sci_msg, prb_start_idx) != SRSRAN_SUCCESS) {
      ERROR("Error decoding PSCCH");
      goto clean_exit;
    }

    // Unpack SCI
    if (srsran_sci_format0_unpack(&sci, sci_msg) != SRSRAN_SUCCESS) {
      ERROR("Error unpacking SCI");
      goto clean_exit;
    }

    // Decode PSSCH
    uint8_t* data = srsran_vec_u8_malloc(pssch.sl_sch_tb_len);
    if (!data) {
      ERROR("Error allocating memory");
      goto clean_exit;
    }

    if (srsran_pssch_decode(&pssch, sf_buffer, ce, data) != SRSRAN_SUCCESS) {
      ERROR("Error decoding PSSCH");
      goto clean_exit;
    }

    // Save DMRS to file
    if (output_file_name) {
      if (srsran_filesink_init(&fsink, output_file_name, SRSRAN_COMPLEX_FLOAT_BIN) != SRSRAN_SUCCESS) {
        ERROR("Error opening file");
        goto clean_exit;
      }

      // Extract DMRS symbols
      cf_t* dmrs = srsran_vec_cf_malloc(sf_n_re);
      if (!dmrs) {
        ERROR("Error allocating memory");
        goto clean_exit;
      }

      if (srsran_chest_sl_get_dmrs(&chest_sl, sf_buffer, dmrs) != SRSRAN_SUCCESS) {
        ERROR("Error extracting DMRS");
        goto clean_exit;
      }

      // Write DMRS to file
      srsran_filesink_write(&fsink, dmrs, sf_n_re);
      srsran_filesink_free(&fsink);
      free(dmrs);
    }

    free(ce);
    free(data);
  }

  ret = SRSRAN_SUCCESS;

clean_exit:
  base_free();
  return ret;
} 