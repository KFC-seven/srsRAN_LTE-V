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

#include "srsran/phy/ch_estimation/chest_sl.h"
#include "srsran/phy/common/phy_common_sl.h"
#include "srsran/phy/dft/ofdm.h"
#include "srsran/phy/io/filesource.h"
#include "srsran/phy/phch/pscch.h"
#include "srsran/phy/phch/pssch.h"
#include "srsran/phy/phch/sci.h"
#include "srsran/phy/utils/debug.h"
#include "srsran/phy/utils/vector.h"
#include "srsran/phy/rf/rf.h"

#define MAX_SF_BUFFER_SIZE (15 * 2048)

static srsran_cell_sl_t cell = {.nof_prb = 6, .N_sl_id = 0, .tm = SRSRAN_SIDELINK_TM2, .cp = SRSRAN_CP_NORM};

static uint32_t        mcs_idx       = 4;
static uint32_t        prb_start_idx = 0;
static uint32_t        sf_n_samples  = 0;
static uint32_t        sf_n_re       = 0;
static cf_t*           sf_buffer     = NULL;
static srsran_ofdm_t   fft;
static srsran_radio_t  radio;
static srsran_pssch_t  pssch;
static srsran_pscch_t  pscch;
static srsran_sci_t    sci;
static srsran_chest_sl_t chest_sl;

static char*            input_file_name = NULL;
static srsran_filesource_t fsrc;
static bool             use_standard_lte_rates = false;
static uint32_t         file_offset            = 0;

// 添加帧间隔相关变量
static uint32_t sf_len = 0;
static uint32_t sf_period = 0;
static bool go_exit = false;

// 信号处理函数
static void sig_int_handler(int signo)
{
  printf("SIGINT received. Exiting...\n");
  if (signo == SIGINT) {
    go_exit = true;
  }
}

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
  if (srsran_ofdm_tx_init(&fft, cell.cp, sf_buffer, sf_buffer, cell.nof_prb)) {
    ERROR("Error creating FFT object");
    return SRSRAN_ERROR;
  }

  // Initialize radio
  if (srsran_radio_init(&radio, NULL)) {
    ERROR("Error initializing radio");
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
  srsran_ofdm_tx_free(&fft);
  srsran_radio_free(&radio);

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

  // 计算子帧长度和周期
  sf_len = srsran_symbol_sz(cell.nof_prb) * 15;  // 一个子帧的采样点数
  sf_period = sf_len * 1000 / 30720;  // 1ms子帧周期(30.72MHz采样率)

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

  // Generate SCI
  uint8_t sci_msg[SRSRAN_SCI_MAX_LEN];
  if (srsran_sci_format0_pack(&sci, sci_msg) != SRSRAN_SUCCESS) {
    ERROR("Error packing SCI");
    goto clean_exit;
  }

  // Generate random data for PSSCH
  uint8_t* data = srsran_vec_u8_malloc(pssch.sl_sch_tb_len);
  if (!data) {
    ERROR("Error allocating memory");
    goto clean_exit;
  }

  // 主循环
  while (!go_exit) {
    // 生成新的随机数据
    for (int i = 0; i < pssch.sl_sch_tb_len; i++) {
      data[i] = rand() % 256;
    }

    // Encode PSCCH
    if (srsran_pscch_encode(&pscch, sci_msg, sf_buffer, prb_start_idx) != SRSRAN_SUCCESS) {
      ERROR("Error encoding PSCCH");
      goto clean_exit;
    }

    // Encode PSSCH
    if (srsran_pssch_encode(&pssch, data, sf_buffer) != SRSRAN_SUCCESS) {
      ERROR("Error encoding PSSCH");
      goto clean_exit;
    }

    // Add DMRS
    if (srsran_chest_sl_put_dmrs(&chest_sl, sf_buffer) != SRSRAN_SUCCESS) {
      ERROR("Error adding DMRS");
      goto clean_exit;
    }

    // Convert to time domain
    srsran_ofdm_tx_sf(&fft);

    // Transmit
    if (srsran_radio_tx(&radio, sf_buffer, sf_n_samples, 0, 0, 0) != SRSRAN_SUCCESS) {
      ERROR("Error transmitting");
      goto clean_exit;
    }

    // 等待一个子帧周期
    usleep(sf_period);
  }

  ret = SRSRAN_SUCCESS;

clean_exit:
  base_free();
  if (data) {
    free(data);
  }
  return ret;
} 