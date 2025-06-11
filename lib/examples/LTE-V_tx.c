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

#include <assert.h>
#include <math.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdarg.h>

#include "srsran/phy/common/phy_common.h"
#include "srsran/phy/phch/pscch.h"
#include "srsran/phy/phch/pssch.h"
#include "srsran/phy/rf/rf.h"
#include "srsran/srsran.h"

// 全局变量
volatile bool go_exit = false;

// 程序参数结构体
typedef struct {
  // RF设备配置
  char* rf_dev;
  char* rf_args;
  float rf_freq;
  float rf_gain;
  uint32_t nof_tx_antennas;

  // LTE-V基本参数
  uint32_t nof_prb;
  uint32_t nof_ports;
  uint32_t cell_id;
  uint32_t rnti;
  uint32_t mcs;
  uint32_t rv;

  // LTE-V高级参数
  uint32_t size_sub_channel;
  uint32_t num_sub_channel;
  uint32_t tdd_config;
  uint32_t tdd_special_sf;
  bool enable_256qam;

  // 高级配置
  bool use_standard_lte_rates;
  int nof_subframes;
  
  // 文件配置
  char* log_file;
} prog_args_t;

// 默认参数设置
static void args_default(prog_args_t* args)
{
  args->rf_dev = "uhd";
  args->rf_args = "type=b205i";
  args->rf_freq = 5900000000; // 5.9GHz for V2X
  args->rf_gain = 20;
  args->nof_tx_antennas = 1;

  args->nof_prb = 25;
  args->nof_ports = 1;
  args->cell_id = 1;
  args->rnti = 0x1234;
  args->mcs = 10;
  args->rv = 0;

  args->size_sub_channel = 10;
  args->num_sub_channel = 1;
  args->tdd_config = 0;
  args->tdd_special_sf = 0;
  args->enable_256qam = false;

  args->use_standard_lte_rates = false;
  args->nof_subframes = -1;
  
  args->log_file = "ltev_tx.log";
}

// 信号处理函数
static void sig_int_handler(int signo)
{
  printf("SIGINT received. Exiting...\n");
  if (signo == SIGINT) {
    go_exit = true;
  }
}

// 使用说明
static void usage(char* prog)
{
  printf("Usage: %s [选项]\n", prog);
  printf("选项:\n");
  printf("  -I <设备名称>    覆盖配置文件中的device_name\n");
  printf("  -a <设备参数>    覆盖配置文件中的device_args\n");
  printf("  -f <频率>        覆盖配置文件中的tx_freq (Hz)\n");
  printf("  -g <增益>        覆盖配置文件中的tx_gain (dB)\n");
  printf("  -p <PRB数>       覆盖配置文件中的nof_prb\n");
  printf("  -c <小区ID>      覆盖配置文件中的cell_id\n");
  printf("  -r <RNTI>        覆盖配置文件中的rnti (十六进制)\n");
  printf("  -m <MCS>         覆盖配置文件中的mcs\n");
  printf("  -n <子帧数>      覆盖配置文件中的nof_subframes\n");
  printf("  -l <日志文件>    日志文件名\n");
  printf("  -v               增加详细输出\n");
  printf("\n示例:\n");
  printf("  %s -I uhd -a type=b205i -f 5900000000 -g 20\n", prog);
}

// 参数解析
static void parse_args(prog_args_t* args, int argc, char** argv)
{
  int opt;
  while ((opt = getopt(argc, argv, "I:a:f:g:p:c:r:m:n:l:v")) != -1) {
    switch (opt) {
      case 'I':
        args->rf_dev = strdup(optarg);
        break;
      case 'a':
        args->rf_args = strdup(optarg);
        break;
      case 'f':
        args->rf_freq = atof(optarg);
        break;
      case 'g':
        args->rf_gain = atof(optarg);
        break;
      case 'p':
        args->nof_prb = atoi(optarg);
        break;
      case 'c':
        args->cell_id = atoi(optarg);
        break;
      case 'r':
        args->rnti = strtol(optarg, NULL, 16);
        break;
      case 'm':
        args->mcs = atoi(optarg);
        break;
      case 'n':
        args->nof_subframes = atoi(optarg);
        break;
      case 'l':
        args->log_file = strdup(optarg);
        break;
      case 'v':
        increase_srsran_verbose_level();
        break;
      default:
        usage(argv[0]);
        exit(-1);
    }
  }
}

// 记录日志
static void log_message(const char* filename, const char* format, ...)
{
  FILE* f = fopen(filename, "a");
  if (f) {
    va_list args;
    va_start(args, format);
    vfprintf(f, format, args);
    va_end(args);
    fclose(f);
  }
}

// 主函数
int main(int argc, char** argv)
{
  int ret;
  prog_args_t prog_args;
  srsran_rf_t radio;
  srsran_pscch_t pscch;
  srsran_pssch_t pssch;
  srsran_cell_sl_t cell;
  srsran_sl_comm_resource_pool_t sl_comm_resource_pool;
  srsran_pssch_cfg_t pssch_cfg;
  srsran_sci_format0_t sci;
  
  // 初始化崩溃处理
  srsran_debug_handle_crash(argc, argv);
  
  // 设置默认参数
  args_default(&prog_args);
  
  // 解析命令行参数
  parse_args(&prog_args, argc, argv);
  
  // 设置信号处理
  signal(SIGINT, sig_int_handler);
  
  // 配置cell参数
  cell.nof_prb = prog_args.nof_prb;
  cell.nof_ports = prog_args.nof_ports;
  cell.id = prog_args.cell_id;
  cell.cp = SRSRAN_CP_NORM;
  cell.tm = SRSRAN_SIDELINK_TM1;
  
  // 配置资源池参数
  sl_comm_resource_pool.prb_start = 0;
  sl_comm_resource_pool.prb_end = cell.nof_prb - 1;
  sl_comm_resource_pool.prb_num = cell.nof_prb;
  
  // 配置PSSCH参数
  pssch_cfg.prb_start_idx = 0;
  pssch_cfg.nof_prb = cell.nof_prb;
  pssch_cfg.mcs_idx = prog_args.mcs;
  pssch_cfg.rv_idx = prog_args.rv;
  pssch_cfg.N_x_id = cell.id;
  pssch_cfg.sf_idx = 0;
  
  // 初始化RF设备
  printf("Opening RF device...\n");
  if (srsran_rf_open_devname(&radio, prog_args.rf_dev, prog_args.rf_args, cell.nof_ports)) {
    fprintf(stderr, "Error opening rf\n");
    exit(-1);
  }
  
  // 配置RF参数
  srsran_rf_set_tx_freq(&radio, 0, prog_args.rf_freq);
  srsran_rf_set_tx_gain(&radio, prog_args.rf_gain);
  
  // 初始化PSCCH
  if (srsran_pscch_init(&pscch, cell.nof_prb) != SRSRAN_SUCCESS) {
    ERROR("Error initializing PSCCH");
    exit(-1);
  }
  if (srsran_pscch_set_cell(&pscch, cell) != SRSRAN_SUCCESS) {
    ERROR("Error setting PSCCH cell");
    exit(-1);
  }
  
  // 初始化PSSCH
  if (srsran_pssch_init(&pssch, &cell, &sl_comm_resource_pool) != SRSRAN_SUCCESS) {
    ERROR("Error initializing PSSCH");
    exit(-1);
  }
  if (srsran_pssch_set_cfg(&pssch, pssch_cfg) != SRSRAN_SUCCESS) {
    ERROR("Error setting PSSCH config");
    exit(-1);
  }
  
  // 分配缓冲区
  cf_t* sf_buffer = srsran_vec_cf_malloc(SRSRAN_SF_LEN_PRB(cell.nof_prb));
  if (!sf_buffer) {
    ERROR("Error allocating memory");
    exit(-1);
  }
  
  uint8_t* data = srsran_vec_u8_malloc(pssch.sl_sch_tb_len);
  if (!data) {
    ERROR("Error allocating memory");
    exit(-1);
  }
  
  // 记录启动信息
  log_message(prog_args.log_file, "LTE-V发射程序启动\n");
  log_message(prog_args.log_file, "发射频率: %.2f MHz\n", prog_args.rf_freq/1e6);
  log_message(prog_args.log_file, "发射增益: %.1f dB\n", prog_args.rf_gain);
  log_message(prog_args.log_file, "PRB数量: %d\n", prog_args.nof_prb);
  
  // 主循环
  int nf = 0;
  while ((nf < prog_args.nof_subframes || prog_args.nof_subframes == -1) && !go_exit) {
    // 准备SCI数据
    sci.format = 0;
    sci.freq_hopping = 0;
    sci.rb_alloc = pssch_cfg.prb_start_idx;
    sci.trp_idx = 0;
    sci.mcs = pssch_cfg.mcs_idx;
    sci.timing_advance = 0;
    sci.group_id = 0;
    
    // 准备传输数据
    for (int i = 0; i < pssch.sl_sch_tb_len; i++) {
      data[i] = rand() % 256;
    }
    
    // 清空子帧缓冲区
    srsran_vec_cf_zero(sf_buffer, SRSRAN_SF_LEN_PRB(cell.nof_prb));
    
    // 编码PSCCH
    if (srsran_pscch_encode(&pscch, sci.data, sf_buffer, pssch_cfg.prb_start_idx) != SRSRAN_SUCCESS) {
      ERROR("Error encoding PSCCH");
      break;
    }
    
    // 编码PSSCH
    if (srsran_pssch_encode(&pssch, data, pssch.sl_sch_tb_len, sf_buffer) != SRSRAN_SUCCESS) {
      ERROR("Error encoding PSSCH");
      break;
    }
    
    // 发送信号
    srsran_rf_send(&radio, sf_buffer, SRSRAN_SF_LEN_PRB(cell.nof_prb), true);
    
    // 记录发送信息
    log_message(prog_args.log_file, "子帧 %d: 成功发送LTE-V信号\n", nf);
    
    // 更新子帧索引
    pssch_cfg.sf_idx = (pssch_cfg.sf_idx + 1) % 10;
    
    // 等待下一个子帧
    usleep(1000);
    
    nf++;
  }
  
  // 记录结束信息
  log_message(prog_args.log_file, "LTE-V发射程序结束，共发送 %d 个子帧\n", nf);
  
  // 清理资源
  srsran_pscch_free(&pscch);
  srsran_pssch_free(&pssch);
  srsran_rf_close(&radio);
  free(sf_buffer);
  free(data);
  
  printf("Done\n");
  exit(0);
} 