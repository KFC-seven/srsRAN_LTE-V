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
#include <matio.h>
#include <sys/stat.h>

#include "srsran/phy/common/phy_common.h"
#include "srsran/phy/phch/pscch.h"
#include "srsran/phy/phch/pssch.h"
#include "srsran/phy/rf/rf.h"
#include "srsran/srsran.h"

// 全局变量
volatile bool go_exit = false;

// DMRS矩阵结构体
typedef struct {
  uint32_t rnti;
  cf_t* dmrs_matrix;
  uint32_t num_frames;
  uint32_t num_samples;
  uint32_t capacity;
} dmrs_matrix_t;

// 设备DMRS矩阵管理结构体
typedef struct {
  dmrs_matrix_t** matrices;  // 设备矩阵数组
  uint32_t num_devices;      // 当前设备数量
  uint32_t capacity;         // 当前容量
} device_manager_t;

// 程序参数结构体
typedef struct {
  // RF设备配置
  char* rf_dev;
  char* rf_args;
  float rf_freq;
  float rf_gain;
  uint32_t nof_rx_antennas;

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
  
  // 文件保存配置
  char* output_dir;
  char* log_file;
  
  // DMRS矩阵配置
  uint32_t initial_matrix_capacity;
  uint32_t initial_device_capacity;
} prog_args_t;

// 默认参数设置
static void args_default(prog_args_t* args)
{
  args->rf_dev = "uhd";
  args->rf_args = "";
  args->rf_freq = 5900000000; // 5.9GHz for V2X
  args->rf_gain = 20;
  args->nof_rx_antennas = 1;

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
  
  args->output_dir = "dmrs_data";
  args->log_file = "ltev_rx.log";
  args->initial_matrix_capacity = 1000;
  args->initial_device_capacity = 10;
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
  printf("  -f <频率>        覆盖配置文件中的rx_freq (Hz)\n");
  printf("  -g <增益>        覆盖配置文件中的rx_gain (dB)\n");
  printf("  -p <PRB数>       覆盖配置文件中的nof_prb\n");
  printf("  -c <小区ID>      覆盖配置文件中的cell_id\n");
  printf("  -r <RNTI>        覆盖配置文件中的rnti (十六进制)\n");
  printf("  -m <MCS>         覆盖配置文件中的mcs\n");
  printf("  -n <子帧数>      覆盖配置文件中的nof_subframes\n");
  printf("  -o <输出目录>    保存DMRS数据的目录\n");
  printf("  -l <日志文件>    日志文件名\n");
  printf("  -v               增加详细输出\n");
  printf("\n示例:\n");
  printf("  %s -I uhd -a type=b200,serial=30F9A43 -f 5900000000 -g 20 -o dmrs_data\n", prog);
}

// 参数解析
static void parse_args(prog_args_t* args, int argc, char** argv)
{
  int opt;
  while ((opt = getopt(argc, argv, "I:a:f:g:p:c:r:m:n:o:l:v")) != -1) {
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
      case 'o':
        args->output_dir = strdup(optarg);
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

// 初始化设备管理器
static device_manager_t* init_device_manager(uint32_t initial_capacity)
{
  device_manager_t* manager = calloc(1, sizeof(device_manager_t));
  if (manager) {
    manager->capacity = initial_capacity;
    manager->num_devices = 0;
    manager->matrices = calloc(initial_capacity, sizeof(dmrs_matrix_t*));
    if (!manager->matrices) {
      free(manager);
      return NULL;
    }
  }
  return manager;
}

// 释放设备管理器
static void free_device_manager(device_manager_t* manager)
{
  if (manager) {
    if (manager->matrices) {
      for (uint32_t i = 0; i < manager->num_devices; i++) {
        if (manager->matrices[i]) {
          free_dmrs_matrix(manager->matrices[i]);
        }
      }
      free(manager->matrices);
    }
    free(manager);
  }
}

// 扩展设备管理器容量
static bool expand_device_manager(device_manager_t* manager)
{
  uint32_t new_capacity = manager->capacity * 2;
  dmrs_matrix_t** new_matrices = realloc(manager->matrices, new_capacity * sizeof(dmrs_matrix_t*));
  if (new_matrices) {
    // 初始化新分配的空间
    memset(new_matrices + manager->capacity, 0, (new_capacity - manager->capacity) * sizeof(dmrs_matrix_t*));
    manager->matrices = new_matrices;
    manager->capacity = new_capacity;
    return true;
  }
  return false;
}

// 初始化DMRS矩阵
static dmrs_matrix_t* init_dmrs_matrix(uint32_t rnti, uint32_t num_samples, uint32_t initial_capacity)
{
  dmrs_matrix_t* matrix = calloc(1, sizeof(dmrs_matrix_t));
  if (matrix) {
    matrix->rnti = rnti;
    matrix->num_samples = num_samples;
    matrix->num_frames = 0;
    matrix->capacity = initial_capacity;
    matrix->dmrs_matrix = srsran_vec_cf_malloc(matrix->capacity * num_samples);
    if (!matrix->dmrs_matrix) {
      free(matrix);
      return NULL;
    }
  }
  return matrix;
}

// 释放DMRS矩阵
static void free_dmrs_matrix(dmrs_matrix_t* matrix)
{
  if (matrix) {
    if (matrix->dmrs_matrix) {
      free(matrix->dmrs_matrix);
    }
    free(matrix);
  }
}

// 扩展DMRS矩阵容量
static bool expand_dmrs_matrix(dmrs_matrix_t* matrix)
{
  uint32_t new_capacity = matrix->capacity * 2;
  cf_t* new_matrix = srsran_vec_cf_malloc(new_capacity * matrix->num_samples);
  if (new_matrix) {
    // 复制现有数据
    memcpy(new_matrix, matrix->dmrs_matrix, matrix->num_frames * matrix->num_samples * sizeof(cf_t));
    // 释放旧数据
    free(matrix->dmrs_matrix);
    // 更新矩阵
    matrix->dmrs_matrix = new_matrix;
    matrix->capacity = new_capacity;
    return true;
  }
  return false;
}

// 保存DMRS矩阵到MAT文件
static void save_dmrs_matrix(const char* output_dir, dmrs_matrix_t* matrix)
{
  if (!matrix || !matrix->dmrs_matrix || matrix->num_frames == 0) {
    return;
  }

  char filename[256];
  snprintf(filename, sizeof(filename), "%s/dmrs_rnti_0x%x.mat", output_dir, matrix->rnti);
  
  mat_t* matfp = Mat_CreateVer(filename, NULL, MAT_FT_MAT5);
  if (matfp) {
    size_t dims[2] = {matrix->num_frames, matrix->num_samples};
    
    // 创建临时缓冲区存储数据
    float* temp_data = malloc(matrix->num_frames * matrix->num_samples * 2 * sizeof(float));
    if (temp_data) {
      // 将复数数据转换为实部和虚部
      for (uint32_t i = 0; i < matrix->num_frames * matrix->num_samples; i++) {
        temp_data[i*2] = crealf(matrix->dmrs_matrix[i]);
        temp_data[i*2+1] = cimagf(matrix->dmrs_matrix[i]);
      }
      
      // 创建MAT变量
      matvar_t* matvar = Mat_VarCreate("dmrs_matrix", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, temp_data, 0);
      if (matvar) {
        Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_NONE);
        Mat_VarFree(matvar);
      }
      free(temp_data);
    }
    Mat_Close(matfp);
  }
}

// 查找或创建设备的DMRS矩阵
static dmrs_matrix_t* get_or_create_matrix(device_manager_t* manager, uint32_t rnti, uint32_t num_samples, uint32_t initial_capacity)
{
  // 查找现有矩阵
  for (uint32_t i = 0; i < manager->num_devices; i++) {
    if (manager->matrices[i] && manager->matrices[i]->rnti == rnti) {
      return manager->matrices[i];
    }
  }
  
  // 如果设备数量达到容量上限，扩展管理器
  if (manager->num_devices >= manager->capacity) {
    if (!expand_device_manager(manager)) {
      return NULL;
    }
  }
  
  // 创建新矩阵
  dmrs_matrix_t* matrix = init_dmrs_matrix(rnti, num_samples, initial_capacity);
  if (matrix) {
    manager->matrices[manager->num_devices++] = matrix;
  }
  return matrix;
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
  srsran_chest_sl_t chest;
  device_manager_t* device_manager = NULL;
  
  // 初始化崩溃处理
  srsran_debug_handle_crash(argc, argv);
  
  // 设置默认参数
  args_default(&prog_args);
  
  // 解析命令行参数
  parse_args(&prog_args, argc, argv);
  
  // 设置信号处理
  signal(SIGINT, sig_int_handler);
  
  // 创建输出目录
  if (mkdir(prog_args.output_dir, 0755) != 0 && errno != EEXIST) {
    fprintf(stderr, "Error creating output directory %s\n", prog_args.output_dir);
    exit(-1);
  }
  
  // 初始化设备管理器
  device_manager = init_device_manager(prog_args.initial_device_capacity);
  if (!device_manager) {
    fprintf(stderr, "Error initializing device manager\n");
    exit(-1);
  }
  
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
  srsran_rf_set_rx_freq(&radio, 0, prog_args.rf_freq);
  srsran_rf_set_rx_gain(&radio, prog_args.rf_gain);
  
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
  
  // 初始化信道估计
  if (srsran_chest_sl_init(&chest, &cell, &sl_comm_resource_pool) != SRSRAN_SUCCESS) {
    ERROR("Error initializing channel estimation");
    exit(-1);
  }
  
  // 分配缓冲区
  cf_t* sf_buffer = srsran_vec_cf_malloc(SRSRAN_SF_LEN_PRB(cell.nof_prb));
  if (!sf_buffer) {
    ERROR("Error allocating memory");
    exit(-1);
  }
  
  // 记录启动信息
  log_message(prog_args.log_file, "LTE-V接收程序启动\n");
  log_message(prog_args.log_file, "接收频率: %.2f MHz\n", prog_args.rf_freq/1e6);
  log_message(prog_args.log_file, "接收增益: %.1f dB\n", prog_args.rf_gain);
  log_message(prog_args.log_file, "PRB数量: %d\n", prog_args.nof_prb);
  
  // 主循环
  int nf = 0;
  while ((nf < prog_args.nof_subframes || prog_args.nof_subframes == -1) && !go_exit) {
    // 接收信号
    srsran_rf_recv(&radio, sf_buffer, SRSRAN_SF_LEN_PRB(cell.nof_prb), true, false, false);
    
    // 解码PSCCH获取RNTI
    uint32_t detected_rnti = 0;
    if (srsran_pscch_decode(&pscch, sf_buffer, &detected_rnti) == SRSRAN_SUCCESS) {
      // 提取DMRS导频
      cf_t* dmrs_received = NULL;
      if (srsran_chest_sl_pssch_get_dmrs(&chest, sf_buffer, &dmrs_received) == SRSRAN_SUCCESS) {
        // 获取或创建该RNTI的DMRS矩阵
        dmrs_matrix_t* matrix = get_or_create_matrix(device_manager, detected_rnti, chest.M_sc_rs, prog_args.initial_matrix_capacity);
        
        if (matrix) {
          // 如果矩阵已满，扩展容量
          if (matrix->num_frames >= matrix->capacity) {
            if (!expand_dmrs_matrix(matrix)) {
              // 如果扩展失败，保存当前数据并重新初始化
              save_dmrs_matrix(prog_args.output_dir, matrix);
              free_dmrs_matrix(matrix);
              matrix = get_or_create_matrix(device_manager, detected_rnti, chest.M_sc_rs, prog_args.initial_matrix_capacity);
            }
          }
          
          if (matrix) {
            // 添加新的DMRS数据
            memcpy(matrix->dmrs_matrix + matrix->num_frames * matrix->num_samples,
                   dmrs_received,
                   matrix->num_samples * sizeof(cf_t));
            matrix->num_frames++;
            
            // 记录接收信息
            log_message(prog_args.log_file, "子帧 %d: RNTI=0x%x, 成功接收DMRS导频 (总帧数: %d)\n", 
                       nf, detected_rnti, matrix->num_frames);
          }
        }
      } else {
        log_message(prog_args.log_file, "子帧 %d: RNTI=0x%x, DMRS导频提取失败\n", nf, detected_rnti);
      }
    } else {
      log_message(prog_args.log_file, "子帧 %d: PSCCH解码失败\n", nf);
    }
    
    // 更新子帧索引
    pssch_cfg.sf_idx = (pssch_cfg.sf_idx + 1) % 10;
    
    nf++;
  }
  
  // 保存所有设备的DMRS数据
  for (uint32_t i = 0; i < device_manager->num_devices; i++) {
    if (device_manager->matrices[i]) {
      save_dmrs_matrix(prog_args.output_dir, device_manager->matrices[i]);
    }
  }
  
  // 记录结束信息
  log_message(prog_args.log_file, "LTE-V接收程序结束，共接收 %d 个子帧\n", nf);
  
  // 清理资源
  free_device_manager(device_manager);
  srsran_pscch_free(&pscch);
  srsran_pssch_free(&pssch);
  srsran_chest_sl_free(&chest);
  srsran_rf_close(&radio);
  free(sf_buffer);
  
  printf("Done\n");
  exit(0);
} 