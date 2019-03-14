# Module ID definition
uint8 MOD_ARM                   = 1 #0x01
uint8 MOD_MOTOR                 = 2 #0x02
uint8 MOD_STM32                 = 3 #0x03
uint8 MOD_CAMERA                = 4 #0x04
uint8 MOD_ODOMETRY              = 5 #0x05
uint8 MOD_LOCALIZATION          = 6 #0x06
uint8 MOD_LIDAR_AVOIDANCE       = 7 #0x07
uint8 MOD_LIDAR_IDENTIFY        = 8 #0x08
uint8 MOD_POWER                 = 9 #0x09
uint8 MOD_MOVE_CONTROL          = 10 #0x0a
uint8 MOD_P2P                   = 11 #0x0b
uint8 MOD_JOY                   = 12 #0x0c

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# error_code = module_id (High Byte) + module_err(Low Byte)
# for example:
#    module_id : 0x08
#    module_err: 0x03
#    err_no    : 0x0803
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++

# arm module error/warning, only error info here
uint16 ARM_COM_BREAK            = 257 #0x0101
uint16 ARM_LATCHED              = 258 #0x0102
uint16 ARM_LOW_VOL              = 259 #0x0103
uint16 ARM_FOLLOW_ERR           = 260 #0x0104

# vehicle module error/warning, only error info here
uint16 VEHC_COM_BREAK           = 513 #0x0201
uint16 VEHC_LATCHED             = 514 #0x0202
uint16 VEHC_LOW_VOL             = 515 #0x0203

# stm32 module warning/error 
uint16 STM_TIMEOUT              = 769 #0x0301

# camera module warning/error 
uint16 CAM_NO_CAM               = 1025 #0x0401
uint16 CAM_NO_INFO              = 1026 #0x0402
uint16 CAM_OPEN_FAIL            = 1027 #0x0403
uint16 CAM_GET_PARA             = 1028 #0x0404
uint16 CAM_SET_PARA             = 1029 #0x0405
uint16 CAM_START_FAIL           = 1030 #0x0406
uint16 CAM_CREAT_THR            = 1031 #0x0407
uint16 CAM_LOAD_DATA            = 1032 #0x0408
uint16 CAM_PARAM_FILE_NOT_LOAD  = 1033 #0x0409
uint16 CAM_STOPPED              = 1034 #0x0410
uint16 CAM_NO_CONF              = 1035 #0x0411
uint16 CAM_BARCODE_DIRTY        = 1036 #0x0412

# Odometry
uint16 ODO_DROP_BYTES           = 1281 #0x0501 
uint16 ODO_JUMP                 = 1282 #0x0502 

uint16 ODO_OPEN_FAIL            = 1299 #0x0509 
uint16 ODO_LOST_DATA            = 1288 #0x0508 

# Localication
uint16 LOC_X_CAL_MAX            = 1537 #0x0601 #
uint16 LOC_Y_CAL_MAX            = 1538 #0x0602 #
uint16 LOC_YAW_CAL_MAX          = 1539 #0x0603 #
uint16 LOC_LOST_NODE            = 1540 #0x0604 
uint16 LOC_NO_II_DEF            = 1541 #0x0605 #
uint16 LOC_CODE_NOT_DEFINED     = 1543 #0x0607 

uint16 LOC_TIMEOUT              = 1542 #0x0606 #
uint16 LOC_NAME_OORANGE         = 1544 #0x0608 #
uint16 LOC_YAW_DRIFTING         = 1545 #0x0609 #

# lidar avoidence
# error
uint16 LID_AVO_BLOCKED          = 1793 #0x0701
uint16 LID_AVO_DATA_TIMEOUT_1   = 1794 #0x0702
uint16 LID_AVO_DATA_TIMEOUT_2   = 1795 #0x0703
# warning
uint16 LID_AVO_BLOCKED_SHORT    = 1796 #0x0704


# Lidar object identification
uint16 LID_IDF_NO_OUTLINE       = 2049 #0x0801
uint16 LID_IDF_MAX_ORI          = 2050 #0x0802
uint16 LID_IDF_MAX_WID          = 2051 #0x0803
uint16 LID_IDF_MAX_DIS          = 2052 #0x0804

uint16 LID_IDF_NO_WHEEL         = 2053 #0x0804
uint16 LID_IDF_MIN_INTV         = 2054 #0x0805
uint16 LID_IDF_NO_DATA          = 2055 #0x0806
uint16 LID_IDF_OPEN_FAIL        = 2056 #0x0807

#power supply
uint16 DISCHG_FAULT             = 2304 #0x0900
uint16 CHARGE_FAULT             = 2305 #0x0901
uint16 UNIT_VOL_HIGH            = 2306 #0x0902
uint16 UNIT_VOL_LOW             = 2307 #0x0903
uint16 TOTAL_VOL_HIGH           = 2308 #0x0904
uint16 TOTAL_VOL_LOW            = 2309 #0x0905
uint16 FDB_CURR_HIGH            = 2310 #0x0906
uint16 DISCHG_CURR_HIGH         = 2311 #0x0907
uint16 TEMPERATURE_HIGH         = 2312 #0x0908
uint16 TEMPERATURE_LOW          = 2313 #0x0909
uint16 TEMP_DIFF_HIGH           = 2314 #0x090a
uint16 PRES_DIFF_HIGH           = 2315 #0x090b
uint16 SOC_LOW                  = 2316 #0x090c
uint16 SOC_HIGH                 = 2317 #0x090d
uint16 INSULATE_FAULT           = 2318 #0x090e
uint16 DETECT_V_FAULT           = 2319 #0x090f
uint16 DETECT_T_FAULT           = 2320 #0x0910
uint16 FDB_DISABLE              = 2321 #0x0911
uint16 NO_DEVICE                = 2322 #0x0912

# motion control
uint16 CTL_SPEED_HI              = 2561 #0x0a01 #
uint16 CTL_ACC_HI                = 2562 #0x0a02 #
uint16 CTL_YAW_RATE_HI           = 2563 #0x0a03 #
uint16 CTL_X_ERR_TOO_HI          = 2564 #0x0a04
uint16 CTL_Y_ERR_TOO_HI          = 2565 #0x0a05
uint16 CTL_YAW_ERR_TOO_HI        = 2566 #0x0a06
uint16 CTL_X_ERR_HI 			 = 2570 #0x0a0a
uint16 CTL_Y_ERR_HI 			 = 2571 #0x0a0b
uint16 CTL_YAW_ERR_HI 			 = 2572 #0x0a0c

uint16 CTL_FM_TIMEOUT            = 2567 #0x0a07 #
uint16 CTL_LOCA_TIMEOUT          = 2568 #0x0a08 #
uint16 CTL_MOTION_TIMEOUT        = 2569 #0x0a09 #
uint16 CTL_WRONG_MODE            = 2576 #0x0a10 # TO UPDATE

# P2P
uint16 P2P_TIMEOUT               = 2817 #0x0b01
uint16 P2P_BREAK                 = 2818 #0x0b02

# JOYSTIKER
uint16 JOY_SIG_LOST              = 3073 #0x0c01
uint16 JOY_DATA_ERR              = 3074 #0x0c02
uint16 JOY_SIG_TIMEOUT           = 3075 #0x0c03
uint16 JOY_SIG_ABR               = 3076 #0x0c04
uint16 JOY_NO_DEV                = 3077 #0x0c05



