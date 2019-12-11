# Message Template

## Definitions
 
|Name              |HEX |
|-:                |:-  |
|RW_CMD            |0x80|
|TRANSFER_SIZE_MASK|0x3f|

## Structure

|Byte|Bit|Meaning        |1    |0      |
|-:  |-: |:-             |:-   |:-     |
|0   |0  |RW_CMD         |valid|invalid|
|    |1  |type           |read |write  |
|    |2  |┎ transfer size|     |       |
|    |3  |┇              |     |       |
|    |   |┇              |     |       |
|    |4  |┇              |     |       |
|    |5  |┇              |     |       |
|    |6  |┇              |     |       |
|    |7  |┖              |     |       |
|    |   |               |     |       |
|1   |0  |┎ address      |     |       |
|    |1  |┇              |     |       |
|    |2  |┇              |     |       |
|    |3  |┇              |     |       |
|    |   |┇              |     |       |
|    |4  |┇              |     |       |
|    |5  |┇              |     |       |
|    |6  |┇              |     |       |
|    |7  |┖              |     |       |
|    |   |               |     |       |
|2   |0  |┎ data         |     |       |
|    |1  |┇              |     |       |
|    |2  |┇              |     |       |
|    |3  |┇              |     |       |
|    |   |┇              |     |       |
|    |4  |┇              |     |       |
|    |5  |┇              |     |       |
|    |6  |┇              |     |       |
|    |7  |┇              |     |       |

## Addresses

|type                  |id  |name                |
|-:                    |-:  |:-                  |
|GUI Variables         |`01`|G_FIRMWARE_VERSION  |
|                      |`02`|G_FULL_SCALE_CURRENT|
|                      |`03`|G_TORQUE_OLD        |
|                      |`04`|G_ISGAIN_OLD        |
|                      |`05`|G_BYPASS_INDEXER    |
|                      |`06`|G_BYPASS_INDEXER_OLD|
|                      |`07`|G_WRITE_ALL_REG     |
|                      |`08`|G_READ_ALL_REG      |
|                      |`09`|G_RESET_FAULTS      |
|                      |`10`|G_MANUAL_WRITE      |
|                      |`11`|G_WRITE_ADDR        |
|                      |`12`|G_WRITE_DATA        |
|                      |`13`|G_MANUAL_READ       |
|                      |`14`|G_READ_ADDR         |
|                      |`15`|G_READ_DATA         |
|Stepper Motion Profile|`16`|G_START_STOP_SPEED  |
|                      |`17`|G_TARGET_SPEED      |
|                      |`18`|G_ACCEL_RATE        |
|                      |`19`|G_TOTAL_NUM_STEPS   |
|                      |`20`|G_STEPS_TO_ACCEL    |
|                      |`21`|G_MOTOR_STATE       |
|                      |`22`|G_SPEED_PROFILE     |
|                      |`23`|G_SPEED_PROFILE_LOCK|
|                      |`24`|G_STEP_PROFILE      |
|                      |`25`|G_STEP_PROFILE_LOCK |
|Motor Status          |`26`|G_CUR_NUM_STEPS     |
|                      |`27`|G_CUR_SPEED         |
|                      |`28`|G_CUR_SPEED_TEMP    |
|                      |`29`|G_SPEED_INCR        |
|                      |`30`|G_ACCEL_FLAG        |
|DRV8711 GPIO          |`31`|G_nSLEEP            |
|                      |`32`|G_RESET             |
|                      |`33`|G_STEP_AIN1         |
|                      |`34`|G_DIR_AIN2          |
|                      |`35`|G_BIN2              |
|                      |`36`|G_BIN1              |
|                      |`37`|G_nFAULT            |
|                      |`38`|G_nSTALL            |