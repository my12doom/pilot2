#ifndef _EULER_API_H__
#define _EULER_API_H__



#include <stdint.h>

/*
	
	head	dev_id     cmd_id      len       data       check
	0xaa    (2byte)    (2byte)    (2byte)   (2byte)    (1byte)
*/
typedef enum
{
        API_UNPACK_HEADER = 0   ,
        API_UNPACK_DEVID    	,
        API_UNPACK_CMDID        ,
        API_UNPACK_LEN          ,
        API_UNPACK_DATA         ,
        API_UNPACK_CHECK        ,
//--------------------------------------
        API_UNPACK_ERR = -1
}api_unpack_step_u;


typedef enum
{
        API_UNPACK_RESULT_OK = 0    ,
        API_UNPACK_RESULT_ERR       ,
        API_UNPACK_RESULT_ING       ,
        API_UNPACK_RESULT_HEADER    ,
        API_UNPACK_FIFO_FULL    
}api_unpack_result_u;
    

#define API_HEADER      (0xaa)
#define API_INTERFACE
#define DATA_MAX_SIZE   (200u)
#define ERR_INDEX       (0xffff)

typedef struct
{
        uint16_t    dev_id;
        uint16_t    cmd_id;
        uint8_t     len;
        uint8_t     data_buf[DATA_MAX_SIZE];
}euler_api_msg_t;
typedef float f_t;
typedef struct _api_up_struct
{
	f_t q0;
	f_t q1;
	f_t q2;
	f_t q3;
	
	f_t vgx;
	f_t vgy;
	f_t vgz;

	f_t agx;
	f_t agy;
	f_t agz;
	
	f_t wx;
	f_t wy;
	f_t wz;

	f_t height;
	f_t status;	
}api_updata_t;
typedef struct
{
	f_t send_yaw;
	f_t send_pitch;
	f_t send_roll;
	f_t send_thr;
	uint32_t req_status;
}api_send_data_t;
typedef struct _api_send_battery
{
	uint16_t full_charge_capacity;
	uint16_t remaining_capacity;
	uint16_t pack_voltage;
	int16_t current;
	int16_t average_current;
	int16_t temperature;
	uint8_t capacity_percentage;
	uint8_t right;
}api_battery_t;
typedef uint8_t (*pfunc_write)(uint8_t* pbuf,uint8_t len);
typedef int16_t (*func_cmd_handler)(uint16_t cmd_id,uint8_t* pbuf,uint16_t len);

typedef struct _cmd_tab
{
    uint16_t            cmd_id;
    func_cmd_handler    pf_cmd_handler;
}cmd_handler_table_t;
typedef struct _dev_tab
{
    uint16_t                dev_id;
    cmd_handler_table_t*    p_cmd_handler_table;
}dev_handler_table_t;

typedef struct 
{
    uint8_t     cur_pack_step;
    uint8_t     cur_pack_data_len;
    uint8_t     cur_pack_data_index;
    uint32_t    num_pack_per_second;
    uint32_t    num_send_pack_err_count;
    uint32_t    num_send_pack_all_count;
    uint32_t    num_recv_pack_err_count;
    uint32_t    num_recv_pack_all_count;
    pfunc_write pf_write;
}euler_protocol_state_t;

API_INTERFACE int16_t init_euler_api_protocol(pfunc_write pf_drc_drite);
API_INTERFACE int16_t euler_api_send_data(uint16_t dev_id,uint16_t cmd_id, uint8_t* pbuf,uint16_t len);
API_INTERFACE void euler_api_recv_ontick(uint8_t char_data,dev_handler_table_t* p_handler_tab);

int16_t euler_api_unpack_by_char(uint8_t char_data,dev_handler_table_t* p_handler_tab);
int16_t euler_api_pack_data2(uint16_t dev_id,uint16_t cmd_id,uint8_t*pbuf,uint8_t len);
#endif //euler_API_H__
