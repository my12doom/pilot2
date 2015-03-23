//#include "euler_api.h"
#include "string.h"
#include "data_interface.h"
using namespace std;
static euler_api_msg_t recv_msg_buf={0};
static euler_api_msg_t send_msg_buf={0};
static euler_protocol_state_t g_euler_api_data={0};

API_INTERFACE int16_t init_euler_api_protocol(pfunc_write pf_drc_drite)
{
    //-----------state init
    g_euler_api_data.cur_pack_step        =API_UNPACK_HEADER;
    g_euler_api_data.cur_pack_data_len    =0;
    g_euler_api_data.cur_pack_data_index  =0;
    g_euler_api_data.num_pack_per_second  =0;
    g_euler_api_data.pf_write             =pf_drc_drite;
    return 1;
}
API_INTERFACE int16_t euler_api_send_data(uint16_t dev_id,uint16_t cmd_id,uint8_t* pbuf,uint16_t len)
{
    return euler_api_pack_data2(dev_id,cmd_id,pbuf,len);
}
API_INTERFACE void euler_api_recv_ontick(uint8_t char_data,dev_handler_table_t* p_handler_tab)
{
    euler_api_unpack_by_char(char_data,p_handler_tab);
}
uint16_t find_dev_index(uint16_t dev_id,dev_handler_table_t* p_dev_handler_tab)
{
    uint16_t item_data=0;
    uint16_t item_index=0;
    if(dev_id == ERR_INDEX)return ERR_INDEX;
    if(p_dev_handler_tab==NULL)return ERR_INDEX;
    
    while(item_data!=ERR_INDEX)
    {
        item_data = p_dev_handler_tab[item_index].dev_id;
        if(item_data == dev_id)
        {
            return item_index;
        }
        item_index++;
    }
    return ERR_INDEX;
}
uint16_t find_cmd_index(uint16_t cmd_id,cmd_handler_table_t* p_cmd_handler_tab)
{  
    uint16_t item_data=0;
    uint16_t item_index=0;
    if(cmd_id == ERR_INDEX) return ERR_INDEX;
    if(p_cmd_handler_tab==NULL) return ERR_INDEX;
    while(item_data!=ERR_INDEX)
    {
        item_data = p_cmd_handler_tab[item_index].cmd_id;
        if(item_data == cmd_id)
        {
            return item_index;
        }
        item_index++;
    }
    return ERR_INDEX;
 }
 int16_t euler_api_unpack_by_char(uint8_t char_data,dev_handler_table_t* p_handler_tab)
{
    api_unpack_result_u ret_unpack_result = API_UNPACK_RESULT_ING;
    euler_protocol_state_t* pstate;
    pstate = &g_euler_api_data;
    switch(pstate->cur_pack_step)
    {
        case API_UNPACK_HEADER:
        {
	   
            ret_unpack_result = API_UNPACK_RESULT_HEADER;
            if(char_data == API_HEADER)
            {
                pstate->cur_pack_step = API_UNPACK_DEVID;
		
            }
        }
        break;
        case API_UNPACK_DEVID:
        {
	  
            static uint8_t nByte = 0;
            static uint16_t data_get = 0;
            data_get |= (char_data<<(nByte*8));
            nByte++;
            if(nByte==2)
            {
                    recv_msg_buf.dev_id = data_get;
                    data_get = 0;
                    nByte = 0;
                    pstate->cur_pack_step = API_UNPACK_CMDID;
            }
        }
        break;
        case API_UNPACK_CMDID:
        {
	   
            static uint8_t nByte = 0;
            static uint16_t data_get = 0;
            
            data_get |= (char_data<<(nByte*8));
            nByte++;
            if(nByte==2)
            {
                recv_msg_buf.cmd_id = data_get;
                data_get = 0;
                nByte = 0;
                pstate->cur_pack_step = API_UNPACK_LEN;
            }
        }
        break;
        case API_UNPACK_LEN:
        {
	    
            pstate->cur_pack_step = API_UNPACK_DATA;
            pstate->cur_pack_data_len = char_data;
            pstate->cur_pack_data_index = 0;
            recv_msg_buf.len    = char_data;
        }
        break;
        case API_UNPACK_DATA:
        {
	    
            if((pstate->cur_pack_data_index < pstate->cur_pack_data_len)&&(pstate->cur_pack_data_len < DATA_MAX_SIZE))
            {
                recv_msg_buf.data_buf[pstate->cur_pack_data_index] = char_data;
                pstate->cur_pack_data_index++;
                if(pstate->cur_pack_data_index >=pstate->cur_pack_data_len)
                {
                    pstate->cur_pack_step = API_UNPACK_CHECK;
                }
            }
            else
            {
                pstate->cur_pack_step = API_UNPACK_CHECK;
            }
        }
        break;
        case API_UNPACK_CHECK:
        {
	   
            uint8_t check_data = 0;
            uint8_t i          = 0;
            uint8_t *pubf = (uint8_t *)&recv_msg_buf;
            for(i=0;i<recv_msg_buf.len+5;i++)
            {
                check_data+=pubf[i];
            }
            ret_unpack_result = API_UNPACK_RESULT_OK;
            pstate->num_recv_pack_all_count++;
            pstate->cur_pack_step = API_UNPACK_HEADER;
            if(check_data!=char_data)
            {
                pstate->num_recv_pack_err_count++;
                ret_unpack_result = API_UNPACK_RESULT_ERR;

            }
            else 
            {
                //直接调用解包的Handler
                uint16_t dev_index = 0;
                uint16_t cmd_index = 0;

                dev_index = find_dev_index(recv_msg_buf.dev_id,p_handler_tab);
                cmd_index = find_cmd_index(recv_msg_buf.cmd_id,p_handler_tab[dev_index].p_cmd_handler_table);
                p_handler_tab[dev_index].p_cmd_handler_table[cmd_index].pf_cmd_handler(recv_msg_buf.cmd_id,
                                                                                       recv_msg_buf.data_buf,
                                                                                       recv_msg_buf.len);
            }
            //计算解包的速度
        }
        break;
        //----------------------------理论不会到这里，除非出现异常错误
		default:
        break;
	}
	return ret_unpack_result;
}
int16_t euler_api_pack_data2(uint16_t dev_id,uint16_t cmd_id,uint8_t* pbuf,uint8_t len)
{
    euler_protocol_state_t* pstate;
    uint8_t     i = 0;
    uint8_t check = 0;
    uint8_t send_buf[DATA_MAX_SIZE+10];
    uint8_t*pubf = (uint8_t *)&send_msg_buf;
    pstate = &g_euler_api_data;
    if(len>DATA_MAX_SIZE)
    {
        return -1;
    }
    send_msg_buf.dev_id = dev_id;
    send_msg_buf.cmd_id = cmd_id;
    send_msg_buf.len    = len;
    memcpy(send_msg_buf.data_buf,pbuf,len);
    for(i=0;i<send_msg_buf.len+5;i++)
    {
        check+=pubf[i];
    }
    send_buf[0] = API_HEADER;
    memcpy(&send_buf[1],&send_msg_buf,len+7);
    send_buf[len+7-1] = check;
    pstate->num_send_pack_all_count++;
    return pstate->pf_write(send_buf,len+7);
}
