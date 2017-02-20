#include "OV7740Control.h"

/*
* @param1: cam_fd 		--cameraMarvin file descriptor cam_fd = open(...
* @param2: slaveAddr	--camera I2C slave address
* @param3: regAddr 		-- register address
* @param4: data			-- data
* @param5: data_size 	--data size
* @param6: reg_size 	--camera register size (1,2 byte)
*
*/
int write_I2C_cam(int cam_fd,uint8_t slaveAddr,uint8_t regAddr,uint8_t data,uint8_t data_size,uint8_t reg_size)
{
	int err=0;
	camsys_i2c_info_t i2cinfo;
		
	i2cinfo.bus_num = 3;
	i2cinfo.slave_addr = slaveAddr; //0x6c; //0x20;
	i2cinfo.reg_addr = regAddr;//0x0103
	i2cinfo.reg_size = reg_size;//2
	i2cinfo.val = data;//
	i2cinfo.val_size = data_size;
	i2cinfo.i2cbuf_directly = 0;
	i2cinfo.speed = 100000;
	if(cam_fd > 0)
		err = ioctl(cam_fd, CAMSYS_I2CWR, &i2cinfo);
	if(err<0)
	{
		//printf(" write process ioctl error\n");
		return -1;
	}
	else
	{
		//printf(" write process ioctl success\n");
		return 0;
	}
}
/*
* @param1: cam_fd 		--cameraMarvin file descriptor cam_fd = open(...
* @param2: slaveAddr	--camera I2C slave address
* @param3: regAddr 		-- register address
* @param4: data			-- data pointer read data
* @param5: data_size 	--data size
* @param6: reg_size 	--camera register size (1,2 byte)
*
*/
int read_I2C_cam(int cam_fd,uint8_t slaveAddr,uint8_t regAddr,uint8_t *data,uint8_t data_size,uint8_t reg_size)
{
	int err=0;
	camsys_i2c_info_t i2cinfo;
		
	i2cinfo.bus_num = 3;
	i2cinfo.slave_addr = slaveAddr; //0x6c; //0x20;
	i2cinfo.reg_addr = regAddr;//0x0103
	i2cinfo.reg_size = reg_size;//2
	i2cinfo.val = 0;//
	i2cinfo.val_size = data_size;
	i2cinfo.i2cbuf_directly = 0;
	i2cinfo.speed = 100000;
	if(cam_fd > 0)
		err = ioctl(cam_fd,CAMSYS_I2CRD,&i2cinfo);
	if(err<0)
	{
		printf(" read process ioctl error\n");
		return -1;
	}
	else
	{
		*data = i2cinfo.val;
		//printf(" read process ioctl success\n");
		return 0;
	}
}
