/********************************************************************************
 *
 *       Copyright (C) 2015-2016 Ichiro Kawazome
 *       All rights reserved.
 * 
 *       Redistribution and use in source and binary forms, with or without
 *       modification, are permitted provided that the following conditions
 *       are met:
 * 
 *         1. Redistributions of source code must retain the above copyright
 *            notice, this list of conditions and the following disclaimer.
 * 
 *         2. Redistributions in binary form must reproduce the above copyright
 *            notice, this list of conditions and the following disclaimer in
 *            the documentation and/or other materials provided with the
 *            distribution.
 * 
 *       THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *       "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *       LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *       A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 *       OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *       SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *       LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *       DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *       THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *       (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *       OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 ********************************************************************************/
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/sysctl.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include <linux/kfifo.h>

#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>

#include <linux/delay.h>

#include <asm/page.h>
#include <asm/byteorder.h>

#include "globalmem.h"

#if     (LINUX_VERSION_CODE >= 0x030B00)
#define USE_DEV_GROUPS      1
#else
#define USE_DEV_GROUPS      0
#endif

/*
typedef struct opt_addr {
	union {
		struct{u8 port, node; } bs;
		u16 ws;
	} S_un;
} OPT_ADDR_T;
*/
typedef union {
	struct{uint8_t node, port; } bs;
	uint16_t ws;
} OPT_ADDR_T;

#pragma pack(1)
typedef struct txstart_format_t {
	uint32_t fsize;
	uint32_t rsize;
	char fname[32];
	uint8_t md5[16];
} TXSTART_FORMAT_T;
#pragma pack()
typedef TXSTART_FORMAT_T * pTXSTART_FORMAT_T;

#define PKT_PAYLOAD_LEN 195
#define SEND_BUF_MAX		PKT_PAYLOAD_LEN
#define KFIFO_BUFFER_SIZE_TX	512
#define TX_START_DATA_LEN       sizeof(TXSTART_FORMAT_T)


/********************************************************************************
 * Receive Control Status Registers
 *-------------------------------------------------------------------------------
 *             31            24              16               8               0
 *             +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *             |F|E|D|C|B|A|9|8|7|6|5|4|3|2|1|0|F|E|D|C|B|A|9|8|7|6|5|4|3|2|1|0|
 *             +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * Addr=0x001C |                                             |O|             |R|
 *             +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * Addr=0x0028 |                                               |    RX DATA    |
 *             +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * Addr=0x002C |                          PAGE CLEAR                           |
 *             +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * Addr=0x0030 |I|                                                           | |
 *             +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *-------------------------------------------------------------------------------
 * Status[7]   = read 1 : asserted I_LAST.
 * Status[6:1] = reserved.
 * Status[0]   = read 1 : when buffer no empty and Control[2]=1
 *-------------------------------------------------------------------------------
 * Control[7]  = write 1 : reset on.       write 0 : reset off.
 * Control[6]  = write 1 : pause on.       write 0 : pause off.
 * Control[5]  = write 1 : abort.          write 0 : do nothing.
 * Control[4]  = write 1 : received data.  write 0 : do nothing.
 * Control[3]  = reserved.
 * Control[2]  = write 1 : if buffer no empty then set 1 to Status[0]
 * Control[1]  = reserved.
 * Control[0]  = reserved.
 ********************************************************************************/

#define VPKT_RX_STA_REG        (0x001C)
#define VPKT_RX_DAT_REG        (0x0028)
#define VPKT_RX_CLR_REG        (0x002C)
#define VPKT_INTSTA_REG        (0x0030)
#define VPKT_INTCTL_REG        (0x0034)

#define VPKT_TX_DES_REG        (0x0004)
#define VPKT_TX_SRC_REG        (0x0008)
#define VPKT_TX_PID_REG        (0x000C)
#define VPKT_TX_DAT_REG        (0x0010)
#define VPKT_TX_RUN_REG        (0x0014)
#define VPKT_TX_STA_REG        (0x0018)

#define RX_INT         ((u32)((0x01)<<0))
#define TX_INT         ((u32)((0x01)<<1))
#define LU_INT         ((u32)((0x01)<<2))
#define LD_INT         ((u32)((0x01)<<3))

/********************************************************************************
 * constant
 ********************************************************************************/
#define DRIVER_NAME        "zptty"
#define DEVICE_NUMS        8
#define ZPTTY_DEBUG_PHASE (0x00000001)
#define ZPTTY_DEBUG_IRQ   (0x00000002)

/********************************************************************************
 * Device driver structure
 ********************************************************************************/
/**
 * struct zptty_device_data - Device driver structure
 */
struct zptty_device_data {
	struct class class;
	struct device*       platform_device;
	struct device*       tty_device;
	int                  minor_number;
	struct tty_port      tty_port;
	struct cdev          cdev;
	struct mutex         sem;
	struct resource*     regs_res;
	struct resource*     irq_res;
	int                  open_count;
	void __iomem*        regs_addr;
	int                  irq;
	spinlock_t           irq_lock;
	struct work_struct   rx_irq_work;
	struct work_struct   tx_irq_work;
	struct work_struct   txstart_work;
	u8 *klayer_frame;
	wait_queue_head_t    tx_wait_queue;
	unsigned int         rx_buf_size;
	unsigned int         tx_buf_size;
	unsigned int         debug;
	unsigned int         rx_count;
	OPT_ADDR_T self_addr;
	pTXSTART_FORMAT_T    txstart_data;
	struct kfifo fifo_tx;
	uint8_t* send_buf;
};

/********************************************************************************
 * local variables.
 ********************************************************************************/
static struct tty_driver*  zptty_tty_driver = NULL;
static struct device*      zptty_device_table[DEVICE_NUMS];

/********************************************************************************
 * Device Register Map
 ********************************************************************************/

#define ZPTTY_TX_HEADER_ADDR    (0x0010)
#define ZPTTY_TX_BUF_SIZE_ADDR  (0x0014)
#define ZPTTY_TX_CONFIG_ADDR    (0x0016)
#define ZPTTY_TX_BUF_COUNT_ADDR (0x0018)
#define ZPTTY_TX_BUF_PTR_ADDR   (0x001A)
#define ZPTTY_TX_PUSH_SIZE_ADDR (0x001C)
#define ZPTTY_TX_STAT_ADDR      (0x001E)
#define ZPTTY_TX_CTRL_ADDR      (0x001F)

/*接收中断使能*/
inline  void zptty_rx_interrupt_enable(struct zptty_device_data* this)
{
	u32 tmp;
	dev_info(this->tty_device, "%s\n", __func__);
	tmp = ioread32( this->regs_addr + VPKT_INTCTL_REG);
	printk("tmpen=%x\n",tmp | RX_INT);
	iowrite32( tmp | RX_INT , this->regs_addr + VPKT_INTCTL_REG);
}
/*可发送中断使能*/
inline void zptty_tx_buf_empty_interrupt_enable(struct zptty_device_data* this)
{
	u32 tmp;
	dev_info(this->tty_device, "%s\n", __func__);
	tmp = ioread32( this->regs_addr + VPKT_INTCTL_REG);
	//iowrite32( tmp | TX_INT , this->regs_addr + VPKT_INTCTL_REG);
}
/*获取所有中断状态*/
inline u32 zptty_get_interrupt_status(struct zptty_device_data* this)
{
	dev_info(this->tty_device, "%s\n", __func__);
	return (u32)ioread32(this->regs_addr + VPKT_INTSTA_REG);
}

/*获取rd_ready状态*/
inline u32 zptty_rx_get_rd_ready(struct zptty_device_data* this)
{
	dev_info(this->tty_device, "%s\n", __func__);
	return (u32)( ioread32(this->regs_addr + VPKT_RX_STA_REG) );
}
/*获取wr_idel状态*/
inline u32 zptty_tx_get_wr_idel(struct zptty_device_data* this)
{
	dev_info(this->tty_device, "%s\n", __func__);
	return (u32)( ioread32(this->regs_addr + VPKT_TX_STA_REG) );
}
/*获取本机自己当前的光纤地址*/
inline u32 zptty_get_self_addr(struct zptty_device_data* this)
{
	return (u32)( ioread32(this->regs_addr + VPKT_TX_STA_REG) );
}
/*
inline u32 zptty_rx_get_intstat(struct zptty_device_data* this)
{
	dev_info(this->tty_device, "%s\n", __func__);
	return (u32)( RX_INT & ioread32(this->regs_addr + VPKT_INTSTA_REG) );
}
inline u32 zptty_tx_get_intstat(struct zptty_device_data* this)
{
	dev_info(this->tty_device, "%s\n", __func__);
	return (u32)( TX_INT & ioread32(this->regs_addr + VPKT_INTSTA_REG) );
}
inline u32 zptty_link_get_status(struct zptty_device_data* this)
{
	dev_info(this->tty_device, "%s\n", __func__);
	return (u32)( LU_INT & ioread32(this->regs_addr + VPKT_INTSTA_REG) );
}
*/
/*读数据寄存器*/
inline u8 zptty_get_rxdata(struct zptty_device_data* this)
{
	return (u8)( ioread32(this->regs_addr + VPKT_RX_DAT_REG) );
}

inline void zptty_clear_rx_flag(struct zptty_device_data* this)
{
	u32 tmp;
	tmp = (uint8_t)ioread32(this->regs_addr + VPKT_RX_CLR_REG);
}

/*清除中断控制寄存器中的RX中断*/
inline void zptty_rx_clear_status_and_interrupt_disable(struct zptty_device_data* this)
{
	u32 tmp;
	dev_info(this->tty_device, "%s\n", __func__);
	tmp = ioread32( this->regs_addr + VPKT_INTCTL_REG);
	tmp &= (~RX_INT);
	printk("tmp=%x\n",tmp);
	iowrite32( 0x00000000 , this->regs_addr + VPKT_INTCTL_REG);
}
/*清除中断控制寄存器中的TX中断*/
inline void zptty_tx_clear_status_and_interrupt_disable(struct zptty_device_data* this)
{
	u32 tmp;
	dev_info(this->tty_device, "%s\n", __func__);
	tmp = ioread32( this->regs_addr + VPKT_INTCTL_REG);
	iowrite32( tmp & (~TX_INT) , this->regs_addr + VPKT_INTCTL_REG);
}

/*TX发包过程中的写地址寄存器操作*/
inline void zptty_tx_set_macaddr(struct zptty_device_data* this, uint16_t des_ad, uint16_t src_ad)
{
	iowrite16((uint16_t)des_ad, this->regs_addr + VPKT_TX_DES_REG);
	iowrite16((uint16_t)src_ad, this->regs_addr + VPKT_TX_SRC_REG);
}
/*TX发包过程中的写长度寄存器操作*/
inline void zptty_tx_set_pid(struct zptty_device_data* this, uint8_t pid )
{
	iowrite8((uint8_t)pid, this->regs_addr + VPKT_TX_PID_REG);
}
/*TX发包过程中的写数据寄存器操作*/
inline void zptty_tx_set_data(struct zptty_device_data* this, uint8_t data )
{
	iowrite8((uint8_t)data, this->regs_addr + VPKT_TX_DAT_REG);
}
/*TX发包过程中的启动传输操作*/
inline void zptty_tx_transmit_start(struct zptty_device_data* this )
{
	iowrite8( 0x00, this->regs_addr + VPKT_TX_RUN_REG);
}
int tx_pkt_frame(struct zptty_device_data* this, uint8_t *data, int len)
{
	uint8_t pid=0;
	int i;
dev_info(this->tty_device, "%s\n", __func__);
	len = (len < PKT_PAYLOAD_LEN) ? len : PKT_PAYLOAD_LEN; 

	i = 0;
	while ( 0 == ( zptty_tx_get_wr_idel(this) ) ){

		udelay(1);
		if ( ++i > 500 ){
			dev_dbg(this->tty_device, "time out!");
			return -1;
		}
	}

	zptty_tx_set_macaddr(this, 0x1234 ,0x5678);

	zptty_tx_set_pid(this , pid);
	zptty_tx_set_data(this , len);

	for (i = 0; i < len ; i++ ){
		zptty_tx_set_data(this , (uint8_t)*(data+i));
	}

	/*START TX*/
	zptty_tx_transmit_start(this);

	return len;

}

#define ZPTTY_RX_HEADER_ADDR    (0x0020)
#define ZPTTY_RX_BUF_SIZE_ADDR  (0x0024)
#define ZPTTY_RX_CONFIG_ADDR    (0x0026)
#define ZPTTY_RX_BUF_COUNT_ADDR (0x0028)
#define ZPTTY_RX_BUF_PTR_ADDR   (0x002A)
#define ZPTTY_RX_PULL_SIZE_ADDR (0x002C)
#define ZPTTY_RX_STAT_ADDR      (0x002E)
#define ZPTTY_RX_CTRL_ADDR      (0x002F)

inline void zptty_tx_reset_on(struct zptty_device_data* this)
{
	iowrite8(0x80, this->regs_addr + ZPTTY_TX_CTRL_ADDR);
}
inline void zptty_tx_reset_off(struct zptty_device_data* this)
{
	iowrite8(0x00, this->regs_addr + ZPTTY_TX_CTRL_ADDR);
}
/*
inline void zptty_tx_buf_empty_interrupt_enable(struct zptty_device_data* this)
{
	iowrite8(0x04, this->regs_addr + ZPTTY_TX_CTRL_ADDR);
}*/
inline unsigned int zptty_tx_get_buf_count(struct zptty_device_data* this)
{
	u16 regs_val = le16_to_cpu(ioread16(this->regs_addr + ZPTTY_TX_BUF_COUNT_ADDR));
	return (unsigned int)(regs_val);
}
#define zptty_tx_get_buf_status(this, buf_offset, buf_count)                         \
{                                                                                    \
	u32 regs_val = le32_to_cpu(ioread32(this->regs_addr + ZPTTY_TX_BUF_COUNT_ADDR)); \
	(buf_count ) = (unsigned int)((regs_val      ) & 0xFFFF);                        \
	(buf_offset) = (unsigned int)((regs_val >> 16) & 0xFFFF);                        \
}

inline  void zptty_rx_reset_on(struct zptty_device_data* this)
{
	iowrite8(0x80, this->regs_addr + ZPTTY_RX_CTRL_ADDR);
}    
inline  void zptty_rx_reset_off(struct zptty_device_data* this)
{
	iowrite8(0x00, this->regs_addr + ZPTTY_RX_CTRL_ADDR);
}    


inline void zptty_rx_remove_from_buf(struct zptty_device_data* this, int size)
{
	u32 regs_val;
	regs_val      = (1 << 28) | (size & 0xFFFF);
	iowrite32(cpu_to_le32(regs_val), this->regs_addr + ZPTTY_RX_PULL_SIZE_ADDR);
}
#define zptty_rx_get_buf_status(this, buf_offset, buf_count)                         \
{                                                                                    \
	u32 regs_val = le32_to_cpu(ioread32(this->regs_addr + ZPTTY_RX_BUF_COUNT_ADDR)); \
	(buf_count)  = (unsigned int)((regs_val      ) & 0xFFFF);                        \
	(buf_offset) = (unsigned int)((regs_val >> 16) & 0xFFFF);                        \
}

/********************************************************************************
 * zptty tty dev_dbg_buf
 ********************************************************************************/
static void dev_dbg_buf(struct device* dev, const char* func, unsigned char* buf, unsigned int length)
{
	switch(length) {
		case 0 : break;
		case 1 : dev_dbg(dev, "%s [%02X]\n",
						 func, buf[0]);
				 break;
		case 2 : dev_dbg(dev, "%s [%02X,%02X]\n",
						 func, buf[0],buf[1]);
				 break;
		case 3 : dev_dbg(dev, "%s [%02X,%02X,%02X]\n",
						 func, buf[0],buf[1],buf[2]);
				 break;
		case 4 : dev_dbg(dev, "%s [%02X,%02X,%02X,%02X]\n",
						 func, buf[0], buf[1],buf[2],buf[3]);
				 break;
		case 5 : dev_dbg(dev, "%s [%02X,%02X,%02X,%02X,%02X]\n",
						 func, buf[0],buf[1],buf[2],buf[3],buf[4]);
				 break;
		case 6 : dev_dbg(dev, "%s [%02X,%02X,%02X,%02X,%02X,%02X]\n",
						 func, buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);
				 break;
		case 7 : dev_dbg(dev, "%s [%02X,%02X,%02X,%02X,%02X,%02X,%02X]\n",
						 func, buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6]);
				 break;
		case 8 : dev_dbg(dev, "%s [%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X]\n",
						 func, buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
				 break;
		case 9 : dev_dbg(dev, "%s [%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X]\n",
						 func, buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8]);
				 break;
		default: dev_dbg(dev, "%s [%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X...]\n",
						 func, buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8]);
				 break;
	}
}

static ssize_t rfs_file_info(struct class *cls, struct class_attribute *attr, char *buf)
{
	struct zptty_device_data *pdata = (struct watchdog_data *)container_of(cls,  struct zptty_device_data, class);

	return sprintf(buf, "%d/%d\n", pdata->txstart_data->rsize, pdata->txstart_data->fsize);

}
static ssize_t rfs_file_name(struct class *cls, struct class_attribute *attr, char *buf)
{
	struct zptty_device_data *pdata = (struct watchdog_data *)container_of(cls,  struct zptty_device_data, class);
	return sprintf(buf, "%s", pdata->txstart_data->fname);
}
static ssize_t rfs_file_md5(struct class *cls, struct class_attribute *attr, char *buf)
{
	int i;
	struct zptty_device_data *pdata = (struct watchdog_data *)container_of(cls,  struct zptty_device_data, class);
	for(i=0;i<16;i++){
		sprintf(buf+i, "%x", pdata->txstart_data->md5[i]);
	}
}

/* Attributes declaration: Here I have declared only one attribute attr1 */
static struct class_attribute zptty_class_attrs[] = {
	__ATTR(file_size, S_IRUGO | S_IWUSR , rfs_file_info/*r*/, NULL/*w*/), //use macro for permission
	__ATTR(file_name, S_IRUGO | S_IWUSR , rfs_file_name/*r*/, NULL/*w*/), //use macro for permission
	__ATTR(file_md5, S_IRUGO | S_IWUSR , rfs_file_md5/*r*/, NULL/*w*/), //use macro for permission
	__ATTR(file_lock_status, S_IRUGO | S_IWUSR , NULL/*r*/, NULL/*w*/), //use macro for permission
	__ATTR_NULL
};

/********************************************************************************
 * zptty tty interrupt service routeins.
 ********************************************************************************/
/**
 * zptty_irq() - The main interrupt handler.
 * @irq:	 The interrupt number.
 * @data:	 Pointer to the driver data structure.
 * returns: IRQ_HANDLED after the interrupt is handled.
 **/
static irqreturn_t zptty_irq(int irq, void *data)
{
	struct zptty_device_data* this = data;
	dev_info(this->tty_device,"IRQ interrupt of pack dev\n");
#if 1
	if (this->debug & ZPTTY_DEBUG_IRQ)
		dev_info(this->tty_device, "zptty_irq(this=%pK)\n", this);

	spin_lock(&this->irq_lock);
	{

		u32 stat = zptty_get_interrupt_status(this);
		printk("stat=%x\n",stat);
		//unsigned int rx_stat = zptty_rx_get_status(this);
		//unsigned int tx_stat = zptty_tx_get_status(this);
		u32 rx_stat = RX_INT & stat;
		u32 tx_stat = TX_INT & stat;
		if (rx_stat != 0) {
			zptty_rx_clear_status_and_interrupt_disable(this);

			schedule_work(&this->rx_irq_work);
		}
		if (tx_stat != 0) {
			zptty_tx_clear_status_and_interrupt_disable(this);
			//schedule_work(&this->tx_irq_work);
			wake_up_interruptible(&this->tx_wait_queue);
		}
	}
	spin_unlock(&this->irq_lock);

	if (this->debug & ZPTTY_DEBUG_IRQ)
		dev_info(this->tty_device, "zptty_irq() => success\n");
#endif
	return IRQ_HANDLED;
}

#define PID_TXFILE_ST        254
#define PID_TXFILE_PAYLOAD   253
#define PID_TXFILE_END       252
#define PID_PROTOCOL_FRAME   251
#define PID_EMU_TTY          250
/**
 *
 */

static void zptty_txstart_work(struct work_struct* work)
{
	int i;
	pGLOBALMEM_DEV_T p_gm_t;
	struct zptty_device_data* this = container_of(work, struct zptty_device_data, txstart_work);

	printk("size=%d\n",this->txstart_data->fsize);
	printk("name=%s\n",this->txstart_data->fname);
	//printk("md5: ");
	//for(i=0;i<16;i++){
	//	printk("%x",this->txstart_data->md5[i]);
	//}
	//printk("\n");

	p_gm_t = globalmem_init(this->txstart_data->fsize);

}
static void zptty_tty_rx_irq_work(struct work_struct* work)
{
	struct zptty_device_data* this = container_of(work, struct zptty_device_data, rx_irq_work);
	struct tty_port*          port;
	struct tty_struct*        tty;
	int i;
	OPT_ADDR_T des, src;
	uint8_t pid;
	uint8_t len;
	uint8_t payload[PKT_PAYLOAD_LEN];
	uint8_t tmp;

dev_info(this->tty_device, "%s\n", __func__);
	if (this == NULL)
		goto done;

	port = &this->tty_port;
	tty  = tty_port_tty_get(port);

	if (tty == NULL)
		goto done;

	dev_dbg(tty->dev, "%s start\n", __func__);
	{
#if (LINUX_VERSION_CODE >= 0x030900)
		struct tty_port*   tty_p = port;
#else
		struct tty_struct* tty_p = tty;
#endif
		while(1) {
			unsigned int   received_size;
			unsigned int   buf_offset;
/*rx io work*/
/* 1. check ready to read */
			unsigned int rx_stat = zptty_rx_get_rd_ready(this);
			if (rx_stat == 0) {
				printk("break\n");
				break;
			}
/* 2. read 4byte addr */
			des.bs.port = zptty_get_rxdata(this);
			des.bs.node = zptty_get_rxdata(this);
			src.bs.port = zptty_get_rxdata(this);
			src.bs.node = zptty_get_rxdata(this);
/* 3. read 1byte len field */
			len = zptty_get_rxdata(this);
			//printk("%x %x %x %x %x\n", mac.qr.a, mac.qr.b, mac.qr.c, mac.qr.d , len);
			len = (len<=PKT_PAYLOAD_LEN) ? len : PKT_PAYLOAD_LEN ;


/* 4. read payload */
			for(i=0; i < len; i++){
				payload[i] = zptty_get_rxdata(this);
			}
/* 5. clear page+1 */
			zptty_clear_rx_flag(this);
/*rx io work end*/
			print_hex_dump(KERN_INFO, "RX Frame: ", DUMP_PREFIX_ADDRESS, 16, 1, payload, len, true);
			//DUMP_PREFIX_ADDRESS,
#if 1
			pid = src.bs.port;
			printk("src.bs.port=%x\n",src.bs.port);
			if ( PID_EMU_TTY == pid ){
				tty_insert_flip_string(tty_p, payload, len);
				tty_flip_buffer_push(tty_p);
				this->rx_count += len;
			}else if( PID_TXFILE_ST == pid ){
				printk("get PID_TXFILE_ST mode\n");
				memcpy(this->txstart_data, payload, len);

				//

			}else if( PID_TXFILE_PAYLOAD == pid ){

				//p_gm_t

			}else if( PID_TXFILE_END == pid ){

			}
#endif
		}
	}

	zptty_rx_interrupt_enable(this);

	dev_info(tty->dev, "%s done(%d)\n", __func__, this->rx_count);

done:
	return;


}

/**
 *
 */
static void zptty_tty_tx_irq_work(struct work_struct* work)
{
	struct zptty_device_data* this = container_of(work, struct zptty_device_data, tx_irq_work);
	struct tty_port*          port;
	struct tty_struct*        tty;
	u32 count,tx_cnt;
	u8* p_buf;
	u32 frame_nums,remain;
	int i;
	if (this == NULL)
		goto done;

	port = &this->tty_port;
	tty  = port->tty;

	if (tty == NULL)
		goto done;

	count = kfifo_out(&this->fifo_tx, this->send_buf, KFIFO_BUFFER_SIZE_TX);
	p_buf = this->send_buf;

	if(count > PKT_PAYLOAD_LEN){

		frame_nums = count/PKT_PAYLOAD_LEN;
		remain = count%PKT_PAYLOAD_LEN;

		for( i=0; i<frame_nums; i++ ){
			tx_cnt += tx_pkt_frame(this, p_buf , PKT_PAYLOAD_LEN);
			p_buf+=PKT_PAYLOAD_LEN;
		}
		tx_cnt += tx_pkt_frame(this, p_buf , remain);

	}else if(  (count <= PKT_PAYLOAD_LEN) && (count > 0) ){
		tx_cnt = tx_pkt_frame(this, p_buf, count);
	}
	//printk("tx_cnt=%d\n",tx_cnt);

	dev_dbg(tty->dev, "%s\n", __func__);

	

done:
	return;
}

/********************************************************************************
 * zptty tty port operations.
 ********************************************************************************/
/**
 * zptty_port_activate() -- Used to start/initialize any items upon first opening of tty port.
 *
 * @port: The tty port.
 * @tty:  The tty struct assocated with this device.
 *
 * Returns:
 *  always returns 0.
 */
static int zptty_port_activate(struct tty_port* port, struct tty_struct* tty)
{
	struct zptty_device_data* this = port->tty->driver_data;
	unsigned long             irq_flags;
	int                       retval = 0;
	int ret;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20) && LINUX_VERSION_CODE < KERNEL_VERSION(4,1,0))
	unsigned long             irq_req_flags = IRQF_DISABLED | IRQF_SHARED;
#else 
	unsigned long             irq_req_flags = IRQF_SHARED;
#endif
	dev_dbg(port->tty->dev, "%s\n", __func__);

	spin_lock_irqsave(&this->irq_lock, irq_flags);
	{
		zptty_rx_reset_off(this);
		zptty_rx_interrupt_enable(this);
		zptty_tx_reset_off(this);
		this->rx_count = 0;
	}
	spin_unlock_irqrestore(&this->irq_lock, irq_flags);

	ret = request_irq(this->irq, zptty_irq, IRQF_TRIGGER_RISING |\
					IRQF_SHARED |\
					IRQF_ONESHOT, \
					"vvtpackdev", \
					this);
	if(ret !=0 ){
		dev_info(port->tty->dev, "request_irq(%pr) failed\n", this->irq_res);
		retval = -ENODEV;
		goto failed;	
	}

	return 0;

 failed:
	return retval;
}
/**
 * zptty_port_shutdown() -- Used to stop/shutdown any items upon the last tty port close.
 *
 * @port: The tty port.
 *
 */
static void zptty_port_shutdown(struct tty_port* port)
{
	struct zptty_device_data* this = port->tty->driver_data;
	unsigned long             irq_flags;

	dev_dbg(port->tty->dev, "%s\n", __func__);

	cancel_work_sync(&this->rx_irq_work);
	cancel_work_sync(&this->tx_irq_work);
	cancel_work_sync(&this->txstart_work);
	free_irq(this->irq, this);

	spin_lock_irqsave(&this->irq_lock, irq_flags);
	{
		zptty_tx_reset_on(this);
		zptty_rx_reset_on(this);
	}
	spin_unlock_irqrestore(&this->irq_lock, irq_flags);
}

static const struct tty_port_operations zptty_port_ops  = {
	.activate    = zptty_port_activate,
	.shutdown    = zptty_port_shutdown,
};

/********************************************************************************
 * zptty tty operations.
 ********************************************************************************/
/**
 * zptty_tty_install() -- Used to setup specific zptty device to tty struct.
 *
 * @driver: tty driver information.
 * @tty: tty struct containing zptty device information.
 *
 * Returns:
 *  0 for scucess
 *  otherwise, error
 */
static int zptty_tty_install(struct tty_driver* driver, struct tty_struct* tty)
{
	struct device*            device;
	struct zptty_device_data* this;
	int                       status = 0;

	dev_dbg(tty->dev, "%s\n", __func__);

	if (tty->index >= DEVICE_NUMS)
		return -ENODEV;

	if ((device = zptty_device_table[tty->index]) == NULL)
		return -ENODEV;

	if ((this = (struct zptty_device_data*)dev_get_drvdata(device)) == NULL)
		return -ENODEV;

	if ((status = tty_port_install(&this->tty_port, driver, tty)) != 0)
		return status;

	tty->driver_data = this;

	return status;
}

/**
 * zptty_tty_cleanup() -- Used to de-allocation resouces tied to tty's of this driver.
 *
 * @tty: tty struct containing zptty device information.
 */
static void zptty_tty_cleanup(struct tty_struct* tty)
{
	dev_dbg(tty->dev, "%s\n", __func__);

	tty->driver_data = NULL;
}

/**
 * zptty_tty_open() -- Open an Application 
 *
 * @tty: tty interface.
 * @file: file interface passed to tty_port_open() call.
 * 
 * Returns:
 *  0 for scucess
 *  otherwise, fail value
 */
static int zptty_tty_open(struct tty_struct* tty, struct file* file)
{
	struct zptty_device_data* this = tty->driver_data;

	dev_dbg(tty->dev, "%s\n", __func__);

	if (this == NULL)
		return -EINVAL;

	return tty_port_open(&this->tty_port, tty, file);
}

/**
 * zptty_tty_close() -- Close tty device.
 *
 * @tty: tty interface.
 * @file: file interface passed to tty_port_close() call.
 * 
 */
static void zptty_tty_close(struct tty_struct* tty, struct file* file)
{
	struct zptty_device_data* this = tty->driver_data;

	dev_dbg(tty->dev, "%s\n", __func__);

	if (this == NULL)
		return;

	tty_port_close(&this->tty_port, tty, file);
}

/**
 * zptty_tty_write() -- Write data to zptty device.
 *
 * @tty: tty interface.
 * @buf: buffer pointer.
 * @count: transmit size.
 *
 * Returns:
 *   int, number of bytes written
 *   otherwise, error
 */
static int zptty_tty_write(struct tty_struct *tty, const unsigned char* buf, int count)
{
	struct zptty_device_data* this   = tty->driver_data;
	int                       retval = 0;
	int                       transmit_size;
	int                       buf_room_size;
	unsigned int              buf_count;
	unsigned int              buf_offset;
	unsigned char*            buf_ptr;

	dev_dbg(tty->dev, "%s(tty=%pK,buf=%pK,count=%d)\n", __func__, tty, buf, count);
	// dev_dbg_buf(tty->dev, __func__, (unsigned char*)buf, count);

	if (this == NULL)
		return -EL3HLT;

	buf_room_size = kfifo_avail(&this->fifo_tx);
    
	transmit_size = (buf_room_size < count) ? buf_room_size : count;

	retval = kfifo_in(&this->fifo_tx, (unsigned char *)buf, transmit_size);

	retval = transmit_size;

	dev_dbg(tty->dev, "%s => %d\n", __func__, retval);

	schedule_work(&this->tx_irq_work);

	return retval;

#if 0
	if(count > PKT_PAYLOAD_LEN){

		frame_nums = count/PKT_PAYLOAD_LEN;
		remain = count%PKT_PAYLOAD_LEN;

		for( i=0; i<frame_nums; i++ ){
			tx_cnt += tx_pkt_frame(this, p_buf , PKT_PAYLOAD_LEN);
			p_buf+=PKT_PAYLOAD_LEN;
		}
		tx_cnt += tx_pkt_frame(this, p_buf , remain);

	}else if(  (count <= PKT_PAYLOAD_LEN) && (count > 0) ){
		tx_cnt = tx_pkt_frame(this, p_buf, count);
	}
	//printk("tx_cnt=%d\n",tx_cnt);
#endif

}

/**
 * zptty_tty_hangup()
 */
static void zptty_tty_hangup(struct tty_struct *tty)
{
	struct zptty_device_data* this   = tty->driver_data;

	//dev_dbg(tty->dev, "%s\n", __func__);
	dev_info(tty->dev, "%s\n", __func__);
	if (this == NULL)
		return;

	tty_port_hangup(&this->tty_port);
}

/**
 * zptty_tty_write_room()
 */
static int zptty_tty_write_room(struct tty_struct *tty)
{
	struct zptty_device_data* this   = tty->driver_data;
	int room = 4096;

	room = kfifo_avail(&this->fifo_tx);

	return room;

#if 0
	struct zptty_device_data* this   = tty->driver_data;
	unsigned int              buf_count;
	int                       retval = 0;

	dev_dbg(tty->dev, "%s\n", __func__);

	if (this == NULL)
		return -EINVAL;

	buf_count = zptty_tx_get_buf_count(this);

	retval = this->tx_buf_size - buf_count;

	dev_dbg(tty->dev, "%s => %d\n", __func__, retval);

	return retval;
	return 512;
#endif
	
}

/**
 * zptty_tty_wait_until_sent()
 */
static void zptty_tty_wait_until_sent(struct tty_struct *tty, int timeout)
{
	struct zptty_device_data* this   = tty->driver_data;
	unsigned int              buf_count;
	int                       status = 0;

	dev_dbg(tty->dev, "%s start\n", __func__);

	buf_count = zptty_tx_get_buf_count(this);

	if (buf_count > 0)
	{
		unsigned long irq_flags;

		dev_dbg(tty->dev, "%s wait\n", __func__);

		spin_lock_irqsave(&this->irq_lock, irq_flags);
		{
			zptty_tx_buf_empty_interrupt_enable(this);
		}
		spin_unlock_irqrestore(&this->irq_lock, irq_flags);
	
		status = wait_event_interruptible_timeout(
					 this->tx_wait_queue      , /* wait_queue_head_t wq */
					 1                        , /* bool condition       */
					 msecs_to_jiffies(timeout)  /* long timeout         */
				 );
	}
	dev_dbg(tty->dev, "%s done\n", __func__);
}

static struct tty_operations     zptty_serial_ops = {
	.install         = zptty_tty_install,
	.cleanup         = zptty_tty_cleanup,
	.open            = zptty_tty_open,
	.close           = zptty_tty_close,
	.hangup          = zptty_tty_hangup,
	.write           = zptty_tty_write,
	.write_room      = zptty_tty_write_room,
	.wait_until_sent = zptty_tty_wait_until_sent,
};

/*********************************************************************************
 * zptty_driver
 ********************************************************************************/
/**
 * zptty_device_probe() -  Probe call for the device.
 *
 * @pdev:	handle to the platform device structure.
 * Returns 0 on success, negative error otherwise.
 *
 * It does all the memory allocation and registration for the device.
 */
static int zptty_device_probe(struct platform_device *pdev)
{
	struct zptty_device_data* this      = NULL;
	int                       retval    = 0;
	unsigned long             regs_addr = 0L;
	unsigned long             regs_size = 0L;
	unsigned int              done      = 0;
	const unsigned int        DONE_DEVICE_REGISTER = (1 << 0);
	const unsigned int        DONE_REGS_RESOUCE    = (1 << 1);
	const unsigned int        DONE_MEM_REGION      = (1 << 2);
	const unsigned int        DONE_MAP_REGS_ADDR   = (1 << 4);
	const unsigned int        DONE_GET_IRQ_RESOUCE = (1 << 4);
	int ret;

	printk("zptty_device_probe\n");
	dev_info(&pdev->dev, "ZPTTY Driver probe start\n");
	/*
	 * create (zptty_device_data*)this.
	 */
	{
		this = kzalloc(sizeof(*this), GFP_KERNEL);
		if (IS_ERR_OR_NULL(this)) {
			dev_err(&pdev->dev, "couldn't allocate device private record\n");
			retval = PTR_ERR(this);
			this = NULL;
			goto failed;
		}
		dev_set_drvdata(&pdev->dev, this);
		this->platform_device = &pdev->dev;
	}
	/*
	 * initialize tty port.
	 */
	{
		tty_port_init(&this->tty_port);
		this->tty_port.ops = &zptty_port_ops;
	}
	/*
	 * get minor number.
	 */
	{
		int status;
		u32 minor_number;
		status = of_property_read_u32(pdev->dev.of_node, "minor-number", &minor_number);
		if (status != 0) {
			dev_err(&pdev->dev, "invalid property minor number\n");
			retval = -ENODEV;
			goto failed;
		}
		if (minor_number >= DEVICE_NUMS) {
			dev_err(&pdev->dev, "invalid property minor number\n");
			retval = -ENODEV;
			goto failed;
		}
		this->minor_number = minor_number;
	}
	/*
	 * register tty device
	 */
	{
		struct device* tty_device = tty_register_device(zptty_tty_driver, this->minor_number, &pdev->dev);
		if (IS_ERR_OR_NULL(tty_device)) {
			dev_err(&pdev->dev, "couldn't register tty device\n");
			retval = -ENODEV;
			goto failed;
		}
		zptty_device_table[this->minor_number] = tty_device;
		dev_set_drvdata(tty_device, this);
		this->tty_device = tty_device;
		done |= DONE_DEVICE_REGISTER;
	}
	/*
	 * get register resouce and ioremap to this->regs_addr.
	 */
	{
		this->regs_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (this->regs_res == NULL) {
			dev_err(&pdev->dev, "invalid register address\n");
			retval = -ENODEV;
			goto failed;
		}
		done |= DONE_REGS_RESOUCE;
		regs_addr = this->regs_res->start;
		regs_size = this->regs_res->end - this->regs_res->start + 1;

		if (request_mem_region(regs_addr, regs_size, dev_name(&pdev->dev)) == NULL) {
			dev_err(&pdev->dev, "couldn't lock memory region at %pr\n", this->regs_res);
			retval = -EBUSY;
			goto failed;
		}
		done |= DONE_MEM_REGION;

		this->regs_addr = ioremap_nocache(regs_addr, regs_size);
		if (this->regs_addr == NULL) {
		  dev_err(&pdev->dev, "ioremap(%pr) failed\n", this->regs_res);
			goto failed;
		}
		done |= DONE_MAP_REGS_ADDR;
	}
	/*
	 * get interrupt number to this->irq
	 */
	{
		this->irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
		if (this->irq_res == NULL) {
			dev_err(&pdev->dev, "interrupt not found\n");
			retval = -ENODEV;
			goto failed;
		}
		this->irq = this->irq_res->start;
		//this->irq = 29;
		done |= DONE_GET_IRQ_RESOUCE;
	}

	this->send_buf = kmalloc(SEND_BUF_MAX, GFP_KERNEL);

	if (!this->send_buf) {
		dev_err(&pdev->dev, "Could not allocate send buffer?\n");
		ret = -ENOMEM;
		goto err_free_sbuf;
	}
	this->txstart_data = kmalloc(TX_START_DATA_LEN, GFP_KERNEL);
	if (!this->txstart_data) {
		dev_err(&pdev->dev, "Could not allocate send buffer?\n");
		ret = -ENOMEM;
		goto err_free_sbuf;
	}

	if (kfifo_alloc(&this->fifo_tx, KFIFO_BUFFER_SIZE_TX,
				GFP_KERNEL)) {
		dev_err(&pdev->dev,
				"Could not allocate kfifo buffer\n");
		ret = -ENOMEM;
		goto err_free_kfifo;
	}
	
	/* Init class */
	this->class.name = DRIVER_NAME;
	this->class.owner = THIS_MODULE;
	this->class.class_attrs = zptty_class_attrs;
	ret = class_register(&this->class);
	if(ret) {
		printk(KERN_ERR "class_register failed!\n");
		goto class_register_fail;
	}

	/*
	 *
	 */
	this->debug = 0;
	mutex_init(&this->sem);
	spin_lock_init(&this->irq_lock);
	INIT_WORK(&this->rx_irq_work, zptty_tty_rx_irq_work);
	INIT_WORK(&this->tx_irq_work, zptty_tty_tx_irq_work);
	INIT_WORK(&this->txstart_work, zptty_txstart_work);
	init_waitqueue_head(&this->tx_wait_queue);
	//this->tx_buf_size = (unsigned int)le16_to_cpu(ioread16(this->regs_addr + ZPTTY_TX_BUF_SIZE_ADDR));
	//this->rx_buf_size = (unsigned int)le16_to_cpu(ioread16(this->regs_addr + ZPTTY_RX_BUF_SIZE_ADDR));
	this->tx_buf_size = 256;
	this->rx_buf_size = 256;
	/*
	 *
	 */
	dev_info(&pdev->dev, "driver installed\n");
	dev_info(&pdev->dev, "device name    = %s\n"           , dev_name(this->tty_device));
	dev_info(&pdev->dev, "private record = %pK (%dbytes)\n", this, sizeof(*this));
	dev_info(&pdev->dev, "major number   = %d\n"           , zptty_tty_driver->major);
	dev_info(&pdev->dev, "minor number   = %d\n"           , this->minor_number);
	dev_info(&pdev->dev, "regs resource  = %pr\n"          , this->regs_res );
	dev_info(&pdev->dev, "regs address   = %pK\n"          , this->regs_addr);
	dev_info(&pdev->dev, "irq resource   = %pr\n"          , this->irq_res  );
	dev_info(&pdev->dev, "tx buf size    = %d\n"           , this->tx_buf_size);
	dev_info(&pdev->dev, "rx buf size    = %d\n"           , this->rx_buf_size);

	return 0;

class_register_fail:

err_free_kfifo:

	kfifo_free(&this->fifo_tx);
err_free_sbuf:
	kfree(this->txstart_data);
	kfree(this->send_buf);


 failed:
	if (done & DONE_MAP_REGS_ADDR)   { iounmap(this->regs_addr); }
	if (done & DONE_MEM_REGION   )   { release_mem_region(regs_addr, regs_size);}
	if (done & DONE_DEVICE_REGISTER) { tty_unregister_device(zptty_tty_driver, this->minor_number);}
	if (done & DONE_DEVICE_REGISTER) { zptty_device_table[this->minor_number] = NULL;}
	if (this != NULL)                { dev_set_drvdata(&pdev->dev, NULL); kfree(this);}
	return retval;



}

/**
 * zptty_device_remove() -  Remove call for the device.
 *
 * @pdev:	handle to the platform device structure.
 * Returns 0 or error status.
 *
 * Unregister the device after releasing the resources.
 */
static int zptty_device_remove(struct platform_device *pdev)
{
	struct zptty_device_data* this = dev_get_drvdata(&pdev->dev);

	if (!this)
		return -ENODEV;

	kfifo_free(&this->fifo_tx);

	kfree(this->send_buf);
	kfree(this->txstart_data);

	iounmap(this->regs_addr);
	release_mem_region(this->regs_res->start, this->regs_res->end - this->regs_res->start + 1);
	tty_unregister_device(zptty_tty_driver, this->minor_number);
	zptty_device_table[this->minor_number] = NULL;
	dev_set_drvdata(&pdev->dev, NULL);
	kfree(this);

	dev_info(&pdev->dev, "driver removed\n");
	return 0;
}

/**
 * Open Firmware Device Identifier Matching Table
 */
static struct of_device_id zptty_of_match[] = {
	{ .compatible = "ikwzm,zptty-0.10.a", },
	{ /* end of table */}
};

MODULE_DEVICE_TABLE(of, zptty_of_match);

/**
 * Platform Driver Structure
 */
static struct platform_driver zptty_platform_driver = {
	.probe  = zptty_device_probe,
	.remove = zptty_device_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name  = DRIVER_NAME,
		.of_match_table = of_match_ptr(zptty_of_match),
	},
};

/*********************************************************************************
 * Module functions.
 ********************************************************************************/
/**
 * zptty_module_init()
 */
static int __init zptty_module_init(void)
{
	int                i;
	int                retval = 0;
	unsigned int       done   = 0;
	const unsigned int DONE_TTY_DRV_REGISTER = (1 << 0);
	const unsigned int DONE_PLT_DRV_REGISTER = (1 << 1);

	globalmem_init(2654720);
printk("zptty_module_init\n");
	for (i = 0; i < DEVICE_NUMS; i++){
		zptty_device_table[i] = NULL;
	}

	zptty_tty_driver = alloc_tty_driver(DEVICE_NUMS);
	if (IS_ERR_OR_NULL(zptty_tty_driver)) {
		pr_err("%s: couldn't create tty driver\n", DRIVER_NAME);
		retval = PTR_ERR(zptty_tty_driver);
		zptty_tty_driver = NULL;
		goto failed;
	}
	zptty_tty_driver->owner        = THIS_MODULE;
	zptty_tty_driver->driver_name  = DRIVER_NAME;
	zptty_tty_driver->name         = DRIVER_NAME;
	zptty_tty_driver->major        = 0;
	zptty_tty_driver->minor_start  = 0;
	zptty_tty_driver->type         = TTY_DRIVER_TYPE_SERIAL;
	zptty_tty_driver->subtype      = SERIAL_TYPE_NORMAL;
	zptty_tty_driver->flags        = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	zptty_tty_driver->init_termios = tty_std_termios;
	// zptty_tty_driver->init_termios.c_flag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	tty_set_operations(zptty_tty_driver, &zptty_serial_ops);

	retval = tty_register_driver(zptty_tty_driver);
	if (retval) {
		pr_err("%s: couldn't register tty driver\n", DRIVER_NAME);
		goto failed;
	}
	done |= DONE_TTY_DRV_REGISTER;

	retval = platform_driver_register(&zptty_platform_driver);
	if (retval) {
		pr_err("%s: couldn't register platform driver\n", DRIVER_NAME);
		goto failed;
	}
	done |= DONE_PLT_DRV_REGISTER;

	return 0;

 failed:
	if (done & DONE_PLT_DRV_REGISTER){platform_driver_unregister(&zptty_platform_driver);}
	if (done & DONE_TTY_DRV_REGISTER){tty_unregister_driver(zptty_tty_driver);}

	return retval;
}

/**
 * zptty_module_exit()
 */
static void __exit zptty_module_exit(void)
{
	platform_driver_unregister(&zptty_platform_driver);
	tty_unregister_driver(zptty_tty_driver);
	globalmem_exit();
}

module_init(zptty_module_init);
module_exit(zptty_module_exit);

MODULE_AUTHOR("ikwzm");
MODULE_DESCRIPTION("ZYNQ-PTTY Driver");
MODULE_LICENSE("GPL");
