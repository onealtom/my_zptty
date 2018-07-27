
#ifndef _GLOBALMEM_LIB_H_
#define _GLOBALMEM_LIB_H_

#ifdef __cplusplus
extern "C" {
#endif

enum txstate_t {
	TX_STA_STATE,
	TX_MID_STATE,
	TX_END_STATE,
	TX_IDLE_STATE,
};

// globalmem struct
struct globalmem_dev {
	uint32_t  magicid ;
	struct cdev cdev; // cdev struct
	//unsigned char mem[GLOBALMEM_SIZE];

	u32  mem_size;
	u8   *mem;
	u32  ppos;/*max buff size64MB, u32 is ok*/
	enum txstate_t state;

	struct mutex mutex;

};

typedef struct globalmem_dev GLOBALMEM_DEV_T;
typedef GLOBALMEM_DEV_T* pGLOBALMEM_DEV_T;

pGLOBALMEM_DEV_T globalmem_init(uint32_t input_size);/*max buff size64MB, u32 is ok*/
void globalmem_exit(void);

#ifdef __cplusplus
}
#endif

#endif /*_GLOBALMEM_LIB_H_*/