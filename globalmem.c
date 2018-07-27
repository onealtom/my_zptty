

#include <linux/platform_device.h>
#include <linux/module.h>

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/io.h>


#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/slab.h>


#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <asm/io.h>
//#include <asm/system.h>
#include <asm/uaccess.h>

#include "globalmem.h"

//#define GLOBALMEM_SIZE 0x1000 // 4KB

// create device node in the board side.
// mknod /dev/globalmem c 120 0
#define GLOBALMEM_MAJOR 120 // preset major number

#define MAX_BUFF_SIZE 67108864
#define CDEV_NODE_NAME  "nodebuff"
// define ioctl cmd
#define GLOBALMEM_MAGIC 0x01
#define MEM_CLEAR _IO(GLOBALMEM_MAGIC, 0)

static int globalmem_major = GLOBALMEM_MAJOR;



struct globalmem_dev *globalmem_devp; // device struct instance


int globalmem_open(struct inode *inode, struct file *filp)
{
	int ret;
	// set device struct pointer to file privatedata pointer
	filp->private_data = globalmem_devp;

	globalmem_devp->magicid = 0x773FDC990;

	if(ret){
		return ret;
	}

	return 0;
}

int globalmem_release(struct inode *inode, struct file *filp)
{
	struct globalmem_dev *dev = filp->private_data;

	return 0;
}

static ssize_t globalmem_read(struct file *filp, char __user *buf, size_t size,
							  loff_t *ppos)
{
	unsigned long p = *ppos;
	unsigned int count = size;
	int ret = 0;
	struct globalmem_dev *dev = filp->private_data; // get device struct pointer

	// analysis and get valid read length
	if (p > dev->mem_size) // overflow
		return 0;

	if (count > dev->mem_size - p) // count is too large
		count = dev->mem_size - p;

	mutex_lock(&dev->mutex);
	// kernel buf -> user buf
	if (copy_to_user(buf, (void *)(dev->mem + p), count))
		ret = -EFAULT;
	else {
		*ppos += count;
		ret = count;
		//printk(KERN_INFO "read %d bytes from %ld\n", count, p);
	}
	mutex_unlock(&dev->mutex);

	return ret;
}

static ssize_t globalmem_write(struct file *filp, const char __user *buf,
							   size_t size, loff_t *ppos)
{
	unsigned long p = *ppos;
	unsigned int count = size;
	int ret = 0;
	struct globalmem_dev *dev = filp->private_data; // get device stuct pointer

	// analysis and get valid write length
	if (p >= dev->mem_size) // write overflow
		return 0;

	if (count > dev->mem_size - p) // write count is too large
	count = dev->mem_size - p;

	mutex_lock(&dev->mutex);

	// user buf -> kernel buf
	if (copy_from_user(dev->mem + p, buf, count))
		ret = -EFAULT;
	else {
		*ppos += count;
		ret = count;
		//printk(KERN_INFO "written %d bytes from %ld\n", count, p);
	}
	mutex_unlock(&dev->mutex);

	return ret;
}

static ssize_t globalmem_knl_write( pGLOBALMEM_DEV_T dev, u8* buf, size_t size, loff_t *ppos)
{
	unsigned long p = *ppos;
	unsigned int count = size;
	int ret = 0;
	//struct globalmem_dev *dev = filp->private_data; // get device stuct pointer

	// analysis and get valid write length
	if (p >= dev->mem_size) // write overflow
		return 0;

	if (count > dev->mem_size - p) // write count is too large
	count = dev->mem_size - p;

	mutex_lock(&dev->mutex);

	// user buf -> kernel buf
	if (copy_from_user(dev->mem + p, buf, count))
		ret = -EFAULT;
	else {
		*ppos += count;
		ret = count;
		//printk(KERN_INFO "written %d bytes from %ld\n", count, p);
	}
	mutex_unlock(&dev->mutex);

	return ret;
}

static loff_t globalmem_llseek(struct file *filp, loff_t offset, int orig)
{
	struct globalmem_dev *dev = filp->private_data;
	loff_t ret = 0;

	switch (orig) {
	case 0: // from the file head
		if (offset < 0 || ((unsigned int) offset > dev->mem_size)) {
			ret = -EINVAL;
			break;
		}
		filp->f_pos = (unsigned int) offset;
		ret = filp->f_pos;
		break;

	case 1: // from current position
		if ((filp->f_pos + offset) > dev->mem_size || (filp->f_pos + offset) < 0) {
			ret = - EINVAL;
			break;
		}
		filp->f_pos += offset;
		ret = filp->f_pos;
		break;
	}

	return ret;
}


static long globalmem_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{

	struct globalmem_dev *dev = filp->private_data; // get device stuct pointer
	
	switch (cmd) {
	case MEM_CLEAR:
		mutex_lock(&dev->mutex);
		memset(dev->mem, 0, dev->mem_size);
		mutex_unlock(&dev->mutex);
		printk(KERN_INFO "globalmem is set to zero\n");
		break;

	default:
		return -EINVAL; // not supported
	}

	return 0;
}

// file operations struct
static const struct file_operations globalmem_fops = {
	.owner = THIS_MODULE,
	.llseek = globalmem_llseek,
	.read = globalmem_read,
	.write = globalmem_write,
	.unlocked_ioctl = globalmem_ioctl,
	.open = globalmem_open,
	.release = globalmem_release,
};
struct class *my_class;

// init and add cdev struct
static void globalmem_setup_cdev(struct globalmem_dev * dev, int index)
{
	int err;
	int devno = MKDEV(globalmem_major, 0);

	cdev_init(&dev->cdev, &globalmem_fops);
	err = cdev_add(&dev->cdev, devno, 1);
	if (err)
		printk(KERN_NOTICE "Error %d adding globalmem", err);

	my_class =class_create(THIS_MODULE, "vavitel_class");
	if(IS_ERR(my_class)) {
		printk("Err: failed in creating class.\n");
		return 0;
	}

	/* register your own device in sysfs, and this will cause udevd to create corresponding device node */
	device_create(my_class,NULL, devno, NULL,CDEV_NODE_NAME);
}

// globalmem device init function
pGLOBALMEM_DEV_T globalmem_init(uint32_t input_size)
{
	int result;
	dev_t devno = MKDEV(globalmem_major, 0);

		// apply globalmem device kernel region
	if (globalmem_major)
		result = register_chrdev_region(devno, 1, "globalmem");
	else {
			// get major no dynamically
		result = alloc_chrdev_region(&devno, 0, 1, "globalmem");
		globalmem_major = MAJOR(devno);
	}

	if (result < 0)
		return NULL;

		// apply device struct memory
	globalmem_devp = kmalloc(sizeof(struct globalmem_dev), GFP_KERNEL);
	if (!globalmem_devp) {
		result = - ENOMEM;
		goto fail_malloc;
	}


	memset(globalmem_devp, 0, sizeof(struct globalmem_dev));

	input_size = ( input_size > MAX_BUFF_SIZE ) ? MAX_BUFF_SIZE : input_size ;

	globalmem_devp->mem_size=input_size;

	globalmem_devp->mem=kzalloc( globalmem_devp->mem_size , GFP_KERNEL);

	globalmem_setup_cdev(globalmem_devp, 0);
	mutex_init(&globalmem_devp->mutex);

	return globalmem_devp;

  fail_malloc:
	unregister_chrdev_region(devno, 1);
	return NULL;
}

// globalmem device exit function
void globalmem_exit(void)
{
	// del cdev struct
	cdev_del(&globalmem_devp->cdev); 
	// free device struct memory
	kfree(globalmem_devp->mem);
	kfree(globalmem_devp);

	device_destroy(my_class, MKDEV(globalmem_major, 0));
	if (my_class     != NULL){
		class_destroy(my_class);}

	// unregister device region
	unregister_chrdev_region(MKDEV(globalmem_major, 0), 1); 
}
