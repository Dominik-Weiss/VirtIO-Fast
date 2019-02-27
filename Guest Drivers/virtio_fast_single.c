#include<linux/module.h>	    //needed for module_init and module_exit
#include<linux/kernel.h>	    //needed for KERN_INFO
#include<linux/init.h>		    //needed for the macros
#include<linux/proc_fs.h>	    //needed for proc files
#include<linux/uaccess.h>	    //needed for copy_from_user
#include<linux/virtio.h>	    //needed for vitio functions
#include<linux/virtio_config.h>	//needed for virtio_device_ready
#include <linux/completion.h>   //needed for waiting

//used to disable debug messages
//#define DEBUG_ENABLED
#ifdef DEBUG_ENABLED
 #define DEBUG
#else
 #define DEBUG for(;0;)
#endif

#define VIRTIO_DEV_ID 26
#define VIRTIO_FEATURES_VQ_SINGLEACK	0
#define VIRTIO_FEATURES_VQ_SINGLENOACK	1
#define VIRTIO_FEATURES_VQ_MULTI		5

//Max size for single transmission: 2^22 = 4194304, independent of package size and number of elements in queue

//Struct, which saves the supported devices information
const static struct virtio_device_id driv_idT[]=
{
	{ VIRTIO_DEV_ID, VIRTIO_DEV_ANY_ID, }, { 0, },
};

//Struct to save all the driver information
struct virt_fast_driv_dat
{
	//Data transmission queue
	struct virtqueue *queue;

	//Message Buffer
	char *vq_out_buf;
	char *vq_in_buf;
  
    //Number of last sent bytes
    size_t lastSentBytes;
    //Is there received data?
    bool dataPresent;
    //NoData message sent?
	bool msgSent;

	//Virtio Device structure
	struct virtio_device *dev;

	//proc directory entry
	struct proc_dir_entry *procdirectory;
};
struct virt_fast_driv_dat *virt_fast;

//Array, which saves the supported features
static unsigned int features[] = {VIRTIO_FEATURES_VQ_SINGLENOACK};

static ssize_t proc_read(struct file *file, char *buffer, size_t count, loff_t *offp)
{
	DEBUG printk(KERN_INFO"virtio_fast_single: proc_read");

	if(!virt_fast->dataPresent)
	{
		if(!virt_fast->msgSent)
		{
			DEBUG printk(KERN_INFO"virtio_fast_single proc_read: No data present");
			if(copy_to_user(buffer, "No data present\n",16))
			{
				printk(KERN_ERR"virtio_fast_single proc_read: Error copying message to user buffer");
				return -EFAULT;
			}
			virt_fast->msgSent = true;
			return count;
		}
		else
		{
			virt_fast->msgSent = false;
			return 0;
		}
	}
	else
	{
        virt_fast->msgSent = true;

        if(count > virt_fast->lastSentBytes)
        {
            count = virt_fast->lastSentBytes;
        }
		if (copy_to_user(buffer, virt_fast->vq_in_buf, count))
		{
            kfree(virt_fast->vq_in_buf);
			return -EFAULT;
		}

        kfree(virt_fast->vq_in_buf);

		virt_fast->dataPresent = false;
		return count;
	}
}


static ssize_t proc_write(struct file *file, const char *buffer, size_t bufSize, loff_t *offp)
{
	struct scatterlist *sls[2], sl_out, sl_in;
	DEBUG printk(KERN_INFO"virtio_fast_single: proc_write");

	virt_fast->lastSentBytes = bufSize;

	//Allocate memory for the buffers
	virt_fast->vq_out_buf = kzalloc(bufSize,GFP_KERNEL);
	if(virt_fast->vq_out_buf == NULL)
	{
		printk(KERN_ERR"virtio_fast_single proc_write: Error getting memory for outbuffer!");
		return ENOMEM;
	}
    virt_fast->vq_in_buf = kzalloc(bufSize,GFP_KERNEL);
    if(virt_fast->vq_in_buf == NULL)
    {
		printk(KERN_ERR"virtio_fast_single proc_write: Error getting memory for outbuffer!");
		return ENOMEM;
	}

	if(copy_from_user(virt_fast->vq_out_buf,buffer,bufSize))
	{
        printk(KERN_ERR"virtio_fast_single proc_write: Error copying data from user space!");
        kfree(virt_fast->vq_out_buf);
		return -1;
	}

	sg_init_one(&sl_out, virt_fast->vq_out_buf,bufSize);
    sg_init_one(&sl_in, virt_fast->vq_in_buf,bufSize);
	DEBUG printk(KERN_INFO"virtio_fast_single proc_write: scatterlist done\n");
    sls[0] = &sl_out;
    sls[1] = &sl_in;

    virtqueue_add_sgs(virt_fast->queue, sls,1,1,virt_fast, GFP_ATOMIC);

	DEBUG printk(KERN_INFO"virtio_fast_single proc_write: buffer attached\n");

	if(!virtqueue_kick(virt_fast->queue))
	{
		printk(KERN_ERR"virtio_fast_single proc_write: Kick failed\n");
        kfree(virt_fast->vq_out_buf);
		return -1;
	}
	DEBUG printk(KERN_INFO"virtio_fast_single proc_write: Kick successful\n");

    virt_fast->dataPresent = true;

	return bufSize;
}

//Struct for the file operations for the proc file
static struct file_operations procFunc =
{
	.owner = THIS_MODULE,
	.read  = proc_read,
	.write = proc_write
};


//Create a new proc file
static void create_proc(void)
{
    DEBUG printk(KERN_INFO"virtio_fast_single: create_proc\n");
	virt_fast->procdirectory = proc_mkdir("virtio",NULL);
	proc_create_data("virtio_fast_single",0660,virt_fast->procdirectory,&procFunc,NULL);
}

void virt_fast_remove(struct virtio_device *device)
{
	DEBUG printk(KERN_INFO"virtio_fast_single: remove\n");
}

static void vq_cb(struct virtqueue *vq)
{
	int size;
	DEBUG printk(KERN_INFO"virtio_fast_single: vqh callback\n");

	virtqueue_get_buf(vq,&size);
    
    if(size != virt_fast->lastSentBytes)
	{
		DEBUG printk(KERN_ERR"virtio_fast_single: Error: Size of received data does not match size of sent data, size: %d, lastSentBytes: %zu",size,virt_fast->lastSentBytes);
	}

	DEBUG printk(KERN_INFO"virtio_fast_single: received bytes: %d\n",size);
}

/**
 * This function is called, when the virtio core thinks, that this driver fits to a new device
 * return: 0 => this driver handles the device
 *	  	  <0 => this driver does not handle the device
*/
int virt_fast_probe(struct virtio_device *device)
{
    struct virtqueue *vqs[1];
	int ret;
	vq_callback_t *cbs[] = {vq_cb};
	static const char * const names[] = { "data" };

	DEBUG printk(KERN_INFO"virtio_fast_single: probe\n");

	//Allocate memory for this driver
	virt_fast = kzalloc(sizeof(struct virt_fast_driv_dat),GFP_KERNEL);
	if(virt_fast == NULL)
	{
		printk(KERN_ERR"virtio_fast_single probe: Error allocating memory for driver structure\n");
		return ENOMEM;
	}

	//Save this driver instance in the device structure and vise versa
	device->priv = virt_fast;
	virt_fast->dev = device;

	//Check if the device config is alright
	if (!device->config->get)
	{
		printk(KERN_ERR"virtio_fast_single probe: error no config\n");
		return -EINVAL;
	}

	//Reset the virtio device
	DEBUG printk(KERN_INFO"virtio_fast_single: reset device\n");
	virt_fast->dev->config->reset(virt_fast->dev);

	//Set the acknowledge bit
	virt_fast->dev->config->set_status(device,VIRTIO_CONFIG_S_ACKNOWLEDGE);

	//Set the driver bit
	virtio_add_status(virt_fast->dev, VIRTIO_CONFIG_S_DRIVER);

	//Check the features
	if(virtio_has_feature(virt_fast->dev,VIRTIO_FEATURES_VQ_SINGLENOACK))
	{
		DEBUG printk(KERN_INFO"virtio_fast_single probe: Single queue available");
	}

	//Set the features_ok bit
	virtio_add_status(virt_fast->dev, VIRTIO_CONFIG_S_FEATURES_OK);

	//Check if the features_ok bit is still set and the device accepts the features
	if((virt_fast->dev->config->get_status(virt_fast->dev) & VIRTIO_CONFIG_S_FEATURES_OK) != VIRTIO_CONFIG_S_FEATURES_OK)
	{
		printk(KERN_ERR"virtio_fast_single probe: Error with features!");
		return -1;
	}

	//Init the vqs
	ret = virtio_find_vqs(virt_fast->dev, 1, vqs, cbs, names, NULL);
	if(ret)
	{
		printk(KERN_ERR"virtio_fast_single: probe error getting vqs\n");
		return ret;
	}

	virt_fast->queue = vqs[0];

	DEBUG printk(KERN_INFO"virtio_fast_single probe: got vqs");

	if(virt_fast->queue == NULL)
	{
		printk(KERN_ERR"queue not ok\n");
		return -1;
	}
	
	//Set the driver_ok bit
	virtio_add_status(virt_fast->dev, VIRTIO_CONFIG_S_DRIVER_OK);

	virt_fast->dataPresent = false;
	virt_fast->lastSentBytes = 0;
	virt_fast->msgSent = false;

	return 0;
}

//Struct, which saves all the information about the virtio driver
static struct virtio_driver virt_fast_driv =
{
	.driver.name = "virtio-fast-single",
	.driver.owner = THIS_MODULE,
	.feature_table = features,
	.feature_table_size = ARRAY_SIZE(features),
	.id_table = driv_idT,
	.probe = virt_fast_probe,
	.remove =  virt_fast_remove
};

int __init hw_init(void)
{

    DEBUG printk(KERN_INFO"virtio_fast_single: started\n");

	//Register driver
	if(register_virtio_driver(&virt_fast_driv))
	{
		printk(KERN_ERR"Register of virtio_fast_single failed\n");
		return -1;
	}

	//Create proc input file
	create_proc();

    return 0;
}

void __exit hw_exit(void)
{
	DEBUG printk(KERN_INFO"virtio_fast_single: stopped\n");

	//Remove the proc file
	remove_proc_entry("virtio_fast_single", virt_fast->procdirectory);
	remove_proc_entry("virtio",NULL);

	//Reset PCI Device
	virt_fast->dev->config->del_vqs(virt_fast->dev);
	virt_fast->dev->config->reset(virt_fast->dev);

	//Unregister the driver
	unregister_virtio_driver(&virt_fast_driv);

	//kfree memory
	kfree(virt_fast);
}

MODULE_LICENSE("GPL");

module_init(hw_init);
module_exit(hw_exit);
