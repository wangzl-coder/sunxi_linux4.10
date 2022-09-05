#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/input/touchscreen.h>
#include <linux/input/mt.h>
#include "ft5316_config.h"
#include <linux/of_device.h>



#define PRESS_MAX (255)

struct ft_chip_data{
    char *chip_name;
    uint8_t max_point;
    uint8_t reglen_per_point;
    uint8_t point_reg_off;
};

struct ft5316_slot_data{
    uint16_t point_x[FT5316_MT_POINT_MAX];
    uint16_t point_y[FT5316_MT_POINT_MAX];
    uint16_t touch_id[FT5316_MT_POINT_MAX];
    uint8_t touch_point;
};

struct ft5316_ts_dev{
    struct i2c_client *i2c_client;
    struct regulator *ctp_supply;
    dev_t devid;
    struct cdev chrdev;
    struct class *class;
    struct device *device;
    struct gpio_desc *reset_gpio;
    struct input_dev *input; 
    struct workqueue_struct *init_workqueue;
    struct touchscreen_properties ts_propties;
    struct ft5316_slot_data solt_data;
    const struct ft_chip_data *chip_data;
};

static struct i2c_client *this_client;

static void ft5316_init_function(struct work_struct *work);

static DECLARE_WORK(ft5316_init_work, ft5316_init_function);  /* init work */


static void ft5316_init_function(struct work_struct *work)
{
    int chip_id = 0;
    int ret = 0;
    int i = 0;

    while((chip_id == 0x00) || (chip_id == 0xa3)){
		msleep(5);
		ret = i2c_smbus_read_byte_data(this_client,0xA3);
        dev_info(&this_client->dev," addr: 0x%x, chip_id_value: 0x%x \r\n",this_client->addr,ret);
		if((ret != 0x00) && (ret != 0xa3)) {
			chip_id = ret;
			break;
		}
		if((i++)>10) {
			break;
		}	
	}
}



static int ft5316_chrdev_open(struct inode *pnode, struct file *pfile)
{
    return 0;
}

static long ft5316_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg)
{
    return 0;
}

static const struct file_operations ft5316_ops = {
    .open = ft5316_chrdev_open,
    .unlocked_ioctl = ft5316_ioctl,
};

static int ft5316_read_parse_touchdata(struct i2c_client *client, struct ft5316_slot_data *slot_data,
                                    struct touchscreen_properties *ts_prop, const struct ft_chip_data *chip_data)
{
    int ret = 0;
    struct i2c_msg msgs[2];
    char buf[32];
    uint8_t point ;
    uint8_t reg_offset;
    memset(buf, 0x0, 32*sizeof(char));
    msgs[0] = (struct i2c_msg) {
        .addr = client->addr,
        .flags = 0,
        .buf = buf,
        .len = 1
    };
    msgs[1] = (struct i2c_msg) {
        .addr = client->addr,
        .flags = I2C_M_RD,
        .buf = buf,
        .len = 31
    };
    ret = i2c_transfer(client->adapter, msgs, 2);
    if(ret < 0){
        dev_err(&client->dev, "ft5316: read touch data failed \r\n");
        return ret;
    }
    memset(slot_data, 0x0, sizeof(struct ft5316_slot_data));
    slot_data->touch_point = buf[2] & 0x07;         /** touch point count **/
    if(slot_data->touch_point > chip_data->max_point)
        return -1;
    reg_offset = chip_data->point_reg_off;
                        /** valid point parser **/
    for(point = 0; point < slot_data->touch_point; point++) {
        slot_data->point_x[point] = ((uint16_t) (buf[reg_offset] & 0x0F))<<8 | ((uint16_t) buf[reg_offset + 1]);
        slot_data->point_y[point] = ((uint16_t) (buf[reg_offset + 2] & 0x0F))<<8 | ((uint16_t) buf[reg_offset + 3]);
        slot_data->touch_id[point] = ((uint16_t) (buf[reg_offset + 2] & 0xF0)) >> 4;
        if(ts_prop->swap_x_y) {
            uint16_t temp = slot_data->point_x[point];
            slot_data->point_x[point] = slot_data->point_y[point];
            slot_data->point_y[point] = temp;
        }
        if(ts_prop->invert_x) {
            slot_data->point_x[point] = ts_prop->max_x - slot_data->point_x[point];
        }
        if(ts_prop->invert_y) {
            slot_data->point_y[point] = ts_prop->max_y - slot_data->point_y[point];
        }
        reg_offset += chip_data->reglen_per_point;
    }
    return ret;
}

static int ft5316_report_position(struct input_dev *input_dev, struct ft5316_slot_data *slot_data)
{
    uint8_t point;
    if(slot_data->touch_point == 0) {
        input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 0);
        input_sync(input_dev);
        return 0;
    }
    /** report valid point **/
    for(point = 0; point < slot_data->touch_point; point++) {
            /** valid point **/
            input_report_abs(input_dev, ABS_MT_TRACKING_ID, slot_data->touch_id[point]);
            input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 200);
            input_report_abs(input_dev, ABS_MT_POSITION_X, slot_data->point_x[point]);
            input_report_abs(input_dev, ABS_MT_POSITION_Y, slot_data->point_y[point]);
            input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, 30);
            input_report_abs(input_dev, ABS_PRESSURE, 200);
            input_mt_sync(input_dev);
    }
    input_sync(input_dev);
    return 0;
}

static irqreturn_t ft5316_ts_isr(int irq, void *dev_id)
{
    struct ft5316_ts_dev *ft5316_data;
    unsigned ret = 0;
    ft5316_data = (struct ft5316_ts_dev*) dev_id;
    

    ret = ft5316_read_parse_touchdata(ft5316_data->i2c_client,&ft5316_data->solt_data,
                             &ft5316_data->ts_propties, ft5316_data->chip_data);
    if(ret < 0)
        return IRQ_HANDLED;

    ft5316_report_position(ft5316_data->input, &ft5316_data->solt_data);
    return IRQ_HANDLED;
}

static int ft5316_input_open(struct input_dev *dev)
{
    dev_info(&dev->dev,"ft5x input dev open \r\n");

    return 0;
}

static void ft5316_input_close(struct input_dev *dev)
{
    dev_info(&dev->dev,"ft5x input dev open \r\n");

}

static int ft5316_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int error = 0;
    struct regulator *regulator;
    struct ft5316_ts_dev *ft5316_ts;
    int valtage;
    int major = 0;
    unsigned long irq_flag;
    struct input_dev *input_dev;
    const struct ft_chip_data *ft5316_chip;

    dev_dbg(&client->dev,"ft5316_ts probe begin \r\n");
    ft5316_ts = devm_kzalloc(&client->dev, sizeof(struct ft5316_ts_dev), GFP_KERNEL);
    if(!ft5316_ts){
        dev_err(&client->dev, "memory request for ft5316_ts failed \r\n");
        return -ENOMEM;
    }
    ft5316_chip = of_device_get_match_data(&client->dev);
    if(!ft5316_chip) {
        dev_err(&client->dev, "ft get chip data failed \r\n");
        return -EINVAL;
    }
    ft5316_ts->chip_data = ft5316_chip;
                /** for regulator supply **/
    regulator = devm_regulator_get(&client->dev, "vcc-ctp");
    if(IS_ERR(regulator)){
        dev_err(&client->dev, "cannot get vcc-ctp power supply \r\n");
        return -EINVAL;
    }
    if(!regulator_is_enabled(regulator)){
        error = regulator_enable(regulator);
    }
    if(error){
        dev_err(&client->dev, "failed to enable regulaor of ft5316 \r\n");
        return -EINVAL;
    }
    valtage = regulator_get_voltage(regulator);
    dev_info(&client->dev, "ts vcc-ctp enable , current valtage is %d mv \r\n",valtage/1000);

                /** for char dev **/
    if(major){
        ft5316_ts->devid = MKDEV(major, 0);
        error = register_chrdev_region(ft5316_ts->devid, 1, FT5316_DRV_NAME);
    } else {
        error = alloc_chrdev_region(&ft5316_ts->devid, 0, 1, FT5316_DRV_NAME);
    }
    if(error < 0) {
        dev_err(&client->dev, "ft5316 chrdev region failed \r\n");
        return -EINVAL;
    }
    cdev_init(&ft5316_ts->chrdev, &ft5316_ops);
    error = cdev_add(&ft5316_ts->chrdev, ft5316_ts->devid, 1);
    if(error < 0) {
        dev_err(&client->dev, "ft5316_ts cdev register failed \r\n");
        goto devid_del;
    }
    ft5316_ts->class = class_create(THIS_MODULE, FT5316_DRV_NAME);
    if(IS_ERR(ft5316_ts->class)) {
        dev_err(&client->dev, "ft5316_ts class register failed \r\n");
        goto chrdev_del;
    }
    ft5316_ts->device = device_create(ft5316_ts->class, NULL, ft5316_ts->devid, ft5316_ts, FT5316_DRV_NAME);
    if(IS_ERR(ft5316_ts->device)) {
        dev_err(&client->dev, "ft5316 device request failed \r\n");
        goto class_del;
    }

                /** power on **/
    ft5316_ts->reset_gpio = devm_gpiod_get_optional(&client->dev, "reset", GPIOD_OUT_LOW);
    if(IS_ERR(ft5316_ts->reset_gpio)) {
        error = PTR_ERR(ft5316_ts->reset_gpio);
        goto device_del;
    }
    
    usleep_range(10000,20000);      /* wait for power on 10-20 ms */
    gpiod_set_raw_value(ft5316_ts->reset_gpio, 1);
    msleep(80);                     /* wait for reset */

            /**** for init  ****/
    ft5316_ts->init_workqueue = create_singlethread_workqueue("ft5316_init");
    if(!ft5316_ts->init_workqueue){
        dev_err(&client->dev, "ft5316 init work create failed \r\n");
        goto device_del;
    }
    this_client = client;
    queue_work(ft5316_ts->init_workqueue, &ft5316_init_work);

            /** for input device **/
    input_dev = devm_input_allocate_device(&client->dev);
    if(IS_ERR(input_dev)){
        dev_err(&client->dev, "ft5316 alloc input failed \r\n");
        goto workqueue_destroy;
    }
    input_dev->name = client->name;
    input_dev->open = ft5316_input_open;
    input_dev->close = ft5316_input_close;
    input_dev->id.bustype = BUS_I2C;
    input_dev->dev.parent = &client->dev;

                                 /** multi touch **/
    __set_bit(EV_ABS, input_dev->evbit);
    __set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
    __set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);	
    __set_bit(BTN_TOUCH, input_dev->keybit);
    __set_bit(EV_KEY, input_dev->evbit);
    input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, ft5316_ts->ts_propties.max_x, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, ft5316_ts->ts_propties.max_y, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, ft5316_chip->max_point - 1, 0, 0);
    touchscreen_parse_properties(input_dev, true, &ft5316_ts->ts_propties);
    printk("ft5316_ts->ts_propties.max_x = %d , ft5316_ts->ts_propties.max_y = %d \r\n",
                                 ft5316_ts->ts_propties.max_x, ft5316_ts->ts_propties.max_y);
    error = input_register_device(input_dev);
    if(error){
        dev_err(&client->dev,"input device register failed \r\n");
        goto workqueue_destroy;
    }
    
            /** for irq **/
    irq_flag = irq_get_trigger_type(client->irq);
    irq_flag |= IRQF_ONESHOT;
    error = devm_request_threaded_irq(&client->dev, client->irq, NULL, ft5316_ts_isr, irq_flag, client->name, ft5316_ts);
    if(error < 0){
        dev_err(&client->dev, "failed to request irq %d , error %d \r\n",client->irq, error);
        goto free_input_device;
    }

    ft5316_ts->input = input_dev;
    ft5316_ts->ctp_supply = regulator;
    ft5316_ts->i2c_client = client;
    i2c_set_clientdata(client, ft5316_ts);
    
    dev_info(&client->dev, "device ft5316_ts ts screen register success \r\n");
    
    return 0;


free_input_device:
    input_unregister_device(input_dev);
workqueue_destroy:
    cancel_work_sync(&ft5316_init_work);
	destroy_workqueue(ft5316_ts->init_workqueue);	
device_del:
    device_destroy(ft5316_ts->class, ft5316_ts->devid);
class_del:
    class_destroy(ft5316_ts->class);
chrdev_del:
    cdev_del(&ft5316_ts->chrdev);
devid_del:
    unregister_chrdev_region(ft5316_ts->devid,1);
    return error;
}

static int ft5316_remove(struct i2c_client *client)
{
    int error = 0;
    struct ft5316_ts_dev *ft5316_data;
    ft5316_data = (struct ft5316_ts_dev*) i2c_get_clientdata(client);
    destroy_workqueue(ft5316_data->init_workqueue);
    input_unregister_device(ft5316_data->input);
    device_destroy(ft5316_data->class, ft5316_data->devid);
    class_destroy(ft5316_data->class);
    cdev_del(&ft5316_data->chrdev);
    unregister_chrdev_region(ft5316_data->devid,1);
    return error;
}

static struct ft_chip_data ft5316_chip_data = {
    .chip_name = "ft5316_ts",
    .max_point = 5,
    .point_reg_off = 3,
    .reglen_per_point = 6
};

static struct of_device_id edt_ft5316_of_match[] = {
    { .compatible = "edt,edt-ft5316", .data = &ft5316_chip_data},
    { }
};

struct i2c_driver ft5316_driver = {
    .probe = ft5316_probe,
    .remove = ft5316_remove,
    .driver = {
        .name = FT5316_DRV_NAME,
        .of_match_table = of_match_ptr(edt_ft5316_of_match)
    }
};


module_i2c_driver(ft5316_driver);
MODULE_LICENSE("GPL");