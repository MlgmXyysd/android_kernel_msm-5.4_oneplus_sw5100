#include <linux/interconnect.h>
#include <linux/platform_device.h>
#include <soc/qcom/rpm-smd.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/errno.h>


#define MAX_MSG_BUFFER 350
#define MAX_KEY_VALUE_PAIRS 20

static struct class *ddrfreq_class;
struct platform_device *icc_pdev;

static u32 src_port;
static u32 dst_port;
static u32 avg_bw;
static u32 peak_bw;

static u32 string_to_uint(const u8 *str)
{
	int i, len;
	u32 output = 0;

	len = strnlen(str, sizeof(u32));
	for (i = 0; i < len; i++)
		output |= str[i] << (i * 8);

	return output;
}

static ssize_t
ddr_freq_show(struct class *class, struct class_attribute *attr, char *buf)
{
    return 0;
}

static ssize_t
ddr_freq_store(struct class *class, struct class_attribute *attr,
        const char *user_buffer, size_t count)
{
    struct icc_path *path;
    char buf[MAX_MSG_BUFFER], rsc_type_str[6] = {}, rpm_set[8] = {},
                                                key_str[6] = {};
    int i, pos = -1, set = -1, nelems = -1;
    char *cmp;
    uint32_t rsc_type = 0, rsc_id = 0, key = 0, data = 0;
    struct msm_rpm_request *req;
    int ret = 0;

    count = min(count, sizeof(buf) - 1);
    
    ret = snprintf(buf, count, user_buffer);
    if (ret <= 0){
        pr_err("ddr_freq_store copy_from_user err:%d  \n",ret);
        return -EFAULT;
    }
    buf[count] = '\0';
    pr_err("ddr_freq_store  count:%d %s\n",count,buf);
    cmp = strstrip(buf);
    pr_err("ddr_freq_store  count:%d %s\n",count,buf);

    if (sscanf(cmp, "%7s %5s %u %d %n", rpm_set, rsc_type_str,
                &rsc_id, &nelems, &pos) != 4) {
        pr_err("Invalid number of arguments passed\n");
        goto err;
    }

    if (strlen(rpm_set) > 6 || strlen(rsc_type_str) > 4) {
        pr_err("Invalid value of set or resource type\n");
        goto err;
    }

    if (!strcmp(rpm_set, "active"))
        set = 0;
    else if (!strcmp(rpm_set, "sleep"))
        set = 1;

    rsc_type = string_to_uint(rsc_type_str);

    if (set < 0 || nelems < 0) {
        pr_err("Invalid value of set or nelems\n");
        goto err;
    }
    if (nelems > MAX_KEY_VALUE_PAIRS) {
        pr_err("Exceeded max no of key-value entries\n");
        goto err;
    }

    
    src_port = 1;
    dst_port = 512;
    avg_bw = 0;
    peak_bw = 8367636;
   
    path = icc_get(&icc_pdev->dev, src_port, dst_port);
    if (IS_ERR(path)){
        pr_err("ddr_freq_store icc_get err \n");
 		return -1;
    }

    ret = icc_set_bw(path, avg_bw, peak_bw);
  	if (ret){
        pr_err("ddr_freq_store icc_set_bw err \n");
  		return -1;
    }

    req = msm_rpm_create_request(set, rsc_type, rsc_id, nelems);
    if (!req){
        pr_err("ddr_freq_store msm_rpm_create_request  err \n");
        return -ENOMEM;
    }

    for (i = 0; i < nelems; i++) {
        cmp += pos;
        if (sscanf(cmp, "%5s %n", key_str, &pos) != 1) {
            pr_err("Invalid number of arguments passed\n");
            goto err_request;
        }

        if (strlen(key_str) > 4) {
            pr_err("Key value cannot be more than 4 charecters\n");
            goto err_request;
        }
        key = string_to_uint(key_str);
        pr_err("Key values key:%u \n",key);
        if (!key) {
            pr_err("Key values entered incorrectly\n");
            goto err_request;
        }

        cmp += pos;
        if (sscanf(cmp, "%u %n", &data, &pos) != 1) {
            pr_err("Invalid number of arguments passed\n");
        }
        pr_err("Key values data:%u \n",data);
        if (msm_rpm_add_kvp_data(req, key,(void *)&data, sizeof(data)))
             goto err_request;
        }
        
        if (msm_rpm_wait_for_ack(msm_rpm_send_request(req)))
                pr_err("Sending the RPM message failed\n");

err_request:
        pr_err("ddr_freq_store err_request  err \n");
        msm_rpm_free_request(req);
err:
        return count;
}

static const struct class_attribute attributes[] = {
    __ATTR_RW(ddr_freq),
};

static int __init ddrfreq_init(void)
{
    int status = 0;
    int i;

    icc_pdev = platform_device_alloc("icc-test", PLATFORM_DEVID_AUTO);
    platform_device_add(icc_pdev);

    ddrfreq_class = class_create(THIS_MODULE, "ddrfreq");
    if (IS_ERR(ddrfreq_class)){
        status = PTR_ERR(ddrfreq_class);
         pr_err("ddrfreq_init class create err:%d\n",status);
         return 0;
    }

    for (i = 0; i < ARRAY_SIZE(attributes); i++){
        if (class_create_file(ddrfreq_class, attributes + i)){
            pr_err("ddrfreq_init class file create \n");
            return 0;
        }
    }

    if (status < 0)
        pr_err("ddrfreq_init status");
    return 0;
}

module_init(ddrfreq_init);

static void __exit ddrfreq_exit(void)
{
    return;
}
module_exit(ddrfreq_exit);

MODULE_AUTHOR("oplus");
MODULE_DESCRIPTION("OPLUS ddr freq driver");
MODULE_LICENSE("GPL");
