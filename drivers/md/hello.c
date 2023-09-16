#include <linux/printk.h>
#include <linux/module.h>

static int __init hello_init(void)
{
	pr_info("hello md\n");
	return 0;
}
module_init(hello_init);
