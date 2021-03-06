/*
 * drivers/lsi/acp/ubootenv.c
 *
 * Copyright (C) 2009 LSI
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  021.1.1_pre.17  USA
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/mtd/mtd.h>
#include <linux/crc32.h>
#include <linux/io.h>

/*
  ======================================================================
  Data types and Macros
  ======================================================================
*/
#include <asm/lsi/debug.h>

/*
  ======================================================================
  Global Variables
  ======================================================================
*/

static struct kobject *ubootenv_kobj;


static unsigned long uboot_env_size = (128 * 1024);
static unsigned long uboot_env_cs_size = (128 * 1024);
static int uboot_env_current = -1;

#define ENVIRONMENT_DATA_SIZE(size) (size - (2 * sizeof(unsigned long)))

typedef struct environment {

	unsigned long crc32;
	unsigned long flags;
	unsigned char data[];

} __packed environment_t;

static environment_t *environment = (environment_t *) 0;

static unsigned long crc32_lut[256] = {

	/*   0 -- */           0u, 1996959894u, 3993919788u, 2567524794u,
	/*   4 -- */   124634137u, 1886057615u, 3915621685u, 2657392035u,
	/*   8 -- */   249268274u, 2044508324u, 3772115230u, 2547177864u,
	/*  12 -- */   162941995u, 2125561021u, 3887607047u, 2428444049u,
	/*  16 -- */   498536548u, 1789927666u, 4089016648u, 2227061214u,
	/*  20 -- */   450548861u, 1843258603u, 4107580753u, 2211677639u,
	/*  24 -- */   325883990u, 1684777152u, 4251122042u, 2321926636u,
	/*  28 -- */   335633487u, 1661365465u, 4195302755u, 2366115317u,
	/*  32 -- */   997073096u, 1281953886u, 3579855332u, 2724688242u,
	/*  36 -- */  1006888145u, 1258607687u, 3524101629u, 2768942443u,
	/*  40 -- */   901097722u, 1119000684u, 3686517206u, 2898065728u,
	/*  44 -- */   853044451u, 1172266101u, 3705015759u, 2882616665u,
	/*  48 -- */   651767980u, 1373503546u, 3369554304u, 3218104598u,
	/*  52 -- */   565507253u, 1454621731u, 3485111705u, 3099436303u,
	/*  56 -- */   671266974u, 1594198024u, 3322730930u, 2970347812u,
	/*  60 -- */   795835527u, 1483230225u, 3244367275u, 3060149565u,
	/*  64 -- */  1994146192u,   31158534u, 2563907772u, 4023717930u,
	/*  68 -- */  1907459465u,  112637215u, 2680153253u, 3904427059u,
	/*  72 -- */  2013776290u,  251722036u, 2517215374u, 3775830040u,
	/*  76 -- */  2137656763u,  141376813u, 2439277719u, 3865271297u,
	/*  80 -- */  1802195444u,  476864866u, 2238001368u, 4066508878u,
	/*  84 -- */  1812370925u,  453092731u, 2181625025u, 4111451223u,
	/*  88 -- */  1706088902u,  314042704u, 2344532202u, 4240017532u,
	/*  92 -- */  1658658271u,  366619977u, 2362670323u, 4224994405u,
	/*  96 -- */  1303535960u,  984961486u, 2747007092u, 3569037538u,
	/* 100 -- */  1256170817u, 1037604311u, 2765210733u, 3554079995u,
	/* 104 -- */  1131014506u,  879679996u, 2909243462u, 3663771856u,
	/* 108 -- */  1141124467u,  855842277u, 2852801631u, 3708648649u,
	/* 112 -- */  1342533948u,  654459306u, 3188396048u, 3373015174u,
	/* 116 -- */  1466479909u,  544179635u, 3110523913u, 3462522015u,
	/* 120 -- */  1591671054u,  702138776u, 2966460450u, 3352799412u,
	/* 124 -- */  1504918807u,  783551873u, 3082640443u, 3233442989u,
	/* 128 -- */  3988292384u, 2596254646u,   62317068u, 1957810842u,
	/* 132 -- */  3939845945u, 2647816111u,   81470997u, 1943803523u,
	/* 136 -- */  3814918930u, 2489596804u,  225274430u, 2053790376u,
	/* 140 -- */  3826175755u, 2466906013u,  167816743u, 2097651377u,
	/* 144 -- */  4027552580u, 2265490386u,  503444072u, 1762050814u,
	/* 148 -- */  4150417245u, 2154129355u,  426522225u, 1852507879u,
	/* 152 -- */  4275313526u, 2312317920u,  282753626u, 1742555852u,
	/* 156 -- */  4189708143u, 2394877945u,  397917763u, 1622183637u,
	/* 160 -- */  3604390888u, 2714866558u,  953729732u, 1340076626u,
	/* 164 -- */  3518719985u, 2797360999u, 1068828381u, 1219638859u,
	/* 168 -- */  3624741850u, 2936675148u,  906185462u, 1090812512u,
	/* 172 -- */  3747672003u, 2825379669u,  829329135u, 1181335161u,
	/* 176 -- */  3412177804u, 3160834842u,  628085408u, 1382605366u,
	/* 180 -- */  3423369109u, 3138078467u,  570562233u, 1426400815u,
	/* 184 -- */  3317316542u, 2998733608u,  733239954u, 1555261956u,
	/* 188 -- */  3268935591u, 3050360625u,  752459403u, 1541320221u,
	/* 192 -- */  2607071920u, 3965973030u, 1969922972u,   40735498u,
	/* 196 -- */  2617837225u, 3943577151u, 1913087877u,   83908371u,
	/* 200 -- */  2512341634u, 3803740692u, 2075208622u,  213261112u,
	/* 204 -- */  2463272603u, 3855990285u, 2094854071u,  198958881u,
	/* 208 -- */  2262029012u, 4057260610u, 1759359992u,  534414190u,
	/* 212 -- */  2176718541u, 4139329115u, 1873836001u,  414664567u,
	/* 216 -- */  2282248934u, 4279200368u, 1711684554u,  285281116u,
	/* 220 -- */  2405801727u, 4167216745u, 1634467795u,  376229701u,
	/* 224 -- */  2685067896u, 3608007406u, 1308918612u,  956543938u,
	/* 228 -- */  2808555105u, 3495958263u, 1231636301u, 1047427035u,
	/* 232 -- */  2932959818u, 3654703836u, 1088359270u,  936918000u,
	/* 236 -- */  2847714899u, 3736837829u, 1202900863u,  817233897u,
	/* 240 -- */  3183342108u, 3401237130u, 1404277552u,  615818150u,
	/* 244 -- */  3134207493u, 3453421203u, 1423857449u,  601450431u,
	/* 248 -- */  3009837614u, 3294710456u, 1567103746u,  711928724u,
	/* 252 -- */  3020668471u, 3272380065u, 1510334235u,  755167117u

};

/*
  ======================================================================
  Prototypes
  ======================================================================
*/

static unsigned long ubootenv_crc32(unsigned char *, unsigned long);
static int ubootenv_initialize(void);
static void ubootenv_finalize(void);
static int ubootenv_read(struct mtd_info *, size_t, void *);
static void create_env_sysfs(void);
/*
  ======================================================================
  ======================================================================
*/

/*
  ----------------------------------------------------------------------
  ubootenv_crc32
*/

static unsigned long
ubootenv_crc32(unsigned char *start, unsigned long size)
{
	unsigned long crc = (unsigned long)0xffffffff, index;

	DEBUG_PRINT("start=0x%lx size=0x%lx\n", (unsigned long)start, size);

	for (index = 0; index < size; index++) {
		unsigned long temp = (crc ^ *(start++)) & 0x000000ff;
		crc = ((crc >> 8) & 0x00ffffff) ^ crc32_lut[temp];
	}
	return ~crc;
}

/*
  ----------------------------------------------------------------------
  ubootenv_read
*/

static int
ubootenv_read(struct mtd_info *mtd, size_t size, void *buffer)
{
	int read = 0;
	loff_t offset = 0;

	DEBUG_PRINT("size=0x%x mtd->erasesize=0x%x mtd->size=0x%llx\n",
		    size, mtd->erasesize, mtd->size);

	if (0 != size % mtd->erasesize) {
		ERROR_PRINT("size=%u/%llu is not a multiple of erasesize=%u\n",
			    size, mtd->size, mtd->erasesize);
		return -1;
	}

	if (size > mtd->size) {
		ERROR_PRINT("size=%llu can't contain size=%u\n",
			    mtd->size, size);
		return -1;
	}

	while ((read < size) && (offset < mtd->size)) {
		int return_code;
		size_t bytes_read;

		DEBUG_PRINT("read=0x%x size=0x%x offset=0x%llx\n",
			    read, size, offset);

		if (0 != mtd_block_isbad(mtd, offset)) {
			offset += mtd->erasesize;
			continue;
		}

		return_code = mtd_read(mtd, offset, mtd->erasesize,
				       &bytes_read, (u_char *) buffer);

		if (mtd->erasesize != bytes_read) {
			ERROR_PRINT("Error Reading Environment!\n");
			return -1;
		}

		offset += mtd->erasesize;
		read += mtd->erasesize;
		buffer += mtd->erasesize;
	}

	return 0;
}

/*
  ----------------------------------------------------------------------
  ubootenv_initialize
*/

static int
ubootenv_initialize(void)
{
	environment_t *env0;
	environment_t *env1;
	unsigned long crc32_env0;
	unsigned long crc32_env1;
	struct mtd_info *mtd_env0;
	struct mtd_info *mtd_env1;

	DEBUG_PRINT("Getting MTD Devices.\n");

	mtd_env0 = get_mtd_device_nm("env-0");
	if ((struct mtd_info *)-ENODEV == mtd_env0) {
		ERROR_PRINT(" --> Couldn't get MTD device by name!\n");
		return -1;
	}

	mtd_env1 = get_mtd_device_nm("env-1");
	if ((struct mtd_info *)-ENODEV == mtd_env1) {
		ERROR_PRINT(" --> Couldn't get MTD device by name!\n");
		return -1;
	}

	/*
	 * If the erasesize is larger than the size of the environment,
	 * change the environment size (so reading and writing will
	 * work as expected) but use the original environment size to
	 * calculate the checksum.
	 */

	if (mtd_env0->erasesize > uboot_env_size)
		uboot_env_size = mtd_env0->erasesize;

	DEBUG_PRINT("Allocating Environment Buffers.\n");

	env0 = vmalloc(uboot_env_size);
	if ((environment_t *)0 == env0) {
		ERROR_PRINT("Unable to allocate %lu bytes\n", uboot_env_size);
		return -1;
	}

	env1 = vmalloc(uboot_env_size);
	if ((environment_t *) 0 == env1) {
		ERROR_PRINT("Unable to allocate %lu bytes\n", uboot_env_size);
		vfree((void *) env0);
		return -1;
	}

	DEBUG_PRINT("Reading Environments.\n");

	if (0 != ubootenv_read(mtd_env0, uboot_env_size, env0))
		return -1;

	if (0 != ubootenv_read(mtd_env1, uboot_env_size, env1))
		return -1;

	DEBUG_PRINT("Calculating CRC values.\n");
	crc32_env0 = ubootenv_crc32((unsigned char *)env0->data,
				    ENVIRONMENT_DATA_SIZE(uboot_env_cs_size));
	crc32_env1 = ubootenv_crc32((unsigned char *)env1->data,
				    ENVIRONMENT_DATA_SIZE(uboot_env_cs_size));
	DEBUG_PRINT("crc32_env0=0x%lx env0->crc32=0x%lx\n",
		    crc32_env0, env0->crc32);
	DEBUG_PRINT("crc32_env2=0x%lx env1->crc32=0x%lx\n",
		     crc32_env1, env1->crc32);
	DEBUG_PRINT("Picking a Copy.\n");

	if ((crc32_env0 == env0->crc32) &&
	    (crc32_env1 != env1->crc32)) {
		/* Use env0 */
		DEBUG_PRINT("Using Copy 0.\n");
		uboot_env_current = 0;
		vfree((void *) env1);
		environment = env0;
	} else if ((crc32_env0 != env0->crc32) &&
		   (crc32_env1 == env1->crc32)) {
		/* Use env1 */
		DEBUG_PRINT("Using Copy 1.\n");
		uboot_env_current = 1;
		vfree((void *) env0);
		environment = env1;
	} else if ((crc32_env0 != env0->crc32) &&
		   (crc32_env1 != env1->crc32)) {
		/* No Environment Available */
		uboot_env_current = -1;
		vfree((void *) env0);
		vfree((void *) env1);
		ERROR_PRINT("Bad CRCs: No Valid U-Boot Environment Found!\n");
		return -1;
	} else if (env0->flags > env1->flags) {
		/* Use env0 */
		DEBUG_PRINT("Using Copy 0.\n");
		uboot_env_current = 0;
		vfree((void *) env1);
		environment = env0;
	} else if (env0->flags < env1->flags) {
		/* Use env1 */
		DEBUG_PRINT("Using Copy 1.\n");
		uboot_env_current = 1;
		vfree((void *) env0);
		environment = env1;
	} else if (env0->flags == env1->flags) {
		/* Use Either */
		DEBUG_PRINT("Using Copy 0.\n");
		uboot_env_current = 0;
		vfree((void *) env1);
		environment = env0;
	} else {
		/* No Environment Available */
		uboot_env_current = -1;
		vfree((void *) env0);
		vfree((void *) env1);
		ERROR_PRINT("Bad Flags: No Valid U-Boot Environment Found!\n");
		return -1;
	}

	DEBUG_PRINT("Done...\n");
	create_env_sysfs();

	return 0;
}

/*
  ----------------------------------------------------------------------
  ubootenv_finalize
*/

static void
ubootenv_finalize(void)
{
	DEBUG_PRINT("Freeing the environment.\n");

	if ((void *)0 != environment)
		vfree((void *)environment);

	environment = (environment_t *)0;
}

/*
  ======================================================================
  Public Interface
  ======================================================================
*/

/*
  ----------------------------------------------------------------------
  ubootenv_get
*/

int
ubootenv_get(const char *key, char *value)
{
	int return_code = -1;
	char *string;

	if (NULL == environment) {
		ERROR_PRINT("Environment Isn't Available!\n");
		return -1;
	}

	string = environment->data;

	while (0x00 != string[0]) {
		if (0 == strncmp(key, string, strlen(key))) {
			char *value_ = strchr(string, '=');
			++value_;
			strcpy(value, value_);
			return_code = 0;
			break;
		}

		string += (strlen(string) + 1);
	}

	return return_code;
}
EXPORT_SYMBOL(ubootenv_get);

/*
  ======================================================================
  ======================================================================
  Sysfs Stuff
  ======================================================================
  ======================================================================
*/

struct bin_attribute uboot_env[100];

static ssize_t read_env(struct file *filep, struct kobject *kobj,
			struct bin_attribute *bin_attr,
			char *buf, loff_t off, size_t size)
{
	int retsize = -1;
	char *string;

	if (size <= 1)
		return 0;

	if (NULL == environment) {
		ERROR_PRINT("Environment Isn't Available!\n");
		return -1;
	}

	string = environment->data;

	while (0x00 != string[0]) {
		if (0 == strncmp(bin_attr->attr.name,
				 string, strlen(bin_attr->attr.name))) {
			char *value_ = strchr(string, '=');
			++value_;

			retsize = 1 + strlcpy(buf, value_, size);
			break;
		}

		string += (strlen(string) + 1);
	}

	return retsize;
}

static void create_env_sysfs(void)
{
	char *string;
	int i = 0;

	if (NULL == environment) {
		ERROR_PRINT("Environment Isn't Available!\n");
		return;
	}

	string = environment->data;

	while (0x00 != string[0]) {
		char *value_ = strchr(string, '=');
		char *name = vmalloc(1 + value_ - string);
		strlcpy(name, string, 1 + value_ - string);

		uboot_env[i].attr.name = name;
		uboot_env[i].attr.mode = 0400;
		uboot_env[i].size = strlen(string) - (value_ - string) ;
		uboot_env[i].read = read_env;

		if (sysfs_create_bin_file(ubootenv_kobj, &uboot_env[i]))
			ERROR_PRINT("unable to add uboot-env sysfs file\n");

		string += (strlen(string) + 1);
		i++;
		if (i > 99) {
			ERROR_PRINT("More than 100 uboot env variables.\n");
			return;
		}
	}
}


static ssize_t list_env(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	int i = 1;
	while (i < uboot_env_size - 2) {
		if (environment->data[i-1] == 0)
			if (environment->data[i] == 0)
				if (environment->data[i+1] == 0)
					break;
		i++;
	}
	memcpy(buf, environment->data, i);
	return i;
}

static DEVICE_ATTR(list_env, 0444, list_env, NULL);

static struct attribute *attrs[] = {
	&dev_attr_list_env.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};


/*
  ======================================================================
  ======================================================================
  Linux Module Stuff
  ======================================================================
  ======================================================================
*/

/*
  ----------------------------------------------------------------------
  ubootenv_module_init
*/

int __init
ubootenv_module_init(void)
{
	int retval;

	DEBUG_PRINT("\n");
	ubootenv_kobj = kobject_create_and_add("uboot-env", kernel_kobj);
	if (!ubootenv_kobj)
		return -ENOMEM;

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(ubootenv_kobj, &attr_group);
	if (retval)
		goto fail;

	retval = ubootenv_initialize();
	if (retval)
		goto fail;

	return 0;
fail:
	kobject_put(ubootenv_kobj);

	return retval;
}

module_init(ubootenv_module_init);

/*
  ----------------------------------------------------------------------
  ubootenv_module_exit
*/

void __exit
ubootenv_module_exit(void)
{
	DEBUG_PRINT("\n");
	kobject_put(ubootenv_kobj);
	ubootenv_finalize();
	return;
}

module_exit(ubootenv_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("John Jacques <john.jacques@lsi.com>");
MODULE_DESCRIPTION("Read Access of the U-Boot Environment");
