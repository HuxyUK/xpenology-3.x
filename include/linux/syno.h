#ifndef MY_ABC_HERE
#define MY_ABC_HERE
#endif

#ifndef __SYNO_H_
#define __SYNO_H_
#ifndef LINUX_VERSION_CODE
#include <linux/version.h>
#endif

#define SYNO_HAVE_KERNEL_VERSION(a,b,c) (LINUX_VERSION_CODE >= KERNEL_VERSION((a),(b),(c)) )
#define SYNO_HAVE_GCC_VERSION(a,b) (__GNUC__ > (a) || (__GNUC__ == (a) && __GNUC_MINOR__ >= (b)))
#define SYNO_HAVE_GLIBC_VERSION(a,b) ( __GLIBC__ > (a) || (__GLIBC__ == (a) && __GLIBC_MINOR__ >= (b)))
#if 0
#ifdef MY_DEF_HERE
#define SYNO_EVANSPORT_TTYS1_PORT	0x2F8
#define SYNO_EVANSPORT_SET8N1		0x3
#define SYNO_EVANSPORT_SHUTDOWN_CMD	0x31
#define SYNO_EVANSPORT_TXR		0
#define SYNO_EVANSPORT_LCR		3
#endif

#endif

#if 1
#define SYNO_USB_FLASH_DEVICE_INDEX 255
#define SYNO_USB_FLASH_DEVICE_NAME  "synoboot"
#define SYNO_USB_FLASH_DEVICE_PATH  "/dev/synoboot"
#if 1
#define IS_SYNO_USBBOOT_ID_VENDOR(VENDOR) (0xF400 == (VENDOR) || 0xF401 == (VENDOR))
#define IS_SYNO_USBBOOT_ID_PRODUCT(PRODUCT) (0xF400 == (PRODUCT) || 0xF401 == (PRODUCT))
#else
#define IS_SYNO_USBBOOT_ID_VENDOR(VENDOR) (0xF400 == (VENDOR))
#define IS_SYNO_USBBOOT_ID_PRODUCT(PRODUCT) (0xF400 == (PRODUCT))
#endif

#endif

#if 1
#define SYNO_SATA_DOM_VENDOR	"SATADOM "
#define SYNO_SATA_DOM_MODEL	"WD5002ABYS-01B1B0"
#define SYNO_DUALHEAD_SYSTEM_DEVICE_PATH  "/dev/synoboot4"
#endif

#if 1
#ifdef MY_ABC_HERE
#define SYNO_MAX_SWITCHABLE_NET_DEVICE 8
#define SYNO_NET_DEVICE_ENCODING_LENGTH 6
#endif

#endif

#ifdef MY_ABC_HERE
#define SYNO_YOTAWIMAX_DESC          "SYNO CDC Ethernet Device for YotaKey"
#define SYNO_YOTAWIMAX_ETHERNET_NAME "wm"
#define SYNO_YOTAWIMAX_NET_NOLINK_EVENT (0xffffffff)
#endif

#define USBCOPY_PORT_LOCATION 99
#ifdef MY_ABC_HERE
#define SDCOPY_PORT_LOCATION 98
#endif

#ifdef MY_ABC_HERE
#define CHECKINTERVAL (7UL*HZ)
#endif

#define SYNO_MD_CHUNK_SIZE 65536
#define SYNO_FIX_MD_RESIZE_BUSY_LOOP 5
#if	defined(MY_ABC_HERE) || defined(SYNO_BADSECTOR_TEST)
#if 1 && (1)
#define SYNO_MAX_INTERNAL_DISK 19
#else
#define SYNO_MAX_INTERNAL_DISK	15
#endif

#endif

#if 1 && defined(MY_ABC_HERE)
#define SYNO_SATA_PM_DEVICE_GPIO
#endif

#if defined(SYNO_SATA_PM_DEVICE_GPIO) && defined(MY_ABC_HERE)
#if 1
#define SYNO_MAX_SATA_ID 20
#define SYNO_PWRPIN_ENCODE_LEN 2
#define SYNO_PORT_SIGN 'N'
#define SYNO_PWRPIN_ITEM_LEN 1 + SYNO_PWRPIN_ENCODE_LEN*2
#endif

#endif

#if 0
#ifdef MY_ABC_HERE
#ifdef MY_DEF_HERE
#define SYNO_CREATE_TIME_SWAP_VERSION 3719
#endif

#endif

#endif

#ifdef MY_ABC_HERE
#if defined (F_CLEAR_ARCHIVE) || defined (F_SETSMB_ARCHIVE) || defined (F_SETSMB_HIDDEN) || \
	defined (F_SETSMB_SYSTEM) || defined (F_CLRSMB_ARCHIVE) || defined (F_CLRSMB_HIDDEN) || \
	defined (F_CLRSMB_SYSTEM) || defined (F_CLEAR_S3_ARCHIVE)
#error "Samba archive bit redefine."
#endif

#if 1
#if defined (F_CLRSMB_READONLY) || defined (F_SETSMB_READONLY) || \
	defined (F_CLRACL_INHERIT)  || defined (F_SETACL_INHERIT)  || \
	defined (F_CLRACL_OWNER_IS_GROUP) || defined (F_SETACL_OWNER_IS_GROUP)  || \
	defined (F_SETACL_SUPPORT) || defined (F_SETACL_SUPPORT)
#error "ACL archive bit redefine."
#endif

#endif

#define SYNO_FCNTL_BASE             513
#define F_CLEAR_ARCHIVE             (SYNO_FCNTL_BASE + 0)
#define F_SETSMB_ARCHIVE            (SYNO_FCNTL_BASE + 1)
#define F_SETSMB_HIDDEN             (SYNO_FCNTL_BASE + 2)
#define F_SETSMB_SYSTEM             (SYNO_FCNTL_BASE + 3)
#define F_CLRSMB_ARCHIVE            (SYNO_FCNTL_BASE + 4)
#define F_CLRSMB_HIDDEN             (SYNO_FCNTL_BASE + 5)
#define F_CLRSMB_SYSTEM             (SYNO_FCNTL_BASE + 6)
#define F_CLEAR_S3_ARCHIVE          (SYNO_FCNTL_BASE + 7)
#ifdef MY_ABC_HERE
#define F_CLRSMB_READONLY           (SYNO_FCNTL_BASE + 8)
#define F_SETSMB_READONLY           (SYNO_FCNTL_BASE + 9)
#define F_CLRACL_INHERIT            (SYNO_FCNTL_BASE + 10)
#define F_SETACL_INHERIT            (SYNO_FCNTL_BASE + 11)
#define F_CLRACL_HAS_ACL            (SYNO_FCNTL_BASE + 12)
#define F_SETACL_HAS_ACL            (SYNO_FCNTL_BASE + 13)
#define F_CLRACL_SUPPORT            (SYNO_FCNTL_BASE + 14)
#define F_SETACL_SUPPORT            (SYNO_FCNTL_BASE + 15)
#define F_CLRACL_OWNER_IS_GROUP     (SYNO_FCNTL_BASE + 16)
#define F_SETACL_OWNER_IS_GROUP     (SYNO_FCNTL_BASE + 17)
#define SYNO_FCNTL_LAST             F_SETACL_OWNER_IS_GROUP
#else
#define SYNO_FCNTL_LAST             F_CLEAR_S3_ARCHIVE
#endif

#else
#undef 1
#endif

#ifdef MY_ABC_HERE
#define SYNO_SMB_PSTRING_LEN 1024
#endif

#ifdef MY_ABC_HERE
#define MAX_CHANNEL_RETRY       2
#define CHANNEL_RETRY_INTERVAL  (3*HZ)
#endif

#include <linux/syno_user.h>
#include <linux/syno_debug.h>
#define SYNO_NFSD_WRITE_SIZE_MIN 131072
#ifdef MY_ABC_HERE
#define SYNO_NFSD_UDP_MAX_PACKET_SIZE 32768
#define SYNO_NFSD_UDP_MIN_PACKET_SIZE 4096
#define SYNO_NFSD_UDP_DEF_PACKET_SIZE 8192
#endif

#ifdef MY_ABC_HERE
#define SYNO_SATA_DEVICE_PREFIX	   "sd"
#define SYNO_ISCSI_DEVICE_PREFIX   "isd"
#define SYNO_ISCSI_DEVICE_INDEX    (26 + 25 * 26)    
#if 0
#define SYNO_INTERNAL_MICROSD_NAME "4-4"
#endif

#endif

#define SYNO_RTL8712_DEBUG_MSG 0
#define SYNO_RTL8192_DEBUG_MSG 0
#if 1
#if defined(MY_ABC_HERE)
#define SYNO_SAS_DISK_NAME
#endif

#if defined(SYNO_SAS_DISK_NAME)
#define SYNO_SAS_USB_DEVICE_PREFIX		"usb"
#define SYNO_SAS_DEVICE_PREFIX			"sas"
#ifdef MY_ABC_HERE
#define SYNO_SAS_ISCSI_DEVICE_PREFIX	"iscsi"
#endif

#endif

#endif

#if 1 && defined(MY_ABC_HERE)
#define SYNO_GLUSTER_FS
#endif

#endif

