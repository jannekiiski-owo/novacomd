/* @@@LICENSE
*
*      Copyright (c) 2008-2012 Hewlett-Packard Development Company, L.P.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
* LICENSE@@@ */

#include <stdio.h>
#include <usb.h>
#include <stdint.h>
#include <sys/time.h>
#include <errno.h>
#include <time.h>
#include <assert.h>
#include <sys/ioctl.h>
#include <string.h>
#include <ctype.h>

#include <transport_usb.h>
#include <platform.h>
#include <novacom.h>
#include <debug.h>
#include <log.h>
#include <sys/queue.h>
#include <sys/inotify.h>

#include <linux/usbdevice_fs.h>
#include "../novacom/mux.h"
#include "device_list.h"

/* debug */
#define LOCAL_TRACE 0
#define USB_SCAN_TRACE 0
#define USB_RECOVERY 0

/* controls */
#define TRACE_ZERO_LEN_PACKETS 0
#define FAULTY_TX 0
#define MAX_MTU 16384

#define USBDEVFS_IOCTL_TIMEOUT  2000

struct myusb_dev_handle {
	int fd;

	// other stuff here
};

typedef struct {
	union {
		usb_dev_handle *handle;

		/* XXX cheezy hack to let us get to the file descriptor hidden in the libusb handle */
		struct myusb_dev_handle *myhandle;
	};

	novacom_usbll_handle_t usbll_handle;

	bool shutdown;
	platform_event_t tx_startup_event;		/* event to block tx thread until any packet received on rx side*/
	int tx_startup_wait;					/* flag to indicate that we are blocked on tx */
	platform_event_t tx_shutdown_event;		/* event to indicate tx thread shutdown */

	int rxep;
	int txep;
	int rx_timeout;
	int tx_timeout;
	const char *devtype;
	int busnum;
	int devnum;
	int iface;
} novacom_usb_handle_t;

typedef struct recovery_entry_s {
	transport_recovery_token_t	*t_token;		/* transport recovery token */
	int timeout;								/* timout value */

	TAILQ_ENTRY(recovery_entry_s) entries;		/* holds pointers to prev, next entries */
} recovery_entry_t;

/* vars */
static platform_thread_t findandattach_thread;
volatile int novacom_shutdown = 0;
		/* list of recovery tokens */
TAILQ_HEAD(recovery_queue_s, recovery_entry_s)  t_recovery_queue;
static platform_mutex_t recovery_lock;


/* find_endpoints */
static int novacom_usb_find_endpoints(usb_dev_handle *handle, int eps[2], int *iface)
{
	int i;
	int rc;
	struct usb_device *dev = usb_device(handle);

#if USB_SCAN_TRACE
	log_printf(LOG_SPEW, "find novacom endpoints: handle %p device %p\n", handle, dev);
#endif

	if(dev == NULL || dev->config == NULL) {
		return -1;
	}
	struct usb_interface *interface = dev->config->interface;

	for (i = 0; i < dev->config->bNumInterfaces; i++) {
#if USB_SCAN_TRACE
		log_printf(LOG_SPEW, "interfacenum %d\n", i);
#endif

		int a;
		for (a = 0; a < interface->num_altsetting; a++) {

#if USB_SCAN_TRACE
			log_printf(LOG_SPEW, "altsetting %d\n", a);

			log_printf(LOG_SPEW, "class %d subclass %d protocol %d endpoints %d\n",
				interface[i].altsetting[a].bInterfaceClass,
				interface[i].altsetting[a].bInterfaceSubClass,
				interface[i].altsetting[a].bInterfaceProtocol,
				interface[i].altsetting[a].bNumEndpoints);
#endif

			// see if it's a blank interface with two or three bulk endpoints, probably us
			//
			// XXX be smarter about it
			if (interface[i].altsetting[a].bInterfaceClass == USB_CLASS_VENDOR_SPEC &&
				// match against our subclass/protocol id
				(interface[i].altsetting[a].bInterfaceSubClass == 0x47 &&
				interface[i].altsetting[a].bInterfaceProtocol == 0x11) ) {

				// match the two bulk endpoints we care about
				eps[0] = eps[1] = 0;
				if (interface[i].altsetting[a].endpoint[0].bEndpointAddress & 0x80)
					eps[0] = interface[i].altsetting[a].endpoint[0].bEndpointAddress;
				if ((interface[i].altsetting[a].endpoint[1].bEndpointAddress & 0x80) == 0)
					eps[1] = interface[i].altsetting[a].endpoint[1].bEndpointAddress;

				if (eps[0] == 0 || eps[1] == 0) {
					log_printf(LOG_ERROR, "failed to find acceptable endpoints\n");
					continue;
				}

#if 0
				// set the config
				rc = usb_set_configuration(handle, dev->config->bConfigurationValue);
				if (rc) {
					log_printf(LOG_ERROR, "failed to set config %d\n", dev->config->bConfigurationValue);
					return -1;
				}
#endif

				// claim this interface
				rc = usb_claim_interface(handle, i);
				if (rc) {
					//log_printf(LOG_ERROR, "failed to claim interface %d, errno %d\n", i, errno);
					return -1;
				} else if(iface) {
					*iface = i;
				}

#if 0
				// set the alternate interface
				if (a != 0) {
					rc = usb_set_altinterface(handle, a);
					if (rc) {
						log_printf(LOG_ERROR, "failed to set altinterface %d\n", a);
						return -1;
					}
				}
#endif
			
				return 0;
			}
		}
	}

	return -1;
}

static novacom_usb_handle_t *novacom_usb_open(const char *busname, const char *devname)
{
	int rc;

//	usb_set_debug(100);

	rc = usb_find_busses();
	if (rc < 0)
		return NULL;

	rc = usb_find_devices();
	if (rc < 0)
		return NULL;

	struct usb_bus *bus;
	struct usb_device *dev;

	for (bus = usb_get_busses(); bus; bus = bus->next) {
		for (dev = bus->devices; dev; dev = dev->next) {
#if USB_SCAN_TRACE
			log_printf(LOG_SPEW, "looking at dev %d (%s) %04x:%04x\n", 
					dev->devnum, dev->filename, 
					dev->descriptor.idVendor, dev->descriptor.idProduct);
#endif
			/* ignore all other devices if specific bus/device given */
			if (busname != NULL && devname != NULL) {
				if (strcmp(bus->dirname, busname) != 0 || strcmp(dev->filename, devname) != 0) {
					continue;
				}
			}

			/* try to match against our list of vendor/products */
			uint i;
			for (i=0; usbid_list[i].name; i++) {
				if ((dev->descriptor.idVendor == usbid_list[i].vendor) && (dev->descriptor.idProduct == usbid_list[i].product)) {
					usb_dev_handle *handle = usb_open(dev);
					if (!handle) continue;

#if USB_SCAN_TRACE
					log_printf(LOG_SPEW, "opened usb handle: fd %d\n", ((struct myusb_dev_handle *)handle)->fd);
#endif

					int eps[2];
					int iface = -1;
					rc = novacom_usb_find_endpoints(handle, eps, &iface);
					if (rc != 0) {
						usb_close(handle);
						continue;
					}

					LTRACEF("got endpoints: %x, %x\n", eps[0], eps[1]);
					novacom_usb_handle_t *usb_handle = platform_calloc(sizeof(novacom_usb_handle_t));
					if(!usb_handle) {
						return NULL;
					}
					usb_handle->handle = handle;
					usb_handle->rxep = eps[0];
					usb_handle->txep = eps[1];
					usb_handle->devtype = usbid_list[i].name;
					usb_handle->busnum = atoi(dev->bus->dirname);
					usb_handle->devnum = dev->devnum;
					usb_handle->iface = iface;
					return usb_handle;
				}
			}
		}
	}

	return NULL;
}

static int novacom_usb_close(novacom_usb_handle_t *usb_handle)
{
	if(usb_handle && usb_handle->handle) {
		/* release iface if used */
		if(usb_handle->iface != -1) {
			usb_release_interface(usb_handle->handle, usb_handle->iface);
		}
		/* close */
		usb_close(usb_handle->handle);
		usb_handle->handle = NULL;
	}

	return 0;
}

int novacom_usb_transport_init(void)
{
	if (geteuid() != 0) {
		log_printf(LOG_ERROR, "need to run as super user to access usb\n");
		return -1;
	}

	usb_init();
	return 0;
}

/* new, native linux implementation */

static int novacom_usb_read(novacom_usb_handle_t *handle, void *buf, size_t len)
{
//	TRACEF("handle %p, buf %p, len %d, timeout %d\n", handle, buf, len, timeout);
//	TRACEF("fd %d\n", handle->myhandle->fd);

	struct usbdevfs_bulktransfer bulktransfer;

	bulktransfer.ep = handle->rxep;
	bulktransfer.len = len;
	bulktransfer.timeout = handle->rx_timeout;
	bulktransfer.data = buf;

	int rc;
	rc = ioctl(handle->myhandle->fd, USBDEVFS_BULK, &bulktransfer);
	if (rc > 0) { //now check the packet header

		if (novacom_usbll_check_packet_header(buf, rc)) { //bad packet
			log_printf(LOG_ERROR, "%s:%d -- received bad packet, set received packet size=%d\n", __FUNCTION__, __LINE__, rc);
			rc = 0;
		}
	}
	//TRACEF("rc %d\n", rc);
	return rc;
}

static int novacom_usb_write(novacom_usb_handle_t *handle, const void *buf, size_t len)
{
//	TRACEF("handle %p, buf %p, len %d, timeout %d\n", handle, buf, len, timeout);
//	TRACEF("fd %d\n", handle->myhandle->fd);

	struct usbdevfs_bulktransfer bulktransfer;

	bulktransfer.ep = handle->txep;
	bulktransfer.len = len;
	bulktransfer.timeout = handle->tx_timeout;
	bulktransfer.data = (void *)buf;

	int rc;
	rc = ioctl(handle->myhandle->fd, USBDEVFS_BULK, &bulktransfer);
//	TRACEF("rc %d\n", rc);
	return rc;
}

struct usb_thread_args {
	novacom_usb_handle_t *handle;
	novacom_usbll_handle_t usbll_handle;
};

static void *novacom_usb_tx_thread(void *arg)
{
	novacom_usb_handle_t *handle = (novacom_usb_handle_t *)arg;
	int rc;
	struct novacom_tx_packet packet;
	char *buf;

	buf = platform_calloc(MAX_MTU);
	platform_assert(buf != NULL);

	LTRACEF("start::wait for startup event: %p\n", handle);
	platform_event_wait(&handle->tx_startup_event);   //why waiting rx for starting ???
	handle->tx_startup_wait = 0;			  //change status to started
	LTRACEF("start::startup event received, continue: %p\n", handle);

	handle->tx_timeout = novacom_usbll_get_timeout(handle->usbll_handle);
	while (!novacom_shutdown && !handle->shutdown) {
		// see if we have something to send
		packet.len = novacom_usbll_get_mtu(handle->usbll_handle);
		packet.buf = buf;
		if (novacom_usbll_prepare_tx_packet(handle->usbll_handle, &packet, 100) != TX_NO_PACKET) {
			// write a block back
#if FAULTY_TX
			if (rand() < (RAND_MAX / 10)) {
				TRACEF("dropped tx packet\n");
			} else {
#endif
				rc = novacom_usb_write(handle, packet.buf, packet.len);
				if (rc < 0) {
					platform_time_t st;
					platform_time_t et;
					int time_used = 0;
					unsigned int count = 0;
					TRACEL(LOG_ALWAYS, "usbll(%08x) error writing packet, result(%d), errno %d\n", novacom_usbll_getuid(handle->usbll_handle), rc, errno);
					platform_get_time(&st);
					while (rc < 0 && !handle->shutdown) { //shutdown asap
						platform_get_time(&et);
						if (platform_delta_time_msecs(&st, &et) >= g_usbio_retry_timeout) {
							handle->shutdown = true;
							break;
						}
						if (g_usbio_retry_delay > 0) {
							if ((g_usbio_retry_timeout-time_used) >= g_usbio_retry_delay) {
								usleep(g_usbio_retry_delay * 1000);
								time_used += g_usbio_retry_delay;
							}
							else {
								usleep((g_usbio_retry_timeout - time_used) * 1000);
								time_used = g_usbio_retry_timeout;
							}
						}
						rc = novacom_usb_write(handle, packet.buf, packet.len);
						count++;

					}
		    			TRACEL(LOG_ALWAYS, "usbll(%08x) writing packet, writes(%ld), duration(%dms), result(%d), last_errno %ld\n", novacom_usbll_getuid(handle->usbll_handle), count, platform_delta_time_msecs(&st, &et), rc, errno);
					count = 0;
				}
				if (rc >=0) {
					TRACEF/*LOG_PRINTF*/("usbll(%08x) wrote tx packet len=%d\n", novacom_usbll_getuid(handle->usbll_handle), rc);
				}

#if FAULTY_TX
			}
#endif
		}
	}

	LTRACEF("shutting down handle %p\n", handle);

	platform_event_signal(&handle->tx_shutdown_event);

	platform_free(buf);

	return NULL;
}

static void *novacom_usb_rx_thread(void *arg)
{
	novacom_usb_handle_t *handle = (novacom_usb_handle_t *)arg;
	transport_recovery_token_t *rec_token = NULL;					///< recovery token
	int rc;
	int packet_type;
	char *buf;
	int sniff = 1;

	buf = platform_calloc(MAX_MTU);
	platform_assert(buf != NULL);

	LTRACEF("start, handle %p\n", handle);

	handle->rx_timeout = novacom_usbll_get_timeout(handle->usbll_handle);
	while (!novacom_shutdown && !handle->shutdown) {
		platform_time_t st;
		int time_used;
		// read a block from the pmux
		rc = novacom_usb_read(handle, buf, novacom_usbll_get_mtu(handle->usbll_handle));
		platform_get_time(&st);
		time_used = 0;
		if (rc <= 0) {
			platform_time_t et;
			unsigned int count = 0;
			TRACEL(LOG_ALWAYS, "%s:%d -- usbll(%08x) error: reading packet, result(%d), errno %d\n", __FUNCTION__, __LINE__, novacom_usbll_getuid(handle->usbll_handle), rc, errno);
			while (rc <= 0 && !handle->shutdown) { //shutdown asap
				platform_get_time(&et);
				if (platform_delta_time_msecs(&st, &et) >= g_usbio_retry_timeout) {
					handle->shutdown = true;
					break;
				}
				if (g_usbio_retry_delay > 0) {
					if ((g_usbio_retry_timeout-time_used) >= g_usbio_retry_delay) {
						usleep(g_usbio_retry_delay * 1000);
						time_used += g_usbio_retry_delay;
					}
					else {
						usleep((g_usbio_retry_timeout - time_used) * 1000);
						time_used = g_usbio_retry_timeout;
					}
				}
				rc = novacom_usb_read(handle, buf, novacom_usbll_get_mtu(handle->usbll_handle));
				count++;

			}
		    TRACEL(LOG_ALWAYS, "%s:%d -- usbll(%08x) reading packet, reads(%ld), duration(%dms), result(%d), last_errno %ld\n",  __FUNCTION__, __LINE__, novacom_usbll_getuid(handle->usbll_handle), count, platform_delta_time_msecs(&st, &et), rc, errno);
 		    count = 0;

		}

		/* sniff */
		if(sniff) {
			uint32_t uid = ((handle->busnum & 0x0FFFF) << 16) | (handle->devnum & 0x0FFFF);
			transport_recovery_token_t sniff_token;
			int ret;

			/* generate token from packet */
			ret = novacom_usbll_generate_recovery_token(buf, rc, &sniff_token);
			if(ret == -1) {
				TRACEL(LOG_ERROR, "%s:%d -- Used out system resouce, exit now !!!\n", __FUNCTION__, __LINE__);
				abort();
			}
			/* check queue for saved connections */
			ret = usbrecords_find(&sniff_token);
			/* free interface recovery token */
			platform_free(sniff_token.token);
			/* check result: create new handle, or recover */
			if(ret) {
				LTRACEF("Unable to recover(%d)\n", ret);
				handle->usbll_handle = novacom_usbll_create(handle->devtype, MAX_MTU, 0, USBDEVFS_IOCTL_TIMEOUT);
			} else {
				TRACEL(LOG_ERROR, "Recovered record...\n");
				handle->usbll_handle = sniff_token.user_data;
			}
			/* update uid */
			novacom_usbll_setuid(handle->usbll_handle, uid);
			handle->rx_timeout = novacom_usbll_get_timeout(handle->usbll_handle);
			handle->tx_timeout = novacom_usbll_get_timeout(handle->usbll_handle);
			sniff = 0;
		}
		/* process */
		packet_type = PACKET_TYPE_NULL;
		if (rc > 0) {
			// process it
			packet_type = novacom_usbll_process_packet(handle->usbll_handle, buf, rc);
			if (packet_type == PACKET_TYPE_BADPACKET) {
				platform_time_t et;
				TRACEF("received bad packet\n");
				platform_get_time(&et);
				if (platform_delta_time_msecs(&st, &et) >= g_usbio_retry_timeout) {
					handle->shutdown = true;
					break;
				}
				if (g_usbio_retry_delay > 0) {
					if ((g_usbio_retry_timeout-time_used) >= g_usbio_retry_delay) {
						usleep(g_usbio_retry_delay * 1000);
						time_used += g_usbio_retry_delay;
					}
					else {
						usleep((g_usbio_retry_timeout - time_used) * 1000);
						time_used = g_usbio_retry_timeout;
					}
				}
				///handle->shutdown = true;
				///break;
			} else if(handle->tx_startup_wait) {
				platform_event_signal(&handle->tx_startup_event);
			}
		} else {
#if TRACE_ZERO_LEN_PACKETS
			log_printf(LOG_TRACE, "RX zero len\n");
#endif
		}
	}

	LTRACEF("shutting down handle %p\n", handle);

	/* wake up tx thread (if still waits for startup) */
	if(handle->tx_startup_wait) {
		LTRACEF("wake up tx thread\n");
		platform_event_signal(&handle->tx_startup_event);
	}

	/* wait for the tx thread to exit */
	LTRACEF("waiting on tx thread\n");
	platform_event_wait(&handle->tx_shutdown_event);

	/* RX thread is responsible for cleaning up */
	LTRACEF("cleaning up handle %p\n", handle);

	/* grab recovery token if available */
	if(handle->usbll_handle) {
		rc = -1;
		rec_token = platform_calloc(sizeof(transport_recovery_token_t));
		if(rec_token) {
			snprintf(rec_token->nduid, sizeof(rec_token->nduid), "%s", novacom_usbll_get_nduid(handle->usbll_handle));
			rc = novacom_usbll_get_recovery_token(handle->usbll_handle, rec_token);
			if(rc != -1) {
				rc = usbrecords_add(rec_token);
			} else {
				LTRACEF("unable to recovery token!!!\n");
			}
		}
		/* error: free memory, destroy device */
		if(rc == -1) { //we should never go here.
			novacom_usbll_destroy(handle->usbll_handle);
			platform_free(rec_token);
		}
	}

	novacom_usb_close(handle);
	platform_event_destroy(&handle->tx_startup_event);
	platform_event_destroy(&handle->tx_shutdown_event);
	platform_free(handle);
	platform_free(buf);

	return NULL;
}

struct novacom_usb_inotify_entry
{
	/* watch descriptor */
	int wd;
	/* bus number (dirname) */
	char *bus;
	SLIST_ENTRY(novacom_usb_inotify_entry) entries;
};

struct novacom_usb_inotify_context
{
	/* inotify fd */
	int fd;
	/* buffer to read inotify event (big enough to fit one event) */
	char event_buf[sizeof(struct inotify_event) + NAME_MAX + 1];
	/* inotify watch descriptors */
	SLIST_HEAD(wds_s, novacom_usb_inotify_entry) wds;
};

/*
 * @brief test that given name is invalid USB busname
 * @ret -1 error
 *       0 success (name contains only digits)
 */
static int novacom_usb_is_invalid_bus_name(const char *name)
{
	while(*name) {
		if (!isdigit(*name))
			return -1;
		++name;
	}
	return 0;
}

/*
 * @brief add inotify watches to track USB device additions
 * @ret -1 error
 *       0 success
 */
static int novacom_usb_inotify_context_init(struct novacom_usb_inotify_context *context)
{
	context->fd = inotify_init();
	if (context->fd == -1) {
		TRACEF("inotify_init failed\n");
		return -1;
	}

	const char *DEV_USB = "/dev/bus/usb/";
	DIR *dusb = opendir(DEV_USB);
	struct dirent *dentry;
	char path[PATH_MAX];

	for (dentry = readdir(dusb); dentry; dentry = readdir(dusb)) {
		if (dentry->d_type != DT_DIR) {
			continue;
		}
		if (novacom_usb_is_invalid_bus_name(dentry->d_name)) {
			continue;
		}

		strcpy(path, DEV_USB);
		strcat(path, dentry->d_name);
		int wd = inotify_add_watch(context->fd, path, IN_CREATE);
		if (wd == -1) {
			TRACEF("inotify_add_watch failed for path: %s, error: %s\n", path, strerror(errno));
			return -1;
		}
		struct novacom_usb_inotify_entry *entry = (struct novacom_usb_inotify_entry *)platform_alloc(sizeof(struct novacom_usb_inotify_entry));
		entry->wd = wd;
		entry->bus = platform_strdup(dentry->d_name);
		SLIST_INSERT_HEAD(&context->wds, entry, entries);
	}
	closedir(dusb);

	return 0;
}

/*
 * @brief cleanup inotify context, destroy watches and close inotify fd
 */
static void novacom_usb_inotify_context_destroy(struct novacom_usb_inotify_context *context)
{
	struct novacom_usb_inotify_entry *entry;
	SLIST_FOREACH(entry, &context->wds, entries) {
		platform_free(entry->bus);
		int rc = inotify_rm_watch(context->fd, entry->wd);
		if (rc == -1) {
			TRACEF("inotify_rm_watch failed: %s\n", strerror(errno));
		}
		platform_free(entry);
	}
	close(context->fd);
}

/*
 * @brief wait for inotify event when new USB device added.
 * @ret -1 error
 *       0 success, busname and devname contains assigned path names for added device
 */
static int novacom_usb_inotify_context_read(struct novacom_usb_inotify_context *context, const char **busname, const char **devname)
{
	platform_assert(context->fd != -1);
	int bytes = read(context->fd, context->event_buf, sizeof(context->event_buf));
	if (bytes < 0) {
		TRACEF("failed to read from inotify fd (errno: %s)\n", strerror(errno));
		return -1;
	}
	struct inotify_event *event = (struct inotify_event *)context->event_buf;
	platform_assert(event->mask & IN_CREATE);
	platform_assert(event->len > 0);

	*devname = event->name;

	struct novacom_usb_inotify_entry *entry;
	SLIST_FOREACH(entry, &context->wds, entries) {
		if (entry->wd == event->wd) {
			*busname = entry->bus;
			TRACEF("device added: bus: %s, device: %s\n", *busname, *devname);
			return 0;
		}
	}
	return -1;
}

/* main worker thread */
static void *novacom_usb_findandattach_thread(void *arg)
{
	novacom_usb_handle_t *usb;

	/* init records */
	usbrecords_init();

	/* initialize records queue */
	TAILQ_INIT(&t_recovery_queue);

	struct novacom_usb_inotify_context inotify_context;
	novacom_usb_inotify_context_init(&inotify_context);

	/* busname/devname == NULL, let novacom_usb_open do full device discovery */
	const char *busname = NULL;
	const char *devname = NULL;

	platform_time_t time_prev;
	platform_get_time(&time_prev);

	/* device discovery */
	while (!novacom_shutdown) {

		usb = novacom_usb_open(busname, devname);
		if (usb ) {
			usb->shutdown = false;
			TRACEF("usb_handle 0x%08x, bus=%03d dev=%03d\n", usb->usbll_handle, usb->busnum, usb->devnum);
			platform_event_create(&usb->tx_startup_event);
			platform_event_unsignal(&usb->tx_startup_event);
			usb->tx_startup_wait = 1;
			platform_event_create(&usb->tx_shutdown_event);
			platform_event_unsignal(&usb->tx_shutdown_event);
	
			platform_create_thread(NULL, &novacom_usb_rx_thread, (void *)usb);
			platform_create_thread(NULL, &novacom_usb_tx_thread, (void *)usb);
		}

		/* wait for new devices */
		novacom_usb_inotify_context_read(&inotify_context, &busname, &devname);

		if (!novacom_shutdown) {
			platform_time_t time_cur;
			platform_get_time(&time_cur);
			const int elapsed_sec = platform_delta_time_msecs(&time_prev, &time_cur) / 1000;
			if (elapsed_sec > 0) {
				time_prev = time_cur;
				/* check recovery records, shutdown interface if timeout expired */
				(void) usbrecords_update(elapsed_sec);
			}
		}
	}

	novacom_usb_inotify_context_destroy(&inotify_context);

	/* update records: forcing shutdown of all records */
	usbrecords_update(TRANSPORT_RECOVERY_TIMEOUT);

	return NULL;
}

int novacom_usb_transport_start(void)
{
	novacom_shutdown = 0;
	platform_create_thread(&findandattach_thread, &novacom_usb_findandattach_thread, NULL);
	platform_mutex_init(&recovery_lock);
	return 0;
}

int novacom_usb_transport_stop(void)
{
	novacom_shutdown = 1;

	platform_waitfor_thread(findandattach_thread);
	platform_mutex_destroy(&recovery_lock);

	return 0;
}

/*
 * @brief: device_online
 */
int novacom_usb_transport_deviceonline(char *nduid)
{
	usbrecords_remove(nduid);
	return 0;
}

