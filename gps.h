#ifndef GPS_h_ee1b4dab_0d3b_42e2_841f_0d4f1ec0e6bd_
#define GPS_h_ee1b4dab_0d3b_42e2_841f_0d4f1ec0e6bd_ 1

//#define LOG_NDEBUG 0
#define LOG_TAG "GPS"
#include <cutils/log.h>
#include <hardware/gps.h>

#include <rpc/rpc.h>
#include "loc_api_rpc_glue.h"

#define TRACEV_ENTER() TRACEV("enter\n")
#define TRACEV_LEAVE() TRACEV("leave\n")
#define TRACEV(fmt,args...) ALOGV("%s:%s:%d: " fmt, __FILE__, __FUNCTION__, __LINE__ , ## args )

#define FUNC_ALOGV(fmt,args...) ALOGV("%s: " fmt, __FUNCTION__ , ## args )
#define FUNC_ALOGD(fmt,args...) ALOGD("%s: " fmt, __FUNCTION__ , ## args )
#define FUNC_ALOGI(fmt,args...) ALOGI("%s: " fmt, __FUNCTION__ , ## args )
#define FUNC_ALOGW(fmt,args...) ALOGW("%s: " fmt, __FUNCTION__ , ## args )
#define FUNC_ALOGE(fmt,args...) ALOGE("%s: " fmt, __FUNCTION__ , ## args )

extern GpsInterface hal_gps_interface;

#define GPS_CLIENT_UPDATED_STATUS         (1<<0)
#define GPS_CLIENT_UPDATED_TIME           (1<<1)
#define GPS_CLIENT_UPDATED_LOC            (1<<2)
#define GPS_CLIENT_UPDATED_XTRA           (1<<3)
#define GPS_CLIENT_UPDATED_DELETE_AIDING  (1<<4)
#define GPS_CLIENT_UPDATED_POSM           (1<<5)
#define GPS_CLIENT_UPDATED_AGPS           (1<<6)
#define GPS_CLIENT_UPDATED_AGPS_SUPL      (1<<7)
#define GPS_CLIENT_UPDATED_REF_LOC        (1<<8)
#define GPS_CLIENT_UPDATED_SET_ID         (1<<9)
#define GPS_CLIENT_UPDATED_NETWORK_STATE  (1<<10)
#define GPS_CLIENT_UPDATED_NETWORK_AVAIL  (1<<11)
#define GPS_CLIENT_UPDATED_RELOAD         (1<<12) /* for server use only */

/* APN: real maximum length is 100 octets/63 octets, not including NUL */
#define GPS_APN_MAX	128

/* DNS: per RFC 1035, section 2.3.4 */
#define GPS_DNS_MAX	256

/* IMSI: maximum length 15 (w/o NUL) */
#define GPS_IMSI_MAX	16

/* MSISDN: real maximum length 16 (w/o NUL or prefixes) */
#define GPS_IMSISDN_MAX	32

/* max(GPS_IMSI_MAX, GPS_IMSISDN_MAX) */
#define GPS_SETID_MAX	GPS_IMSISDN_MAX

/* Values the client desires */
struct gps_client_state {
	pthread_mutex_t lock;
	pthread_cond_t updated_cond;
	unsigned updated_flags;

	pthread_rwlock_t cb_lock;
	GpsCallbacks *cb;
	GpsXtraCallbacks *xtra_cb;
	AGpsCallbacks *agps_cb;
	AGpsRilCallbacks *ril_cb;

	GpsStatusValue status;

	struct gps_client_time {
		GpsUtcTime time;
		int64_t ref;
		int unc;
	} time;

	struct gps_client_loc {
		double latitude, longitude;
		float accuracy;
	} loc;

	GpsAidingData delete_aiding;

	struct gps_client_posm {
		GpsPositionMode mode;
		GpsPositionRecurrence recurrence;
		uint32_t min_interval;
		uint32_t preferred_accuracy;
		uint32_t preferred_time;
	} posm;

	struct gps_client_xtra {
		char *data;
		int len;
	} xtra;

	struct gps_client_agps {
		enum gps_server_agps_status {
			AGPS_STATUS_NONE,
			AGPS_STATUS_OPEN,
			AGPS_STATUS_CLOSED,
			AGPS_STATUS_FAILED,
		} status;
		char apn[GPS_APN_MAX];
	} agps;

	struct gps_client_agps_supl {
		AGpsType type;
		char hostname[GPS_DNS_MAX];
		int port;
	} agps_supl;

	AGpsRefLocation ref_loc;

	struct gps_client_set_id {
		AGpsSetIDType type;
		char setid[GPS_SETID_MAX];
	} set_id;

	struct gps_client_network_state {
		int connected;
		int type;
		int roaming;
		char *extra_info;
	} network_state;

	struct gps_client_network_avail {
		int available;
		char apn[GPS_APN_MAX];
	} network_avail;

};

/* Current server values */
#define GPS_SERVER_EVENT_COUNT	128

struct gps_server_state {
	pthread_mutex_t lock;

	rpc_loc_client_handle_type handle;

	struct {
		pthread_mutex_t lock;
		pthread_cond_t cond;
		volatile unsigned head, tail;
		struct gps_server_event {
			rpc_loc_event_mask_type event;
			rpc_loc_event_payload_u_type payload;
		} q[GPS_SERVER_EVENT_COUNT];
	} event;

	pthread_t send_thread, recv_thread;

	struct gps_server_api {
		unsigned char major;
		unsigned char minor;
	} api;

	GpsStatus status;
	rpc_loc_engine_state_e_type engine_state;
	rpc_loc_fix_session_state_e_type session_state;
	rpc_loc_fix_criteria_s_type fix_criteria;
	rpc_loc_lock_e_type engine_lock;
	rpc_boolean sbas_enabled;
	rpc_loc_nmea_sentence_type nmea_types;
	rpc_boolean lpm;

	struct gps_server_agps {
		rpc_loc_server_connection_handle conn_handle;
		AGpsStatus status;
	} agps;

	struct gps_server_gps {
		rpc_uint64 timestamp;
		rpc_uint8 leap_seconds;
		float magnetic_deviation;
		rpc_loc_pos_technology_mask_type tech;
		float speed_vertical;
	} gps;

	struct gps_server_xtra {
		rpc_uint32 max_file_size;
		rpc_uint32 max_part_size;
		int uploaded;
		rpc_int32 error;
		rpc_uint64 valid_start;
		rpc_uint16 valid_hrs;
	} xtra;
};

extern struct gps_client_state gps_client;
extern struct gps_server_state gps_server;

extern void server_recv_thread(void *arg);
extern void server_send_thread(void *arg);

#define CLIENT_LOCK() pthread_mutex_lock(&gps_client.lock)
#define CLIENT_UPDATE(flags) do { \
		gps_client.updated_flags |= (flags); \
		pthread_cond_broadcast(&gps_client.updated_cond); \
	} while(0)
#define CLIENT_UPDATE_NB(flags) do { gps_client.updated_flags |= (flags); } while(0)
#define CLIENT_UPDATE_B() pthread_cond_broadcast(&gps_client.updated_cond)
#define CLIENT_UNLOCK() pthread_mutex_unlock(&gps_client.lock)

#define SERVER_LOCK() pthread_mutex_lock(&gps_server.lock)
#define SERVER_UNLOCK() pthread_mutex_unlock(&gps_server.lock)

#define EVENT_LOCK() pthread_mutex_lock(&gps_server.event.lock)
#define EVENT_UPDATE() pthread_cond_broadcast(&gps_server.event.cond)
#define EVENT_WAIT() pthread_cond_wait(&gps_server.event.cond, &gps_server.event.lock)
#define EVENT_UNLOCK() pthread_mutex_unlock(&gps_server.event.lock)

#if DEBUG_CHECK_CALLBACKS
#define CB(cb, cb_func, args) ({ \
	typeof(gps_client.cb->cb_func args) ret_; \
	pthread_rwlock_rdlock(&gps_client.cb_lock); \
	if (gps_client.cb && gps_client.cb->cb_func) ret_ = gps_client.cb->cb_func args; \
	else ALOGE("CALLING UNREGISTERED CALLBACK " #cb #cb_func #cb_args "\n"); \
	pthread_rwlock_unlock(&gps_client.cb_lock); \
	ret_; \
	})
#else
#define CB(cb, cb_func, args) ( \
	(!gps_client.cb || !gps_client.cb->cb_func) ? \
		0 \
	: \
		(gps_client.cb->cb_func args) \
	)
#endif

/* DEBUG TODO: Check attempt to register while read locked */
#define CB_REGISTER(cb_memb, callbacks) ({ \
		int ret_ = 0; \
		pthread_rwlock_wrlock(&gps_client.cb_lock); \
		if (__builtin_strcmp(#cb_memb, "cb") ? (gps_client.cb==NULL) : (gps_client.cb!=NULL)) \
			ret_ = -1; \
		else gps_client.cb_memb = (callbacks); \
		pthread_rwlock_unlock(&gps_client.cb_lock); \
		ret_; \
	})

#endif
