#include "gps.h"

#include <string.h>

struct gps_client_state gps_client = {
	.lock = PTHREAD_MUTEX_INITIALIZER,
	.updated_cond = PTHREAD_COND_INITIALIZER,
	.cb_lock = PTHREAD_RWLOCK_INITIALIZER,
	.cb = NULL,
};

struct gps_server_state gps_server = {
	.lock = PTHREAD_MUTEX_INITIALIZER,
	.event = {
		.lock = PTHREAD_MUTEX_INITIALIZER,
		.cond = PTHREAD_COND_INITIALIZER,
	},
};

/* ======================== Client-side API ========================= */

/* ==================
   == GpsInterface == */

static int gps_init(GpsCallbacks *callbacks)
{
	const uint32_t caps =
		GPS_CAPABILITY_SCHEDULING |
		GPS_CAPABILITY_MSB |
		GPS_CAPABILITY_MSA |
		GPS_CAPABILITY_SINGLE_SHOT |
		GPS_CAPABILITY_ON_DEMAND_TIME;

	TRACEV_ENTER();

	if (CB_REGISTER(cb, callbacks)) {
		TRACEV("already loaded?!\n");
		return -1;
	}

	CB(cb, set_capabilities_cb, (caps));

	loc_api_glue_init();

	CLIENT_LOCK();
	gps_client.status = GPS_STATUS_ENGINE_ON;
	gps_client.updated_flags = GPS_CLIENT_UPDATED_STATUS;

	gps_client.posm.mode = GPS_POSITION_MODE_MS_BASED;
	gps_client.posm.recurrence = GPS_POSITION_RECURRENCE_PERIODIC;
	gps_client.posm.min_interval = 1000;
	gps_client.posm.preferred_accuracy = 50;
	gps_client.posm.preferred_time = 10000;

	gps_client.xtra.data = NULL;
	gps_client.xtra.len = 0;

	gps_client.agps_supl.hostname[0] = '\0';
	gps_client.agps_supl.port = 0;

	SERVER_LOCK();
	gps_server.status.size = sizeof(gps_server.status);
	gps_server.status.status = GPS_STATUS_ENGINE_OFF;
	gps_server.event.head = gps_server.event.tail = 0;

	gps_server.engine_state = 0;
	gps_server.session_state = 0;
	gps_server.gps.timestamp = 0;
	gps_server.gps.tech = 0;

	gps_server.send_thread = CB(cb, create_thread_cb, ("gps-server-send", server_send_thread, NULL));
	gps_server.recv_thread = CB(cb, create_thread_cb, ("gps-server-recv", server_recv_thread, NULL));
	pthread_detach(gps_server.recv_thread);
	SERVER_UNLOCK();
	CLIENT_UNLOCK();

	TRACEV_LEAVE();

	return 0;
}

static int gps_start(void)
{
	CLIENT_LOCK();
	gps_client.status = GPS_STATUS_SESSION_BEGIN;
	CLIENT_UPDATE(GPS_CLIENT_UPDATED_STATUS);
	CLIENT_UNLOCK();
	return 0;
}

static int gps_stop(void)
{
	CLIENT_LOCK();
	gps_client.status = GPS_STATUS_ENGINE_ON;
	CLIENT_UPDATE(GPS_CLIENT_UPDATED_STATUS);
	CLIENT_UNLOCK();
	return 0;	
}

static int gps_inject_time(GpsUtcTime time, int64_t timeReference, int uncertainty)
{
	CLIENT_LOCK();
	gps_client.time.time = time;
	gps_client.time.ref = timeReference;
	gps_client.time.unc = uncertainty;
	CLIENT_UPDATE(GPS_CLIENT_UPDATED_TIME);
	CLIENT_UNLOCK();
	return 0;
}

static int gps_inject_location(double latitude, double longitude, float accuracy)
{
	CLIENT_LOCK();
	gps_client.loc.latitude = latitude;
	gps_client.loc.longitude = longitude;
	gps_client.loc.accuracy = accuracy;
	CLIENT_UPDATE(GPS_CLIENT_UPDATED_LOC);
	CLIENT_UNLOCK();
	return 0;
}

#define GPS_DELETE_XTRA (GPS_DELETE_EPHEMERIS|GPS_DELETE_ALMANAC)
static void gps_delete_aiding_data(GpsAidingData flags)
{
	char *free_xtra_data = NULL;
	CLIENT_LOCK();
	if ((flags & GPS_DELETE_XTRA) == GPS_DELETE_XTRA) {
		free_xtra_data = gps_client.xtra.data;
		gps_client.xtra.data = NULL;
		gps_client.xtra.len = 0;
		CLIENT_UPDATE_NB(GPS_CLIENT_UPDATED_XTRA);
	}
	gps_client.delete_aiding = flags;
	CLIENT_UPDATE(GPS_CLIENT_UPDATED_DELETE_AIDING);
	CLIENT_UNLOCK();
	free(free_xtra_data);
}

static int gps_set_position_mode(
	GpsPositionMode mode,
	GpsPositionRecurrence recurrence,
	uint32_t min_interval,
	uint32_t preferred_accuracy,
	uint32_t preferred_time
)
{
	CLIENT_LOCK();
	gps_client.posm.mode = mode;
	gps_client.posm.recurrence = recurrence;
	gps_client.posm.min_interval = min_interval;
	gps_client.posm.preferred_accuracy = preferred_accuracy;
	gps_client.posm.preferred_time = preferred_time;
	CLIENT_UPDATE(GPS_CLIENT_UPDATED_POSM);
	CLIENT_UNLOCK();
	return 0;
}

static void gps_cleanup(void)
{
	void *ignored_retval;

	TRACEV_ENTER();

	CLIENT_LOCK();
	gps_client.status = GPS_STATUS_NONE;
	CLIENT_UPDATE(GPS_CLIENT_UPDATED_STATUS);
	CLIENT_UNLOCK();

	pthread_join(gps_server.send_thread, &ignored_retval);

	free(gps_client.xtra.data);
	gps_client.xtra.data = NULL;
	gps_client.xtra.len = 0;

	free(gps_client.network_state.extra_info);
	gps_client.network_state.extra_info = NULL;

	pthread_rwlock_wrlock(&gps_client.cb_lock);
	gps_client.cb = NULL;
	pthread_rwlock_unlock(&gps_client.cb_lock);

	TRACEV_LEAVE();
}

static const void *gps_get_extension(const char *name);

GpsInterface hal_gps_interface = {
	.size = sizeof(GpsInterface),
	.init = gps_init,
	.start = gps_start,
	.stop = gps_stop,
	.cleanup = gps_cleanup,
	.inject_time = gps_inject_time,
	.inject_location = gps_inject_location,
	.delete_aiding_data = gps_delete_aiding_data,
	.set_position_mode = gps_set_position_mode,
	.get_extension = gps_get_extension,
};

/* ======================
   == GpsXtraInterface == */

static GpsXtraCallbacks *xtra_cb = NULL;

static int xtra_init(GpsXtraCallbacks *callbacks)
{
	return CB_REGISTER(xtra_cb, callbacks);
}

static int xtra_inject_xtra_data(char *data, int length)
{
	char *dup_data = malloc(length), *free_data;
	memcpy(dup_data, data, length);

	CLIENT_LOCK();
	free_data = gps_client.xtra.data;
	gps_client.xtra.data = dup_data;
	gps_client.xtra.len = length;
	CLIENT_UPDATE(GPS_CLIENT_UPDATED_XTRA);
	CLIENT_UNLOCK();

	free(free_data);

	return 0;
}

static GpsXtraInterface hal_xtra_interface = {
	.size = sizeof(GpsXtraInterface),
	.init = xtra_init,
	.inject_xtra_data = xtra_inject_xtra_data,
};

/* GpsDebugInterface not supported */

/* ===================
   == AGpsInterface == */

static AGpsCallbacks *agps_cb = NULL;

static void agps_init(AGpsCallbacks *callbacks)
{
	CB_REGISTER(agps_cb, callbacks);
}

static int agps_data_conn_open(const char *apn)
{
	CLIENT_LOCK();
	strlcpy(gps_client.agps.apn, apn, sizeof(gps_client.agps.apn));
	gps_client.agps.status = AGPS_STATUS_OPEN;
	CLIENT_UPDATE(GPS_CLIENT_UPDATED_AGPS);
	CLIENT_UNLOCK();
	return 0;
}

static int agps_data_conn_closed()
{
	CLIENT_LOCK();
	gps_client.agps.status = AGPS_STATUS_CLOSED;
	CLIENT_UPDATE(GPS_CLIENT_UPDATED_AGPS);
	CLIENT_UNLOCK();
	return 0;
}

static int agps_data_conn_failed()
{
	CLIENT_LOCK();
	gps_client.agps.status = AGPS_STATUS_FAILED;
	CLIENT_UPDATE(GPS_CLIENT_UPDATED_AGPS);
	CLIENT_UNLOCK();
	return 0;
}

static int agps_set_server(AGpsType type, const char *hostname, int port)
{
	if (type != AGPS_TYPE_SUPL) {
		FUNC_ALOGW("server type %d not supported.\n", type);
		return -1;
	}

	CLIENT_LOCK();
	gps_client.agps_supl.type = type;
	strlcpy(gps_client.agps_supl.hostname, hostname, sizeof(gps_client.agps_supl.hostname));
	gps_client.agps_supl.port = port;
	CLIENT_UPDATE(GPS_CLIENT_UPDATED_AGPS_SUPL);
	CLIENT_UNLOCK();

	return 0;
}

static AGpsInterface hal_agps_interface = {
	.size = sizeof(AGpsInterface),
	.init = agps_init,
	.data_conn_open = agps_data_conn_open,
	.data_conn_closed = agps_data_conn_closed,
	.data_conn_failed = agps_data_conn_failed,
	.set_server = agps_set_server,
};

/* GpsNiInterface not supported */

/* ======================
   == AGpsRilInterface == */

static AGpsRilCallbacks *ril_cb = NULL;

static void ril_init(AGpsRilCallbacks *callbacks)
{
	CB_REGISTER(ril_cb, callbacks);
}

static void ril_set_ref_location(const AGpsRefLocation *refloc, size_t size)
{
	CLIENT_LOCK();
	if (size > sizeof(gps_client.ref_loc)) {
		FUNC_ALOGW("size of refloc > sizeof(AGpsRefLocation) (API mismatch?)\n");
		size = sizeof(gps_client.ref_loc);
	}
	memcpy(&gps_client.ref_loc, refloc, size);
	CLIENT_UPDATE(GPS_CLIENT_UPDATED_REF_LOC);
	CLIENT_UNLOCK();
}

static void ril_set_set_id(AGpsSetIDType type, const char *setid)
{
	CLIENT_LOCK();
	gps_client.set_id.type = type;
	strlcpy(gps_client.set_id.setid, setid, sizeof(gps_client.set_id.setid));
	CLIENT_UPDATE(GPS_CLIENT_UPDATED_SET_ID);
	CLIENT_UNLOCK();
}

static void ril_ni_message(uint8_t *msg, size_t len)
{
	FUNC_ALOGW("AGpsRilInterface->ni_message not supported yet\n");
}

static void ril_update_network_state(int connected, int type, int roaming, const char *extra_info)
{
	char *dup_extra_info = strdup(extra_info), *free_extra_info;
	CLIENT_LOCK();
	gps_client.network_state.connected = connected;
	gps_client.network_state.type = type;
	gps_client.network_state.roaming = roaming;
	free_extra_info = gps_client.network_state.extra_info;
	gps_client.network_state.extra_info = dup_extra_info;
	CLIENT_UPDATE(GPS_CLIENT_UPDATED_NETWORK_STATE);
	CLIENT_UNLOCK();
	free(free_extra_info);
}

static void ril_update_network_availability(int available, const char *apn)
{
	CLIENT_LOCK();
	gps_client.network_avail.available = available;
	strlcpy(gps_client.network_avail.apn, apn, sizeof(gps_client.network_avail.apn));
	CLIENT_UPDATE(GPS_CLIENT_UPDATED_NETWORK_AVAIL);
	CLIENT_UNLOCK();
}

static AGpsRilInterface hal_ril_interface = {
	.size = sizeof(AGpsRilInterface),
	.init = ril_init,
	.set_ref_location = ril_set_ref_location,
	.set_set_id = ril_set_set_id,
	.ni_message = ril_ni_message,
	.update_network_state = ril_update_network_state,
	.update_network_availability = ril_update_network_availability,
};

/* ======================== HAL ========================= */

static const void *gps_get_extension(const char *name)
{
	if (!strcmp(name, GPS_XTRA_INTERFACE))
		return &hal_xtra_interface;
	if (!strcmp(name, AGPS_INTERFACE))
		return &hal_agps_interface;
	if (!strcmp(name, AGPS_RIL_INTERFACE))
		return &hal_ril_interface;
	return NULL;
}
