#include "gps.h"

#include <sys/types.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netdb.h>

#include <netinet/in.h>

#define XTRA_UPLOAD_TIMEOUT	5	/* seconds */

/* ========================   Server-side   ========================= */

/* adds reports from the server to the queue and returns quickly. */
static int32 server_loc_event_cb(
	rpc_loc_client_handle_type handle,
	rpc_loc_event_mask_type loc_event,
	const rpc_loc_event_payload_u_type *loc_event_payload
)
{
	CB(cb, acquire_wakelock_cb, ());

	unsigned head = gps_server.event.head;
	unsigned tail = gps_server.event.tail;

	if (handle != gps_server.handle) {
		FUNC_ALOGE("got event for wrong handle\n");
		if (head == tail)
			CB(cb, release_wakelock_cb, ());
		return RPC_LOC_API_SUCCESS;
	}

	unsigned hidx = head % GPS_SERVER_EVENT_COUNT;
	unsigned tidx = tail % GPS_SERVER_EVENT_COUNT;
	if (head != tail && hidx == tidx) {
		FUNC_ALOGE("buffer full, dropping event %llu\n", (unsigned long long)loc_event);
		return RPC_LOC_API_SUCCESS;
	}

	FUNC_ALOGV("got event %llu\n", (unsigned long long)loc_event);
	struct gps_server_event *q = &gps_server.event.q[tidx];
	q->event = loc_event;
	memcpy(&q->payload, loc_event_payload, sizeof(q->payload));

	EVENT_LOCK();
	gps_server.event.tail = tail + 1;
	CB(cb, acquire_wakelock_cb, ());
	EVENT_UPDATE();
	EVENT_UNLOCK();

	return RPC_LOC_API_SUCCESS;
}

static void server_handle_PARSED_POSITION_REPORT(struct gps_server_event *q)
{
	const rpc_loc_parsed_position_s_type *r =
		&q->payload.rpc_loc_event_payload_u_type_u.parsed_location_report;

	GpsLocation loc;
	memset(&loc, 0, sizeof(loc));
	loc.size = sizeof(loc);

	if (!(r->valid_mask & RPC_LOC_POS_VALID_SESSION_STATUS)) {
		FUNC_ALOGW("invalid session status\n");
		return;
	}

	if (r->session_status != RPC_LOC_SESS_STATUS_SUCCESS &&
	    r->session_status != RPC_LOC_SESS_STATUS_IN_PROGESS)
	{
		if (r->session_status != RPC_LOC_SESS_STATUS_GENERAL_FAILURE &&
		    r->session_status != RPC_LOC_SESS_STATUS_USER_END)
			FUNC_ALOGW("bad session status %d", r->session_status);
		return;
	}

	if (r->valid_mask & RPC_LOC_POS_VALID_TIMESTAMP_UTC) {
		loc.timestamp = r->timestamp_utc;
	} else if (r->valid_mask & RPC_LOC_POS_VALID_TIMESTAMP_CALENDAR) {
		struct tm tm = {
			.tm_year = r->timestamp_calendar.year,
			.tm_mon = r->timestamp_calendar.month,
			.tm_mday = r->timestamp_calendar.day,
			.tm_hour = r->timestamp_calendar.hour,
			.tm_min = r->timestamp_calendar.minute,
			.tm_sec = r->timestamp_calendar.second,
			.tm_isdst = 0,
		};
		time_t timestamp = timegm(&tm);
		loc.timestamp = timestamp*1000ULL + r->timestamp_calendar.millisecond;
	} else {
		/* these are really inaccurate anyway :P */
		FUNC_ALOGV("no timestamp in location; valid_mask = 0x%08lx", (unsigned long)r->valid_mask);
		return;
	}

	if ((r->valid_mask & RPC_LOC_POS_VALID_LATITUDE) && (r->valid_mask & RPC_LOC_POS_VALID_LONGITUDE)) {
		loc.latitude = r->latitude;
		loc.longitude = r->longitude;
		loc.flags |= GPS_LOCATION_HAS_LAT_LONG;
	}

	if (r->valid_mask & RPC_LOC_POS_VALID_ALTITUDE_WRT_ELLIPSOID) {
		loc.altitude = r->altitude_wrt_ellipsoid;
		loc.flags |= GPS_LOCATION_HAS_ALTITUDE;
	} else if (r->valid_mask & RPC_LOC_POS_VALID_ALTITUDE_WRT_MEAN_SEA_LEVEL) {
		/* this is wrong. */
		FUNC_ALOGV("engine only gave altitude from mean sea level\n");
		loc.altitude = r->altitude_wrt_mean_sea_level;
		loc.flags |= GPS_LOCATION_HAS_ALTITUDE;
	}

	if (r->valid_mask & RPC_LOC_POS_VALID_SPEED_HORIZONTAL) {
		loc.speed = r->speed_horizontal;
		loc.flags |= GPS_LOCATION_HAS_SPEED;
	}

	if (r->valid_mask & RPC_LOC_POS_VALID_HEADING) {
#if (RPC_LOC_EVENT_CB_F_TYPE_VERSION >> 16) == 2
		/* AMSS 20000 expresses heading as [0, 1024/10) instead of [0, 360) */
		loc.bearing = r->heading * 360 * 10/1024;
#else
		loc.bearing = r->heading;
#endif
		loc.flags |= GPS_LOCATION_HAS_BEARING;
	}

	if (r->valid_mask & RPC_LOC_POS_VALID_HOR_UNC_CIRCULAR) {
		loc.accuracy = r->hor_unc_circular;
		loc.flags |= GPS_LOCATION_HAS_ACCURACY;
	}

	/* keep track of some misc. stuff for debugging */
	SERVER_LOCK();
	gps_server.gps.timestamp = (r->valid_mask & RPC_LOC_POS_VALID_TIMESTAMP_UTC) ? r->timestamp_utc : 0;
	gps_server.gps.leap_seconds = (r->valid_mask & RPC_LOC_POS_VALID_LEAP_SECONDS) ? r->leap_seconds : 0;
	if (r->valid_mask & RPC_LOC_POS_VALID_MAGNETIC_VARIATION)
		gps_server.gps.magnetic_deviation = r->magnetic_deviation;
	if (r->valid_mask & RPC_LOC_POS_VALID_TECHNOLOGY_MASK) {
		if (gps_server.gps.tech != r->technology_mask)
			FUNC_ALOGI("tech = 0x%08x\n", (unsigned int)r->technology_mask);
	}
	gps_server.gps.tech = (r->valid_mask & RPC_LOC_POS_VALID_TECHNOLOGY_MASK) ? r->technology_mask : 0;
	gps_server.gps.speed_vertical = (r->valid_mask & RPC_LOC_POS_VALID_SPEED_VERTICAL) ? r->speed_vertical : -1;
	SERVER_UNLOCK();

	FUNC_ALOGV("location report: valid:%08x ts:%lld pos:%f,%f±%f alt:%f spd:%f bng:%f\n",
		(unsigned)loc.flags,
		(unsigned long long)loc.timestamp,
		loc.latitude, loc.longitude, (double)loc.accuracy,
		loc.altitude,
		(double)loc.speed,
		(double)loc.bearing);

	CB(cb, location_cb, (&loc));
}

static void server_handle_SATELLITE_REPORT(struct gps_server_event *q)
{
	const rpc_loc_gnss_info_s_type *r =
		&q->payload.rpc_loc_event_payload_u_type_u.gnss_report;
	unsigned i;

	GpsSvStatus sv;
	memset(&sv, 0, sizeof(sv));
	sv.size = sizeof(sv);

	if (!(r->valid_mask & RPC_LOC_GNSS_INFO_VALID_SV_COUNT)) {
		FUNC_ALOGV("no satellite count");
		return;
	}
	if (!(r->valid_mask & RPC_LOC_GNSS_INFO_VALID_SV_LIST)) {
		if (r->sv_count)
			FUNC_ALOGV("no satellite list (count %u)", (unsigned)r->sv_count);
		CB(cb, sv_status_cb, (&sv));
		return;
	}

	for (i = 0; i < r->sv_list.sv_list_len && sv.num_svs < GPS_MAX_SVS; ++i) {
		GpsSvInfo *svi = &sv.sv_list[sv.num_svs];
		const rpc_loc_sv_info_s_type *rsv = &r->sv_list.sv_list_val[i];

		if (!(rsv->valid_mask & RPC_LOC_SV_INFO_VALID_SYSTEM))
			continue;

		svi->size = sizeof(*svi);

		if (rsv->valid_mask & RPC_LOC_SV_INFO_VALID_PRN) {
			switch (rsv->system) {
			case RPC_LOC_SV_SYSTEM_GPS:
				svi->prn = rsv->prn;
				break;
			case RPC_LOC_SV_SYSTEM_GLONASS:
				/* FIXME: How does the AMSS report the PRNs for these, 1-24 or 66-85? */
				if (rsv->prn >= 1 && rsv->prn <= 24) {
					FUNC_ALOGI("AMSS reports GLONASS as 1-24\n");
					svi->prn = rsv->prn + 65 - 1;
				} else if (rsv->prn >= 66 && rsv->prn <= 85) {
					FUNC_ALOGI("AMSS reports GLONASS as 66-85\n");
					svi->prn = rsv->prn - 1;
				} else {
					FUNC_ALOGI("AMSS reported GLONASS satellite with PRN %d\n", rsv->prn);
					svi->prn = rsv->prn + 65 - 1;
				}
				break;
			case RPC_LOC_SV_SYSTEM_SBAS:
			case RPC_LOC_SV_SYSTEM_GALILEO:
			case RPC_LOC_SV_SYSTEM_COMPASS:
				svi->prn = rsv->prn - 120 + 33;
				break;
			default:
				FUNC_ALOGV("ignoring satellite with system %d\n", (int)rsv->system);
				continue;
			}
		} else {
			FUNC_ALOGV("ignoring satellite with no PRN\n");
			continue;
		}

		/* Android doesn't support this data for sats with prn > 32. */
		if (svi->prn >= 1 && svi->prn <= 32) {
			if ((rsv->valid_mask & RPC_LOC_SV_INFO_VALID_HAS_EPH) && rsv->has_eph)
				sv.ephemeris_mask |= 1UL << (svi->prn - 1);
			if ((rsv->valid_mask & RPC_LOC_SV_INFO_VALID_HAS_ALM) && rsv->has_alm)
				sv.almanac_mask |= 1UL << (svi->prn - 1);
			if ((rsv->valid_mask & RPC_LOC_SV_INFO_VALID_PROCESS_STATUS) && rsv->process_status == RPC_LOC_SV_STATUS_TRACK)
				sv.used_in_fix_mask |= 1UL << (svi->prn - 1);
		}

		svi->snr = (rsv->valid_mask & RPC_LOC_SV_INFO_VALID_SNR) ? rsv->snr : 0;
		svi->elevation = (rsv->valid_mask & RPC_LOC_SV_INFO_VALID_ELEVATION) ? rsv->elevation : 0;
		svi->azimuth = (rsv->valid_mask & RPC_LOC_SV_INFO_VALID_AZIMUTH) ? rsv->azimuth : 0;

		++sv.num_svs;
	}

	FUNC_ALOGV("got %u(%u listed) satellites, notifying of %u satellites\n", (unsigned)r->sv_list.sv_list_len, (unsigned)r->sv_count, (unsigned)sv.num_svs);

	CB(cb, sv_status_cb, (&sv));
}

static void server_handle_NMEA_POSITION_REPORT(struct gps_server_event *q)
{
	const rpc_loc_nmea_report_s_type *r =
		&q->payload.rpc_loc_event_payload_u_type_u.nmea_report;

#if AMSS_VERSION==3200 || AMSS_VERSION==20000
	const char *s = r->nmea_sentences.nmea_sentences_val;
	unsigned int len = r->nmea_sentences.nmea_sentences_len;
#else
	const char *s = r->nmea_sentences;
	unsigned int len = r->length;
#endif

	if (len <= 1) {
		FUNC_ALOGV("empty NMEA\n");
		return;
	}

	GpsUtcTime utc;
	SERVER_LOCK();
	utc = gps_server.gps.timestamp;
	SERVER_UNLOCK();
	if (!utc) utc = 0/*android::elapsedRealtime()*/;

	FUNC_ALOGV("reporting NMEA @ %llu: %s\n", (unsigned long long)utc, s);

	CB(cb, nmea_cb, (utc, s, len));
}

static void server_handle_NI_NOTIFY_VERIFY_REQUEST(struct gps_server_event *q)
{
	FUNC_ALOGW("received NI notify verify request");
}

static void server_handle_ASSISTANCE_DATA_REQUEST(struct gps_server_event *q)
{
	const rpc_loc_assist_data_request_s_type *r =
		&q->payload.rpc_loc_event_payload_u_type_u.assist_data_request;

	switch (r->event) {
	case RPC_LOC_ASSIST_DATA_TIME_REQ: {
		/* ignore servers/delay_threshhold */
		FUNC_ALOGV("requesting utc time\n");
		CB(cb, request_utc_time_cb, ());
		break; }
	case RPC_LOC_ASSIST_DATA_PREDICTED_ORBITS_REQ: {
		const rpc_loc_predicted_orbits_data_source_s_type *p =
			&r->payload.rpc_loc_assist_data_request_payload_u_type_u.data_download;
		SERVER_LOCK();
		gps_server.xtra.max_file_size = p->max_file_size;
		gps_server.xtra.max_part_size = p->max_part_size;
		/* ignore servers. */
		SERVER_UNLOCK();
		FUNC_ALOGV("requesting xtra data\n");
		CB(xtra_cb, download_request_cb, ());
		break; }
	default:
		FUNC_ALOGW("unknown assistance data request type %d\n", r->event);
		break;
	}
}

static void server_handle_LOCATION_SERVER_REQUEST(struct gps_server_event *q)
{
	const rpc_loc_server_request_s_type *r =
		&q->payload.rpc_loc_event_payload_u_type_u.loc_server_request;

	switch (r->event) {
	case RPC_LOC_SERVER_REQUEST_OPEN: {
		rpc_loc_server_protocol_e_type protocol = r->payload.rpc_loc_server_request_u_type_u.open_req.protocol;
		FUNC_ALOGV("requesting location server (proto %d)\n", (int)protocol);
		if (protocol != RPC_LOC_SERVER_PROTOCOL_DEFAULT &&
		    protocol != RPC_LOC_SERVER_PROTOCOL_SUPL)
			break;
		SERVER_LOCK();
		gps_server.agps.conn_handle = r->payload.rpc_loc_server_request_u_type_u.open_req.conn_handle;
		gps_server.agps.status.status = GPS_REQUEST_AGPS_DATA_CONN;
		CB(agps_cb, status_cb, (&gps_server.agps.status));
		SERVER_UNLOCK();
		break; }
	case RPC_LOC_SERVER_REQUEST_CLOSE: {
		FUNC_ALOGV("releasing location server\n");
		SERVER_LOCK();
		if (gps_server.agps.conn_handle == r->payload.rpc_loc_server_request_u_type_u.open_req.conn_handle) {
			gps_server.agps.status.status = GPS_RELEASE_AGPS_DATA_CONN;
			CB(agps_cb, status_cb, (&gps_server.agps.status));
		}
		/* FIXME: Send fail response back? */
		SERVER_UNLOCK();
		break; }
	default:
		FUNC_ALOGW("unknown location server request type %d\n", r->event);
		break;
	}
}

static void server_handle_ENGINE_STATE(const rpc_loc_engine_state_e_type *state)
{
	SERVER_LOCK();
	gps_server.engine_state = *state;
	SERVER_UNLOCK();
	FUNC_ALOGV("engine state = %d\n", (int)*state);
}

static void server_handle_FIX_SESSION_STATE(const rpc_loc_fix_session_state_e_type *state)
{
	SERVER_LOCK();
	gps_server.session_state = *state;
	if (*state == RPC_LOC_FIX_SESSION_STATE_END)
		gps_server.gps.timestamp = 0;
	FUNC_ALOGV("session state = %d\n", (int)*state);
	SERVER_UNLOCK();
}

static void server_handle_IOCTL_REPORT(struct gps_server_event *q)
{
	const rpc_loc_ioctl_callback_s_type *r =
		&q->payload.rpc_loc_event_payload_u_type_u.ioctl_report;

	switch (r->type) {
	/* Store information about server state for these */
	case RPC_LOC_IOCTL_GET_API_VERSION: {
		const rpc_loc_api_version_s_type *api =
			&r->data.rpc_loc_ioctl_callback_data_u_type_u.api_version;
		SERVER_LOCK();
		gps_server.api.major = api->major;
		gps_server.api.minor = api->minor;
		SERVER_UNLOCK();
		FUNC_ALOGV("api version = 0x%04x.0x%04x\n", (unsigned)api->major, (unsigned)api->minor);
		break; }

	case RPC_LOC_IOCTL_GET_FIX_CRITERIA: {
		const rpc_loc_fix_criteria_s_type *fix_criteria =
			&r->data.rpc_loc_ioctl_callback_data_u_type_u.fix_criteria;
		if (r->status == RPC_LOC_API_SUCCESS) {
			SERVER_LOCK();
			gps_server.fix_criteria = *fix_criteria;
			SERVER_UNLOCK();
		} else FUNC_ALOGW("GET_FIX_CRITERIA failed (%lu)\n", r->status);
		break; }

	case RPC_LOC_IOCTL_INJECT_PREDICTED_ORBITS_DATA: {
		SERVER_LOCK();
		gps_server.xtra.uploaded = (r->status == RPC_LOC_API_SUCCESS);
		gps_server.xtra.error = r->status;
		SERVER_UNLOCK();
		if (r->status != RPC_LOC_API_SUCCESS)
			FUNC_ALOGW("XTRA upload failed (%lu)\n", r->status);
		else
			FUNC_ALOGV("XTRA upload succeeded\n");
		break; }

	case RPC_LOC_IOCTL_QUERY_PREDICTED_ORBITS_DATA_VALIDITY: {
		const rpc_loc_predicted_orbits_data_validity_report_s_type *valid =
			&r->data.rpc_loc_ioctl_callback_data_u_type_u.predicted_orbits_data_validity;
		if (r->status == RPC_LOC_API_SUCCESS) {
			SERVER_LOCK();
			gps_server.xtra.valid_start = valid->start_time_utc;
			gps_server.xtra.valid_hrs = valid->valid_duration_hrs;
			SERVER_UNLOCK();
			FUNC_ALOGV("XTRA valid for %uhrs from %llu\n", (unsigned)valid->valid_duration_hrs, (unsigned long long)valid->start_time_utc);
		} else FUNC_ALOGW("QUERY_PREDICTED_ORBITS_DATA_VALIDITY failed (%lu)\n", r->status);
		break; }

	case RPC_LOC_IOCTL_GET_ENGINE_LOCK: {
		const rpc_loc_lock_e_type *lock =
			&r->data.rpc_loc_ioctl_callback_data_u_type_u.engine_lock;
		if (r->status == RPC_LOC_API_SUCCESS) {
			SERVER_LOCK();
			gps_server.engine_lock = *lock;
			SERVER_UNLOCK();
			FUNC_ALOGV("engine lock = %d\n", (int)*lock);
		} else FUNC_ALOGW("GET_ENGINE_LOCK failed (%lu)\n", r->status);
		break; }

	case RPC_LOC_IOCTL_GET_SBAS_CONFIG: {
		const rpc_boolean *mode =
			&r->data.rpc_loc_ioctl_callback_data_u_type_u.sbas_mode;
		if (r->status == RPC_LOC_API_SUCCESS) {
			SERVER_LOCK();
			gps_server.sbas_enabled = *mode;
			SERVER_UNLOCK();
			FUNC_ALOGV("SBAS = %d\n", (int)*mode);
		} else FUNC_ALOGW("GET_SBAS_CONFIG failed (%lu)\n", r->status);
		break; }

	case RPC_LOC_IOCTL_GET_NMEA_TYPES: {
		const rpc_loc_nmea_sentence_type *types =
			&r->data.rpc_loc_ioctl_callback_data_u_type_u.nmea_types;
		if (r->status == RPC_LOC_API_SUCCESS) {
			SERVER_LOCK();
			gps_server.nmea_types = *types;
			SERVER_UNLOCK();
			FUNC_ALOGV("NMEA types = 0x%08x\n", (unsigned)*types);
		} else FUNC_ALOGW("GET_NMEA_TYPES failed (%lu)\n", r->status);
		break; }

	case RPC_LOC_IOCTL_GET_ON_DEMAND_LPM: {
		const rpc_boolean *lpm =
			&r->data.rpc_loc_ioctl_callback_data_u_type_u.on_demand_lpm;
		if (r->status == RPC_LOC_API_SUCCESS) {
			SERVER_LOCK();
			gps_server.lpm = *lpm;
			SERVER_UNLOCK();
			FUNC_ALOGV("on demand LPM = %d\n", (int)*lpm);
		} else FUNC_ALOGW("GET_ON_DEMAND_LPM failed (%lu)\n", r->status);
		break; }

	/* We only care if these fail. */
	case RPC_LOC_IOCTL_SET_FIX_CRITERIA:
	case RPC_LOC_IOCTL_INFORM_NI_USER_RESPONSE:
	case RPC_LOC_IOCTL_SET_PREDICTED_ORBITS_DATA_AUTO_DOWNLOAD:
	case RPC_LOC_IOCTL_INJECT_UTC_TIME:
	case RPC_LOC_IOCTL_INJECT_RTC_VALUE:
	case RPC_LOC_IOCTL_INJECT_POSITION:
	case RPC_LOC_IOCTL_QUERY_ENGINE_STATE:
	case RPC_LOC_IOCTL_INFORM_SERVER_OPEN_STATUS:
	case RPC_LOC_IOCTL_INFORM_SERVER_CLOSE_STATUS:
	case RPC_LOC_IOCTL_SEND_WIPER_POSITION_REPORT:
	case RPC_LOC_IOCTL_NOTIFY_WIPER_STATUS:
	case RPC_LOC_IOCTL_SET_ENGINE_LOCK:
	case RPC_LOC_IOCTL_SET_SBAS_CONFIG:
	case RPC_LOC_IOCTL_SET_NMEA_TYPES:
	case RPC_LOC_IOCTL_SET_CDMA_PDE_SERVER_ADDR:
	case RPC_LOC_IOCTL_SET_CDMA_MPC_SERVER_ADDR:
	case RPC_LOC_IOCTL_SET_UMTS_SLP_SERVER_ADDR:
	case RPC_LOC_IOCTL_SET_ON_DEMAND_LPM:
	case RPC_LOC_IOCTL_DELETE_ASSIST_DATA:
	case RPC_LOC_IOCTL_SET_CUSTOM_PDE_SERVER_ADDR:
		if (r->status != RPC_LOC_API_SUCCESS)
			FUNC_ALOGW("ioctl %d failed (%lu)\n", r->type, r->status);
		else
			FUNC_ALOGV("ioctl %d succeeded\n", r->type);
		break;

	/* We don't ever send these, so shouldn't ever get a reply for them. */
	case RPC_LOC_IOCTL_QUERY_PREDICTED_ORBITS_DATA_SOURCE:
		/* RPC_LOC_IOCTL_QUERY_PREDICTED_ORBITS_DATA_SOURCE doesn't send a reply in AMSS 1240;
		   it just makes the server send an XTRA request. */
	case RPC_LOC_IOCTL_ACCESS_EFS_DATA:
	case RPC_LOC_IOCTL_GET_CDMA_PDE_SERVER_ADDR:
	case RPC_LOC_IOCTL_GET_CDMA_MPC_SERVER_ADDR:
	case RPC_LOC_IOCTL_GET_UMTS_SLP_SERVER_ADDR:
	case RPC_LOC_IOCTL_GET_CUSTOM_PDE_SERVER_ADDR:
		FUNC_ALOGE("unexpected reply to ioctl %d (status %lu)\n", r->type, r->status);
		break;
	}
}

static void server_handle_STATUS_REPORT(struct gps_server_event *q)
{
	const rpc_loc_status_event_s_type *r =
		&q->payload.rpc_loc_event_payload_u_type_u.status_report;

	if (r->event == RPC_LOC_STATUS_EVENT_ENGINE_STATE) {
		server_handle_ENGINE_STATE(&r->payload.rpc_loc_status_event_payload_u_type_u.engine_state);
	} else if (r->event == RPC_LOC_STATUS_EVENT_FIX_SESSION_STATE) {
		server_handle_FIX_SESSION_STATE(&r->payload.rpc_loc_status_event_payload_u_type_u.fix_session_state);
	} else {
		FUNC_ALOGE("unexpected status report %d (AMSS confused?)\n", r->event);
	}
}

static void server_handle_WPS_NEEDED_REQUEST(struct gps_server_event *q)
{
	FUNC_ALOGW("received WIPER request\n");
}

/* server_recv_thread: processes reports from the server and sends callbacks
   to the client */
void server_recv_thread(void *arg)
{
	TRACEV_ENTER();

	while (1) {
		unsigned head, tail, hidx;
		EVENT_LOCK();
		if (gps_server.event.head == gps_server.event.tail)
			CB(cb, release_wakelock_cb, ());
		while ((head = gps_server.event.head) == (tail = gps_server.event.tail) && (gps_client.status != GPS_STATUS_NONE))
			EVENT_WAIT();
		EVENT_UNLOCK();

		if (gps_client.status == GPS_STATUS_NONE)
			break;

		/* head != tail */

		hidx = head % GPS_SERVER_EVENT_COUNT;
		struct gps_server_event *q = &gps_server.event.q[hidx];

		FUNC_ALOGV("processing event 0x%08llx\n", (unsigned long long)q->event);

		switch (q->event) {
		case RPC_LOC_EVENT_PARSED_POSITION_REPORT:   server_handle_PARSED_POSITION_REPORT(q); break;
		case RPC_LOC_EVENT_SATELLITE_REPORT:         server_handle_SATELLITE_REPORT(q); break;
		case RPC_LOC_EVENT_NMEA_1HZ_REPORT:          /* fallthrough */
		case RPC_LOC_EVENT_NMEA_POSITION_REPORT:     server_handle_NMEA_POSITION_REPORT(q); break;
		case RPC_LOC_EVENT_NI_NOTIFY_VERIFY_REQUEST: server_handle_NI_NOTIFY_VERIFY_REQUEST(q); break;
		case RPC_LOC_EVENT_ASSISTANCE_DATA_REQUEST:  server_handle_ASSISTANCE_DATA_REQUEST(q); break;
		case RPC_LOC_EVENT_LOCATION_SERVER_REQUEST:  server_handle_LOCATION_SERVER_REQUEST(q); break;
		case RPC_LOC_EVENT_IOCTL_REPORT:             server_handle_IOCTL_REPORT(q); break;
		case RPC_LOC_EVENT_STATUS_REPORT:            server_handle_STATUS_REPORT(q); break;
		case RPC_LOC_EVENT_WPS_NEEDED_REQUEST:       server_handle_WPS_NEEDED_REQUEST(q); break;
		default: FUNC_ALOGE("unhandled event 0x%08llx\n", (unsigned long long)q->event); break;
		}

		gps_server.event.head = head + 1;
	}

	TRACEV_LEAVE();
}

static int server_send_TIME(void)
{
	rpc_loc_ioctl_data_u_type ioctl_data;
	rpc_loc_assist_data_time_s_type *data =
		&ioctl_data.rpc_loc_ioctl_data_u_type_u.assistance_data_time;
	ioctl_data.disc = RPC_LOC_IOCTL_INJECT_UTC_TIME;
	data->time_utc = gps_client.time.time;
	data->uncertainty = gps_client.time.unc + 1;
	data->time_utc += 0/*(rpc_uint64)(android::elapsedRealtime() - gps_client.time.ref)*/;
	return loc_ioctl(gps_server.handle, ioctl_data.disc, &ioctl_data);
}

static int server_send_LOC(void)
{
	rpc_loc_ioctl_data_u_type ioctl_data;
	rpc_loc_assist_data_pos_s_type *data =
		&ioctl_data.rpc_loc_ioctl_data_u_type_u.assistance_data_position;
	ioctl_data.disc = RPC_LOC_IOCTL_INJECT_POSITION;
	data->latitude = gps_client.loc.latitude;
	data->longitude = gps_client.loc.longitude;
	data->hor_unc_circular = gps_client.loc.accuracy;
	data->confidence_horizontal = 63;
	data->valid_mask = \
		RPC_LOC_ASSIST_POS_VALID_LATITUDE |
		RPC_LOC_ASSIST_POS_VALID_LONGITUDE |
		RPC_LOC_ASSIST_POS_VALID_HOR_UNC_CIRCULAR |
		RPC_LOC_ASSIST_POS_VALID_CONFIDENCE_HORIZONTAL;
	return loc_ioctl(gps_server.handle, ioctl_data.disc, &ioctl_data);
}

static int server_send_XTRA(void)
{
	rpc_loc_ioctl_data_u_type ioctl_data;
	rpc_loc_predicted_orbits_data_s_type *data =
		&ioctl_data.rpc_loc_ioctl_data_u_type_u.predicted_orbits_data;
	unsigned int part_len, i, parts, tries;
	struct timeval tv;

	gettimeofday(&tv, NULL);
	tv.tv_sec += XTRA_UPLOAD_TIMEOUT;

	ioctl_data.disc = RPC_LOC_IOCTL_INJECT_PREDICTED_ORBITS_DATA;

	data->format_type = RPC_LOC_PREDICTED_ORBITS_XTRA;
	data->total_size = gps_client.xtra.len;

	part_len = gps_server.xtra.max_part_size;
	if (!part_len) part_len = 1024;
	data->total_parts = parts = (gps_client.xtra.len + part_len - 1) / part_len;
	FUNC_ALOGV("sending xtra %u*%u byte parts\n", parts, part_len);
	tries = 0;
retry:
	for (i = 1; i <= parts; ++i) {
		int ret;

		data->part = i;
		if (i == parts) {
			data->part_len = data->total_size % part_len;
			if (!data->part_len) data->part_len = part_len;
		} else {
			data->part_len = part_len;
		}
		data->data_ptr.data_ptr_val = gps_client.xtra.data + part_len*(i-1);
		data->data_ptr.data_ptr_len = data->part_len;
		ret = loc_ioctl(gps_server.handle, ioctl_data.disc, &ioctl_data);
		if (ret != RPC_LOC_API_SUCCESS) {
			if (!tries++ || ret != RPC_LOC_API_ENGINE_BUSY)
				FUNC_ALOGW("XTRA injection failed on part %d/%d (%d)\n", i, parts, ret);

			if (ret == RPC_LOC_API_ENGINE_BUSY) {
				struct timeval now;
				gettimeofday(&now, NULL);
				if (now.tv_sec < tv.tv_sec || (now.tv_sec == tv.tv_sec && now.tv_usec < tv.tv_usec))
					goto retry;
			}
			return -1;
		} else {
			if (tries) {
				FUNC_ALOGW("uploaded part %u/%u (%u bytes, %d retries)\n", i, parts, data->part_len, tries);
			} else {
				FUNC_ALOGV("uploaded part %u/%u (%u bytes)\n", i, parts, data->part_len);
			}
		}
	}

	return 0;
}

static int server_send_DELETE_AIDING(void)
{
	rpc_loc_ioctl_data_u_type ioctl_data;
	rpc_loc_assist_data_delete_s_type *data =
		&ioctl_data.rpc_loc_ioctl_data_u_type_u.assist_data_delete;
	ioctl_data.disc = RPC_LOC_IOCTL_DELETE_ASSIST_DATA;
	memset(&data->reserved[0], 0, sizeof(data->reserved));
	if (gps_client.delete_aiding == GPS_DELETE_ALL) {
		data->type = RPC_LOC_ASSIST_DATA_ALL;
	} else {
		/* Android's flags match qcom's; mask out unallocated Android flags though,
		   because we don't know if they'll be allocated in the future. */
		data->type = gps_client.delete_aiding & 0x87FF;
	}
	return loc_ioctl(gps_server.handle, ioctl_data.disc, &ioctl_data);
}

static int server_send_NETWORK_STATE(void)
{
	/* no mechanism provided. */
	return 0;
}

static int server_send_NETWORK_AVAIL(void)
{
	/* no mechanism provided. */
	/* set WIPER avail, when supported? */
	return 0;
}

static int server_send_POSM(void)
{
	rpc_loc_ioctl_data_u_type ioctl_data;
	rpc_loc_fix_criteria_s_type *data =
		&ioctl_data.rpc_loc_ioctl_data_u_type_u.fix_criteria;

	ioctl_data.disc = RPC_LOC_IOCTL_SET_FIX_CRITERIA;

	/* TODO: Setting for these. */
	data->intermediate_pos_report_enabled = 0;
	data->notify_type = RPC_LOC_NOTIFY_ON_INTERVAL;
	/* for distance notify: min_distance, min_dist_sample_interval */

	if (gps_client.posm.mode == GPS_POSITION_MODE_MS_ASSISTED) {
		/* Android doesn't normally request this; obey. */
		data->preferred_operation_mode = RPC_LOC_OPER_MODE_MSA;
	} else {
		/* Android has a setting for STANDALONE/MS_BASED, but
		   doesn't properly implement it, so ignore the setting. */
		data->preferred_operation_mode = RPC_LOC_OPER_MODE_MSB;
	}

	switch (gps_client.posm.recurrence) {
	default: FUNC_ALOGW("unknown recurrence type %u\n", (unsigned)gps_client.posm.recurrence); /* fallthrough */
	case GPS_POSITION_RECURRENCE_PERIODIC: data->recurrence_type = RPC_LOC_PERIODIC_FIX; break;
	case GPS_POSITION_RECURRENCE_SINGLE: data->recurrence_type = RPC_LOC_SINGLE_FIX; break;
	}

	data->min_interval = gps_client.posm.min_interval;
	if (data->min_interval < 1000)
		data->min_interval = 1000;
	data->preferred_accuracy = gps_client.posm.preferred_accuracy;
	if (data->preferred_accuracy < 1) data->preferred_accuracy = 1;
	data->preferred_response_time = gps_client.posm.preferred_time;
	if (data->preferred_response_time < 1) data->preferred_response_time = 1;
	data->valid_mask =
		RPC_LOC_FIX_CRIT_VALID_INTERMEDIATE_POS_REPORT_ENABLED |
		RPC_LOC_FIX_CRIT_VALID_NOTIFY_TYPE |
		RPC_LOC_FIX_CRIT_VALID_PREFERRED_OPERATION_MODE |
		RPC_LOC_FIX_CRIT_VALID_RECURRENCE_TYPE |
		RPC_LOC_FIX_CRIT_VALID_MIN_INTERVAL |
		RPC_LOC_FIX_CRIT_VALID_PREFERRED_ACCURACY |
		RPC_LOC_FIX_CRIT_VALID_PREFERRED_RESPONSE_TIME;

	return loc_ioctl(gps_server.handle, ioctl_data.disc, &ioctl_data);
}

static int server_send_AGPS(void)
{
	rpc_loc_ioctl_data_u_type ioctl_data;

	switch (gps_client.agps.status) {
	case AGPS_STATUS_FAILED:
	case AGPS_STATUS_OPEN: {
		rpc_loc_server_open_status_s_type *data =
			&ioctl_data.rpc_loc_ioctl_data_u_type_u.conn_open_status;
		data->conn_handle = gps_server.agps.conn_handle;
		data->open_status = (gps_client.agps.status == AGPS_STATUS_OPEN) ? RPC_LOC_SERVER_OPEN_SUCCESS : RPC_LOC_SERVER_OPEN_FAIL;
		strlcpy(data->apn_name, gps_client.agps.apn, sizeof(data->apn_name));
		ioctl_data.disc = RPC_LOC_IOCTL_INFORM_SERVER_OPEN_STATUS;
		break; }
	case AGPS_STATUS_CLOSED: {
		rpc_loc_server_close_status_s_type *data =
			&ioctl_data.rpc_loc_ioctl_data_u_type_u.conn_close_status;
		data->conn_handle = gps_server.agps.conn_handle;
		data->close_status = RPC_LOC_SERVER_CLOSE_SUCCESS;
		ioctl_data.disc = RPC_LOC_IOCTL_INFORM_SERVER_CLOSE_STATUS;
		break; }
	default: return -1;
	}

	return loc_ioctl(gps_server.handle, ioctl_data.disc, &ioctl_data);
}

static int server_send_AGPS_SUPL(void)
{
	rpc_loc_ioctl_data_u_type ioctl_data;
	rpc_loc_server_info_s_type *data =
		&ioctl_data.rpc_loc_ioctl_data_u_type_u.server_addr;
	char portname[20];
	struct addrinfo hint, *res;
	int ret;

	ioctl_data.disc = RPC_LOC_IOCTL_SET_UMTS_SLP_SERVER_ADDR;
	snprintf(portname, sizeof(portname), "%u", gps_client.agps_supl.port);

	memset(&hint, 0, sizeof(hint));
	hint.ai_flags = AI_NUMERICHOST | AI_NUMERICSERV;
	hint.ai_family = AF_INET; /* AMSS doesn't support IPv6 */
	ret = getaddrinfo(gps_client.agps_supl.hostname, portname, &hint, &res);
	if (!ret) {
		struct sockaddr_in *sin = (struct sockaddr_in *)res->ai_addr;
		data->addr_type = RPC_LOC_SERVER_ADDR_IPV4;
		data->addr_info.rpc_loc_server_addr_u_type_u.ipv4.addr = sin->sin_addr.s_addr;
		data->addr_info.rpc_loc_server_addr_u_type_u.ipv4.port = sin->sin_port;
		freeaddrinfo(res);
	} else {
		data->addr_type = RPC_LOC_SERVER_ADDR_URL;
		strlcpy(data->addr_info.rpc_loc_server_addr_u_type_u.url.addr, gps_client.agps_supl.hostname, sizeof(data->addr_info.rpc_loc_server_addr_u_type_u.url.addr));
		data->addr_info.rpc_loc_server_addr_u_type_u.url.length = strlen(data->addr_info.rpc_loc_server_addr_u_type_u.url.addr);
	}
	return loc_ioctl(gps_server.handle, ioctl_data.disc, &ioctl_data);
}

static int server_send_REF_LOC(void)
{
	/* no mechanism provided. */
	FUNC_ALOGV("not supported\n");
	return 0;
}

static int server_send_SET_ID(void)
{
	/* no mechanism provided. */
	FUNC_ALOGV("not supported\n");
	return 0;
}

static int server_send_RELOAD(void)
{
	rpc_loc_ioctl_data_u_type ioctl_data;

#if LIBLOC_USE_GPS_PRIVACY_LOCK
	/* engine lock */
	ioctl_data.disc = RPC_LOC_IOCTL_SET_ENGINE_LOCK;
	ioctl_data.rpc_loc_ioctl_data_u_type_u.engine_lock = RPC_LOC_LOCK_NONE;
	loc_ioctl(gps_server.handle, ioctl_data.disc, &ioctl_data);
#endif

	/* sbas */
	ioctl_data.disc = RPC_LOC_IOCTL_SET_SBAS_CONFIG;
	ioctl_data.rpc_loc_ioctl_data_u_type_u.sbas_mode = TRUE;
	loc_ioctl(gps_server.handle, ioctl_data.disc, &ioctl_data);

	/* nmea */
	ioctl_data.disc = RPC_LOC_IOCTL_SET_NMEA_TYPES;
	ioctl_data.rpc_loc_ioctl_data_u_type_u.nmea_types = RPC_LOC_NMEA_MASK_ALL;
	loc_ioctl(gps_server.handle, ioctl_data.disc, &ioctl_data);

	/* lpm? */

	/* wiper unavail */
	ioctl_data.disc = RPC_LOC_IOCTL_NOTIFY_WIPER_STATUS;
	ioctl_data.rpc_loc_ioctl_data_u_type_u.wiper_status = RPC_LOC_WIPER_STATUS_UNAVAILABLE;
	loc_ioctl(gps_server.handle, ioctl_data.disc, &ioctl_data);

	return 0;
}

static int server_send_STATUS(void)
{
	int ret;

	if (gps_client.status == GPS_STATUS_ENGINE_ON ||
	    gps_client.status == GPS_STATUS_SESSION_BEGIN) {
		SERVER_LOCK();
		if (gps_server.status.status == GPS_STATUS_ENGINE_OFF) {
			const rpc_loc_event_mask_type events =
				RPC_LOC_EVENT_PARSED_POSITION_REPORT |
				RPC_LOC_EVENT_SATELLITE_REPORT |
				RPC_LOC_EVENT_NMEA_1HZ_REPORT |
				RPC_LOC_EVENT_NMEA_POSITION_REPORT |
				/* RPC_LOC_EVENT_NI_NOTIFY_VERIFY_REQUEST | */
				RPC_LOC_EVENT_ASSISTANCE_DATA_REQUEST |
				RPC_LOC_EVENT_LOCATION_SERVER_REQUEST |
				RPC_LOC_EVENT_IOCTL_REPORT |
				RPC_LOC_EVENT_STATUS_REPORT |
				/* RPC_LOC_EVENT_WPS_NEEDED_REQUEST | */
				0;

			gps_server.handle = loc_open(events, server_loc_event_cb);
			if (gps_server.handle >= 0) {
				gps_server.status.status = GPS_STATUS_ENGINE_ON;
				SERVER_UNLOCK();
				FUNC_ALOGI("engine on\n");

				/* load initial config, supply any xtra, etc */
				gps_client.updated_flags |=
					GPS_CLIENT_UPDATED_RELOAD |
					GPS_CLIENT_UPDATED_POSM |
					GPS_CLIENT_UPDATED_XTRA |
					GPS_CLIENT_UPDATED_AGPS_SUPL;

				if (gps_client.status == GPS_STATUS_SESSION_BEGIN)
					gps_client.updated_flags |= GPS_CLIENT_UPDATED_STATUS;

				CLIENT_UNLOCK();
				CB(cb, status_cb, (&gps_server.status));
				CLIENT_LOCK();
				return 0;
			} else {
				FUNC_ALOGE("loc_open returned %ld\n", (long)gps_server.handle);
				SERVER_UNLOCK();
				return 0;
			}
		}
		SERVER_UNLOCK();
	}

	if (gps_client.status == GPS_STATUS_SESSION_BEGIN) {
		SERVER_LOCK();
		if (gps_server.status.status == GPS_STATUS_ENGINE_ON) {
			ret = loc_start_fix(gps_server.handle);
			if (ret == RPC_LOC_API_SUCCESS) {
				gps_server.status.status = GPS_STATUS_SESSION_BEGIN;
				SERVER_UNLOCK();
				FUNC_ALOGI("begin navigation\n");
				CLIENT_UNLOCK();
				CB(cb, status_cb, (&gps_server.status));
				CLIENT_LOCK();
			} else FUNC_ALOGW("loc_start_fix returned %d\n", ret);
		} else if (gps_server.status.status != GPS_STATUS_SESSION_BEGIN) {
			FUNC_ALOGW("SESSION_BEGIN with engine status %d\n", gps_server.status.status);
			SERVER_UNLOCK();
			return 0;
		}
	} else if (gps_client.status == GPS_STATUS_ENGINE_ON) {
		SERVER_LOCK();
		if (gps_server.status.status == GPS_STATUS_SESSION_BEGIN) {
			ret = loc_stop_fix(gps_server.handle);
			if (ret == RPC_LOC_API_SUCCESS) {
				gps_server.status.status = GPS_STATUS_ENGINE_ON;
				SERVER_UNLOCK();
				FUNC_ALOGI("end navigation\n");

				/* Client doesn't update navigation status for GPS_STATUS_ENGINE_ON */
				GpsStatus gps_notify_status = {
					.size = sizeof(GpsStatus),
					.status = GPS_STATUS_SESSION_END,
				};
				CLIENT_UNLOCK();
				CB(cb, status_cb, (&gps_notify_status));
				CLIENT_LOCK();
			} else FUNC_ALOGW("loc_stop_fix return %d\n", ret);
		} else if (gps_server.status.status != GPS_STATUS_ENGINE_ON) {
			FUNC_ALOGW("SESSION_END with engine status %d\n", gps_server.status.status);
			SERVER_UNLOCK();
			return 0;
		}
	} else if (gps_client.status == GPS_STATUS_ENGINE_OFF ||
	           gps_client.status == GPS_STATUS_NONE) {
		SERVER_LOCK();
		if (gps_server.status.status != GPS_STATUS_ENGINE_OFF) {
			do {
				ret = loc_close(gps_server.handle);
				gps_server.handle = -1;
				gps_server.status.status = GPS_STATUS_ENGINE_OFF;
				if (ret && ret != RPC_LOC_API_ENGINE_BUSY)
					FUNC_ALOGE("loc_close returned %d\n", ret);
			} while (ret == RPC_LOC_API_ENGINE_BUSY);
			SERVER_UNLOCK();
			FUNC_ALOGI("engine off\n");
			CLIENT_UNLOCK();
			CB(cb, status_cb, (&gps_server.status));
			CLIENT_LOCK();
		} else {
			SERVER_UNLOCK();
			return 0;
		}
	}

	return 0;
}

/* server_send_thread: checks gps_client for any changes the server
   needs to reconfigure itself for, and sends the request to the server */
void server_send_thread(void *arg)
{
	int requested_xtra = 0;
	TRACEV_ENTER();

	while (!loc_api_glue_init()) {
		FUNC_ALOGW("loc_api_glue_init() failed, trying again...\n");
		sleep(1);
	}

	CLIENT_LOCK();
	while (1) {
		int updated_flags;
		int engine_off;
#if AMSS_VERSION==1240
		int moving_to_nav = 0;
#endif

		while (!gps_client.updated_flags)
			pthread_cond_wait(&gps_client.updated_cond, &gps_client.lock);

		updated_flags = gps_client.updated_flags;
		gps_client.updated_flags = 0;

#define SERVER_CHECK_SEND(flag) do { \
		if (updated_flags & GPS_CLIENT_UPDATED_ ## flag ) \
			server_send_ ## flag (); \
	} while(0)

		if (gps_client.status == GPS_STATUS_NONE) {
			TRACEV("shutting down\n");
			server_send_STATUS();
			break;
		}

		SERVER_LOCK();
		engine_off = (gps_server.status.status == GPS_STATUS_ENGINE_OFF);
#if AMSS_VERSION==1240
		if (updated_flags & GPS_CLIENT_UPDATED_STATUS)
			moving_to_nav = (gps_server.status.status != GPS_STATUS_SESSION_BEGIN && gps_client.status == GPS_STATUS_SESSION_BEGIN);
#endif
		SERVER_UNLOCK();

		FUNC_ALOGV("client updated: 0x%08x\n", updated_flags);

		if (engine_off) {
			SERVER_CHECK_SEND(STATUS);
			continue;
		}

		if (!requested_xtra && !gps_client.xtra.len) {
			requested_xtra = 1;
			CLIENT_UNLOCK();
			CB(xtra_cb, download_request_cb, ());
			CLIENT_LOCK();
		} else if (gps_client.xtra.len) {
			requested_xtra = 0;
		}

#if AMSS_VERSION==1240
		if (moving_to_nav)
			updated_flags |= GPS_CLIENT_UPDATED_XTRA | GPS_CLIENT_UPDATED_AGPS_SUPL;
#endif

		SERVER_CHECK_SEND(RELOAD);
		SERVER_CHECK_SEND(TIME);
		SERVER_CHECK_SEND(LOC);
		SERVER_CHECK_SEND(XTRA);
		SERVER_CHECK_SEND(DELETE_AIDING);

		SERVER_CHECK_SEND(NETWORK_STATE);
		SERVER_CHECK_SEND(NETWORK_AVAIL);

		SERVER_CHECK_SEND(POSM);
		SERVER_CHECK_SEND(AGPS);
		SERVER_CHECK_SEND(AGPS_SUPL);
		SERVER_CHECK_SEND(REF_LOC);
		SERVER_CHECK_SEND(SET_ID);

		SERVER_CHECK_SEND(STATUS);
	}

	SERVER_LOCK();
	gps_server.status.status = GPS_STATUS_NONE;
	SERVER_UNLOCK();

	CLIENT_UNLOCK();

	TRACEV_LEAVE();
}
