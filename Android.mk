ifeq ($(BOARD_USES_QCOM_GPS),true)

LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw

LOCAL_MODULE := gps.$(BOARD_VENDOR_QCOM_GPS_LOC_API_HARDWARE)
LOCAL_MODULE_TAGS := optional
LOCAL_STATIC_LIBRARIES:= \
    libloc_api-rpc 

LOCAL_SHARED_LIBRARIES := \
    librpc \
    libutils \
    libcutils

LOCAL_SRC_FILES += hal.c gps.c qcom_gps.c

LOCAL_CFLAGS += \
    -fno-short-enums \
    -DAMSS_VERSION=$(BOARD_VENDOR_QCOM_GPS_LOC_API_AMSS_VERSION)

LOCAL_C_INCLUDES:= \
	$(TARGET_OUT_HEADERS)/libloc_api-rpc \
	$(TARGET_OUT_HEADERS)/libloc_api-rpc/inc \
	$(TARGET_OUT_HEADERS)/libcommondefs-rpc/inc \
	$(TARGET_OUT_HEADERS)/librpc

ifneq ($(filter 50000 20000 1240,$(BOARD_VENDOR_QCOM_GPS_LOC_API_AMSS_VERSION)),)
    LOCAL_CFLAGS += -DLIBLOC_USE_GPS_PRIVACY_LOCK=1
endif

include $(BUILD_SHARED_LIBRARY)
endif
