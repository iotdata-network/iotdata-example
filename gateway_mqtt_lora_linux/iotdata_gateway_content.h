#ifndef IOTDATA_GATEWAY_CONTENT_H
#define IOTDATA_GATEWAY_CONTENT_H

#include "d_module_filestore_linux.h"
#include "iotdata_node_content_control.h"

// -----------------------------------------------------------------------------------------------------------------------------------------
// THE CONTENT STORE
//
// A directory of bundles, the store above it, and the words that read it. The gateway is the end of
// the line content arrives from rather than a node that consumes any: nothing here registers a
// consumer, so every file is unowned, which the listing says rather than inventing an owner.
//
// WHICH MAKES THE INTAKE A COPY, and deliberately so. ota-bundle writes a file; putting that file
// in this directory, named as its content id in six hex digits, is the whole of loading it. The
// store walks the directory at startup and announces what it finds, so there is no upload protocol
// to speak to a machine you already have a shell on -- and the names are the same six hex digits
// the device backend uses, so the same file means the same thing at both ends of the wire.
//
// A STORE THAT WILL NOT OPEN IS NOT A STARTUP FAILURE, the same as on a relay: content stops
// working, it is said once, and the gateway carries on doing the job it is actually deployed for.
// -----------------------------------------------------------------------------------------------------------------------------------------

static filestore_linux_t gateway_filestore;
static iotdata_node_content_t gateway_content;

static void gateway_content_begin(const uint32_t now_ms) {
    const char *const dir = iotdata_node_config_string(CONTENT_FILE_DIRECTORY);
    const iotdata_node_content_backend_t *const be = filestore_linux_open(&gateway_filestore, dir, iotdata_node_config_u32(CONTENT_FILE_MAX_BYTES));
    iotdata_node_content_init(&gateway_content, be);
    /* The adapters are initialised even when the store did not open, so the commands still resolve
       and answer "no store" rather than going unrecognised -- a gateway that stopped admitting a
       word because of a missing directory would look like a gateway with older firmware. */
    iotdata_node_content_control_init(&gateway_content, NULL);
    if (be == NULL) {
        PRINTF_ERROR("content: no store -- '%s' could not be opened\n", dir);
        return;
    }
    if (!filestore_linux_persistent(&gateway_filestore))
        PRINTF_WARN("content: '%s' is a tmpfs -- nothing here will survive a reboot\n", dir);
    if (!iotdata_node_config_bool(CONTENT_ENABLED)) {
        /* Not begun, so nothing is announced and no insert can start. What is in the directory
           stays in the directory: see IT IS A GATE ON ACCEPTING in iotdata_node_config_content.h. */
        PRINTF_INFO("content: disabled (path=%s)\n", dir);
        return;
    }
    const int offered = iotdata_node_content_begin(&gateway_content, now_ms);
    uint32_t used = 0u, budget = 0u;
    (void)filestore_linux_info(&gateway_filestore, &used, &budget);
    PRINTF_INFO("content: %" PRIu32 " file(s), %d offered, %u owned, %" PRIu32 " of %" PRIu32 " bytes used (path=%s)\n", iotdata_node_content_count(&gateway_content), offered,
                (unsigned)gateway_content.owns, used, budget, dir);
}

static void gateway_content_end(void) {
    iotdata_node_content_end(&gateway_content);
    filestore_linux_close(&gateway_filestore);
}

/* Beside the recorder's tick in the main loop. There is no consumer to tick for yet, so the only
   thing this can resolve today is an insert that was abandoned -- but it is wired now rather than
   when the first consumer lands, because a store whose tick is missing fails by doing nothing. */
static void gateway_content_tick(const uint32_t now_ms) {
    const iotdata_node_content_event_t ev = iotdata_node_content_tick(&gateway_content, now_ms);
    if (ev == IOTDATA_NODE_CONTENT_EVENT_REBOOT_REQUIRED)
        PRINTF_WARN("content: a consumer asked for a restart, which this gateway does not do on its own\n");
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#endif /* IOTDATA_GATEWAY_CONTENT_H */
