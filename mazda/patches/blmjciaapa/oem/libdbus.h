// Minimal libdbus-1 bindings for the svcjcinavi HUD transport.
//
// This transport emits a com.NNG.Api.Server.Guidance.GuidanceChangedForHUD
// signal the way the NNG engine does — with raw libdbus-1 — rather than
// through libjcidbus. libjcidbus's JCIDBUS_signal_send is a SERVE-only
// path: it calls signal_registered() and refuses (returns -1, logs
// "Signal %s is not added in any Interface!") to emit any signal whose
// interface+member wasn't registered as a served object on the
// connection. We are a client impersonating NNG, not serving an
// interface, so that path can never work for us. NNG itself links
// libdbus-1 directly (no libjcidbus DT_NEEDED) and emits the bare
// signal; we do the same.
//
// As with the other oem/ bindings there is NO init step and NO link
// dependency: each prototype is backed by a self-resolving dlsym thunk
// in libdbus.cpp (see oem_thunk.h). libdbus-1.so.3 is always resident
// in the jciAAPA process (libjcidbus pulls it in), so dlopen finds it.
//
// Types and constants are hand-declared from the libdbus public ABI
// (dbus-message.h / dbus-protocol.h). The DBusMessageIter layout is
// part of libdbus's stable public ABI and is safe to reproduce.

#ifndef LIBPATCH_BLMJCIAAPA_LIBDBUS_H
#define LIBPATCH_BLMJCIAAPA_LIBDBUS_H

#include <stdint.h>

// D-Bus basic type codes (dbus-protocol.h). Only the ones we marshal.
#define DBUS_TYPE_BYTE   ((int)'y')   // 121
#define DBUS_TYPE_INT32  ((int)'i')   // 105
#define DBUS_TYPE_STRING ((int)'s')   // 115

// DBusMessageIter — stable public ABI (dbus-message.h). Opaque to us;
// we only ever pass its address. Sized via dummy fields exactly as
// the upstream header declares so the on-stack object is large enough.
struct DBusMessageIter {
    void     *dummy1;
    void     *dummy2;
    uint32_t  dummy3;
    int       dummy4;
    int       dummy5;
    int       dummy6;
    int       dummy7;
    int       dummy8;
    int       dummy9;
    int       dummy10;
    int       dummy11;
    int       pad1;
    int       pad2;
    void     *pad3;
};

// All functions resolve as dynamic exports of libdbus-1.so.3 (verified
// T in `nm -D`). Pointers are opaque (DBusConnection* / DBusMessage*).
// dbus_bool_t is a plain int (TRUE=1, FALSE=0). A NULL DBusError* is
// accepted by libdbus, so we pass none.

// Connection lifecycle. open_private gives us a dedicated connection
// (not shared with libjcidbus's); register performs the bus Hello so
// the daemon will route our signals; set_exit_on_disconnect(FALSE)
// keeps a torn service-bus socket (e.g. source switch) from raising a
// fatal signal — the same safety libjcidbus gave us.
void *dbus_connection_open_private(const char *address, void *error);
int   dbus_bus_register(void *conn, void *error);
void  dbus_connection_set_exit_on_disconnect(void *conn, int enabled);
void  dbus_connection_close(void *conn);
void  dbus_connection_unref(void *conn);

// Signal build + send.
void *dbus_message_new_signal(const char *path, const char *iface,
                              const char *member);
void  dbus_message_iter_init_append(void *msg, DBusMessageIter *iter);
int   dbus_message_iter_append_basic(DBusMessageIter *iter, int type,
                                     const void *value);
int   dbus_connection_send(void *conn, void *msg, uint32_t *serial);
void  dbus_connection_flush(void *conn);
void  dbus_message_unref(void *msg);

// Signal receive (no main loop): the mute bridge subscribes with add_match,
// then loops read_write + pop_message, checking each popped message with
// is_signal and reading its args with the iter API. All are stable
// libdbus-1.so.3 dynamic exports. dbus_bus_add_match sends the AddMatch to
// the bus; read_write returns FALSE when the connection has disconnected.
void dbus_bus_add_match(void *conn, const char *rule, void *error);
int  dbus_connection_read_write(void *conn, int timeout_milliseconds);
void *dbus_connection_pop_message(void *conn);
int  dbus_message_is_signal(void *msg, const char *iface, const char *member);
int  dbus_message_iter_init(void *msg, DBusMessageIter *iter);
int  dbus_message_iter_get_arg_type(DBusMessageIter *iter);
void dbus_message_iter_get_basic(DBusMessageIter *iter, void *value);

#endif  // LIBPATCH_BLMJCIAAPA_LIBDBUS_H
