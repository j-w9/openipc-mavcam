#pragma once
#include <stddef.h>

/* Blocking HTTP/1.0 GET with a total timeout.
 * On success returns the body length (>= 0), stores the HTTP status in *status
 * and a malloc'd, NUL-terminated body in *body (caller frees).
 * Returns -1 on connect/IO failure or if the body exceeds max_body. */
long http_get(const char *host, int port, const char *path, int timeout_ms,
              size_t max_body, int *status, char **body);
