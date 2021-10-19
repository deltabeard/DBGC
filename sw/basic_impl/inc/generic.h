#pragma once

/* Helpers. */
#define UNUSED(x)	(void)(x)
#define ARRAYSIZE(x)	(sizeof(x)/sizeof(x[0]))

/* Optimisations. */
#define LIKELY(expr)	__builtin_expect(!!(expr), 1)
#define UNLIKELY(expr)	__builtin_expect(!!(expr), 0)
#define UNREACHABLE()	__builtin_unreachable()
