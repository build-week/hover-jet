#pragma once

// Most people call this "FORCE_INLINE", but those people aren't very fun
#define SUPER_INLINE inline __attribute__((always_inline))
