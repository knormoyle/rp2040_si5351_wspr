#ifndef WSPR_ENCODED_CT_H_
#define WSPR_ENCODED_CT_H_

// Umbrella header for the WSPR encoded-message library.
//
// Note: in this rewrite there is no separate "Dynamic" header. Field arrays
// are always heap-allocated at the runtime size requested via the
// constructor, so the static-vs-dynamic split from the original codebase no
// longer applies.

#include "Wspr.h"
#include "WsprChannelMap.h"
#include "WsprMessageRegularType1.h"
#include "WsprMessageTelemetryBasic.h"
#include "WsprMessageTelemetryExtendedCommon.h"
#include "WsprMessageTelemetryExtendedUserDefined.h"

#endif  // WSPR_ENCODED_CT_H_
