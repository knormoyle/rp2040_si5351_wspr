#pragma once

#include "WsprMessageTelemetryExtendedCommon.h"
#include "WsprMessageTelemetryExtendedVendorDefinedBase.h"


template <uint8_t FIELD_COUNT = 29>
class WsprMessageTelemetryExtendedVendorDefined
: public WsprMessageTelemetryExtendedVendorDefinedBase<WsprMessageTelemetryExtendedCommon<FIELD_COUNT>, FIELD_COUNT>
{
};
