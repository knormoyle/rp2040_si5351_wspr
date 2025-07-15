#pragma once

#include "WsprMessageTelemetryExtendedCommonDynamic.h"
#include "WsprMessageTelemetryExtendedVendorDefinedBase.h"


template <uint8_t FIELD_COUNT = 29>
class WsprMessageTelemetryExtendedVendorDefined
: public WsprMessageTelemetryExtendedVendorDefinedBase<WsprMessageTelemetryExtendedCommonDynamic<FIELD_COUNT>, FIELD_COUNT>
{
};
