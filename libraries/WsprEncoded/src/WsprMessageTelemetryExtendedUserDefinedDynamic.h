#pragma once

#include "WsprMessageTelemetryExtendedCommonDynamic.h"
#include "WsprMessageTelemetryExtendedUserDefinedBase.h"


template <uint8_t FIELD_COUNT = 29>
class WsprMessageTelemetryExtendedUserDefined
: public WsprMessageTelemetryExtendedUserDefinedBase<WsprMessageTelemetryExtendedCommonDynamic<FIELD_COUNT>, FIELD_COUNT>
{
};
