#pragma once

#include "WsprMessageTelemetryExtendedCommon.h"
#include "WsprMessageTelemetryExtendedUserDefinedBase.h"


template <uint8_t FIELD_COUNT = 29>
class WsprMessageTelemetryExtendedUserDefined
: public WsprMessageTelemetryExtendedUserDefinedBase<WsprMessageTelemetryExtendedCommon<FIELD_COUNT>, FIELD_COUNT>
{
};
