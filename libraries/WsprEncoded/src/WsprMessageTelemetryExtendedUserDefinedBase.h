#pragma once

#include <cstdint>


template <typename WsprMessageTelemetryExtendedCommonType, uint8_t FIELD_COUNT = 29>
class WsprMessageTelemetryExtendedUserDefinedBase
: public WsprMessageTelemetryExtendedCommonType
{
    using Base = WsprMessageTelemetryExtendedCommonType;

public:

    WsprMessageTelemetryExtendedUserDefinedBase()
    {
        ResetEverything();
    }

    void ResetEverything()
    {
        Base::ResetEverything();

        // don't allow this named class to change its protocol type
        Base::SetCanSetHdrType(true);
        Base::Set("HdrType", (uint8_t)Base::HdrType::USER_DEFINED);
        Base::SetCanSetHdrType(false);
    }

    // Returns true on successful decode.
    // Returns false on error.
    //
    // An error will occur when:
    // - The HdrTelemetryType is not ExtendedTelemetry
    // - The HdrType is not of type UserDefined
    bool Decode()
    {
        bool retVal = true;
        
        retVal &= Base::Decode();
        retVal &= Base::GetHdrType() == Base::HdrType::USER_DEFINED;

        return retVal;
    }
    

private:

    // hide header setting
    using Base::SetCanSetHdrType;
};
