#pragma once

#include "WsprMessageTelemetryExtendedCommon.h"

#include <string>
#include <vector>


template <uint8_t FIELD_COUNT = 29>
class WsprMessageTelemetryExtendedCommonDynamic
: public WsprMessageTelemetryExtendedCommon<FIELD_COUNT>
{
    using Base = WsprMessageTelemetryExtendedCommon<FIELD_COUNT>;

public:

    WsprMessageTelemetryExtendedCommonDynamic()
    {
        fieldList_.reserve(FIELD_COUNT);
    }

    void ResetEverything()
    {
        Base::ResetEverything();

        fieldList_.clear();
    }

    bool DefineField(const char *fieldName,
                     double      lowValue,
                     double      highValue,
                     double      stepSize)
    {
        fieldList_.push_back(fieldName);

        bool ok = true;

        {
            std::string &fieldNameDynamic = fieldList_[fieldList_.size() - 1];

            ok = Base::DefineField(fieldNameDynamic.c_str(), lowValue, highValue, stepSize);
        }

        if (ok == false)
        {
            fieldList_.pop_back();
        }

        return ok;
    }

    const std::vector<std::string> &GetFieldList()
    {
        return fieldList_;
    }


private:

    std::vector<std::string> fieldList_;
};
