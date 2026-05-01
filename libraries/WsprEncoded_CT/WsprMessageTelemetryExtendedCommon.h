#ifndef WSPR_MESSAGE_TELEMETRY_EXTENDED_COMMON_H
#define WSPR_MESSAGE_TELEMETRY_EXTENDED_COMMON_H

#include <cstdint>
#include <cstring>
#include <math.h>
#include "WsprMessageTelemetryCommon.h"

// class supports a configurable number of fields at construction time. 
// field array is heap-allocated with runtime size passed to the constructor.
//
// total user-field bitspace is 35.180 bits, 
// occupying everything below HdrTelemetryType in the encoded WSPR Type 1 message.
class WsprMessageTelemetryExtendedCommon : public WsprMessageTelemetryCommon {
public:
    // 29.180 (original budget) + 6.000 freed 
    // by removing HdrRESERVED +  HdrType) = 35.180 bits available 
    // for user-defined fields.
    static constexpr double kMaxBits = 35.180;
    static const uint8_t kDefaultFieldCount = 35;

    struct FieldDef {
        const char* name;

        // field configuration
        double low_value;
        double high_value;
        double step_size;

        // calculated properties of configuration
        uint32_t num_values;
        double   num_bits;

        // dynamic value clamped to configuration
        double value;
    };

public:
    // field_count is the maximum number of user-defined fields supported.
    // user-defined field array is allocated on the heap with requested size.
    explicit WsprMessageTelemetryExtendedCommon(
            uint8_t field_count = kDefaultFieldCount)
            : field_count_(field_count),
              field_def_user_defined_list_(nullptr),
              field_def_user_defined_name_storage_(nullptr) {
        AllocateStorage();
        ResetEverything();
    }

    // A virtual function (C++) is a class member accessible to all, which derived classes can override
    // Enables runtime polymorphism. 
    // Allows a base class to define default behavior while permitting derived classes to specialize it.
    // Public virtual methods allow anyone to alter behavior, which can break encapsulation.
    virtual ~WsprMessageTelemetryExtendedCommon() {
        FreeStorage();
    }

    // Reset field values, but not field definitions.
    void Reset() {
        WsprMessageTelemetryCommon::Reset();
        // reset default field value for user-defined fields
        for (int i = 0; i < field_def_user_defined_list_idx_; ++i) {
            field_def_user_defined_list_[i].value =
                field_def_user_defined_list_[i].low_value;
        }
        // reset header values
        field_def_header_list_[0].value = 0;
        field_def_header_list_[1].value = 0;
    }

    // Reset field definitions and values.
    void ResetEverything() {
        WsprMessageTelemetryCommon::Reset();
        // zero all user-defined field defs and clear owned name buffers
        for (uint8_t i = 0; i < field_count_; ++i) {
            field_def_user_defined_list_[i] = FieldDef();
            if (field_def_user_defined_name_storage_[i] != nullptr) {
                field_def_user_defined_name_storage_[i][0] = '\0';
            }
        }
        field_def_user_defined_list_idx_ = 0;
        field_def_fail_reason_ = "";
        num_bits_sum_ = 0.0;

        // header field 0: HdrTelemetryType
        field_def_header_list_[0].name       = "HdrTelemetryType";
        field_def_header_list_[0].low_value  = 0;
        field_def_header_list_[0].high_value = 1;
        field_def_header_list_[0].step_size  = 1;
        field_def_header_list_[0].num_values = 2;
        field_def_header_list_[0].num_bits   = 1;
        field_def_header_list_[0].value      = 0;  // Extended Telemetry
        // header field 1: HdrSlot
        field_def_header_list_[1].name       = "HdrSlot";
        field_def_header_list_[1].low_value  = 0;
        field_def_header_list_[1].high_value = 4;
        field_def_header_list_[1].step_size  = 1;
        field_def_header_list_[1].num_values = 5;
        field_def_header_list_[1].num_bits   = log(5.0) / log(2.0);  // ~2.321
        field_def_header_list_[1].value      = 0;
    }


    // User-Defined Field Definitions, Setters, Getters
    // Set up this object to know about named fields with a given numeric
    // range and resolution (step size).
    //
    // Values will be clamped between low_value - high_value, inclusive.
    // Negative, zero, positive, and decimal values are all supported.
    //
    // See Set() for details on rounding.
    //
    // The initial value of a defined field will be the specified low_value.
    //
    // Returns true if field is accepted.
    // Returns false if field is rejected.
    //
    // A field will be rejected due to:
    // - The configured number of fields have already been used up
    // - The field name is a nullptr
    // - The field already exists
    // - low_value, high_value, or step_size is too precise (> 3 decimals)
    // - low_value >= high_value
    // - step_size <= 0
    // - The step_size does not evenly divide the range between low_value
    //   and high_value
    // - The field size exceeds the sum total capacity of 35.180 bits along
    //   with other fields, or by itself
    bool DefineField(const char* field_name,
                     double      low_value,
                     double      high_value,
                     double      step_size) {
        bool ret_val = true;
        if (CanFitOneMore() == false) {
            ret_val = false;
            field_def_fail_reason_ = "Can not fit another field";
        } else if (field_name == nullptr) {
            ret_val = false;
            field_def_fail_reason_ = "Field name is nullptr";
        } else if (FieldDefExists(field_name)) {
            ret_val = false;
            field_def_fail_reason_ = "Field already exists";
        } else if (FieldIsTooPrecise(low_value)) {
            ret_val = false;
            field_def_fail_reason_ = "Low value is too precise";
        } else if (FieldIsTooPrecise(high_value)) {
            ret_val = false;
            field_def_fail_reason_ = "High value is too precise";
        } else if (FieldIsTooPrecise(step_size)) {
            ret_val = false;
            field_def_fail_reason_ = "Step size is too precise";
        } else if (low_value >= high_value) {
            ret_val = false;
            field_def_fail_reason_ = "Low value >= High value";
        } else if (step_size <= 0) {
            ret_val = false;
            field_def_fail_reason_ = "Step size <= 0";
        } else {
            // Is there an integer-number number of divisions of the
            // low-to-high range when incremented by the step size?
            //
            // Because of floating point issues, is done in a way
            // that scales the (potentially) decimal numbers into pure integer space. 
            // Safe to do here because already checked that the numbers are not 
            // any more precise than the amount we scale.
            uint32_t step_count = GetStepCount(low_value, high_value, step_size);
            if (step_count == 0) {
                ret_val = false;
                field_def_fail_reason_ =
                    "Step size does not evenly divide the low-to-high range";
            } else {
                FieldDef fd;
                // known to be an integer value as checked previously
                fd.num_values = step_count + 1;
                // calc bits used by this field
                fd.num_bits =
                    log(static_cast<double>(fd.num_values)) / log(2.0);
                // check if can fit
                if (num_bits_sum_ + fd.num_bits > kMaxBits) {
                    ret_val = false;
                    field_def_fail_reason_ = "Field overflows available bits";
                } else {
                    // increment number of bits used by total set of fields
                    num_bits_sum_ += fd.num_bits;

                    // Copy the field name into our owned storage 
                    // So callers do not need to keep their string alive.
                    char* owned_name =
                        field_def_user_defined_name_storage_[
                            field_def_user_defined_list_idx_];
                    strncpy(owned_name, field_name, kFieldNameCapacity - 1);
                    owned_name[kFieldNameCapacity - 1] = '\0';

                    // capture field def values
                    fd.name       = owned_name;
                    fd.low_value  = low_value;
                    fd.high_value = high_value;
                    fd.step_size  = step_size;

                    // set initial value to known-within-range
                    fd.value = low_value;

                    // add to field def list
                    field_def_user_defined_list_[field_def_user_defined_list_idx_] = fd;
                    ++field_def_user_defined_list_idx_;
                }
            }
        }
        return ret_val;
    }

    const char* GetDefineFieldErr() const {
        return field_def_fail_reason_;
    }

    uint16_t GetFieldDefListLen() const {
        return field_def_user_defined_list_idx_;
    }

    // Return a pointer to the user-defined field def array.
    // Use GetFieldDefListLen() to know how many entries are valid.
    const FieldDef* GetFieldDefList() const {
        return field_def_user_defined_list_;
    }

    // Return the maximum number of user-defined fields this object can hold.
    uint8_t GetFieldCapacity() const {
        return field_count_;
    }

    // Set the value of a configured field.
    //
    // value parameter is double so a wide range of values easily settable, 
    // Even if you do not intend to use a floating point number.
    //
    // A value that is set is retained internally at that precise value, and
    // will be returned at that value with a subsequent Get().
    //
    // When a field is encoded, the encoded wspr data will contain the encoded value, 
    // which is rounded to the precision specified in the field definition.
    //
    // Returns true on success.
    // Returns false on error.
    // An error will occur when the field is not defined.
    bool Set(const char* field_name, double value) {
        bool ret_val = true;
        if (FieldDefExists(field_name) && IsOkToSet(field_name)) {
            FieldDef& fd = *GetFieldDef(field_name);

            // do value type validation
            if (isnan(value)) {
                value = fd.low_value;
                ret_val = false;
            } else if (isinf(value)) {
                value = fd.low_value;
                ret_val = false;
            }

            // Check for negative zero, which may still wind up getting
            // clamped.
            if (value == 0.0 && signbit(value)) {
                value = 0.0;
                ret_val = false;
            }

            // clamp to range
            if (value < fd.low_value) {
                value = fd.low_value;
                ret_val = false;
            } else if (value > fd.high_value) {
                value = fd.high_value;
                ret_val = false;
            }
            fd.value = value;

        } else {
            ret_val = false;
        }
        return ret_val;
    }

    // Get the value of a configured field.
    //
    // When a field is Set() then Get(), the value which was Set() will be
    // returned by Get().
    // When a Decode() operation occurs, the decoded values are overwritten
    // onto the field values, and will become the new value which is
    // returned by Get().
    //
    // Returns the field value on success.
    // Returns NAN on error.
    // - Must use isnan() to check for NAN, can't compare via  == NAN

    // An error will occur when the field is not defined.
    double Get(const char* field_name) {
        double ret_val = NAN;
        if (FieldDefExists(field_name)) {
            FieldDef& fd = *GetFieldDef(field_name);
            ret_val = fd.value;
        }
        return ret_val;
    }

    // Header Field Setters / Getters
    // Read the default HdrTelemetryType, or read the value which was set
    // from Decode().
    uint8_t GetHdrTelemetryType() {
        return static_cast<uint8_t>(Get("HdrTelemetryType"));
    }

    // Set the Extended Telemetry HdrSlot value.
    // This field associates the encoded telemetry with the sender.
    // In a given repeating 10-minute cycle, starting on the start minute,
    // which is the 0th minute, the slots are defined as:
    // - start minute = slot 0
    // - +2 min       = slot 1
    // - +4 min       = slot 2
    // - +6 min       = slot 3
    // - +8 min       = slot 4
    void SetHdrSlot(uint8_t val) {
        Set("HdrSlot", val);
    }

    // Read the default HdrSlot, the previously SetHdrSlot() slot number, or
    // read the slot number which was set from Decode().
    uint8_t GetHdrSlot() {
        return static_cast<uint8_t>(Get("HdrSlot"));
    }

    // Encode / Decode
    // Encode the values of the defined fields into a set of encoded WSPR
    // Type 1 message fields (callsign, grid4, power_dbm). This overwrites
    // the Type 1 message fields.
    //
    // The functions GetCallsign(), GetGrid4(), and GetPowerDbm() will
    // subsequently return the encoded values for those fields.
    void Encode() {
        // create big number
        uint64_t val = 0;
        // pack application fields into big number in reverse-definition
        // order
        PackFields(val,
                   field_def_user_defined_list_,
                   field_def_user_defined_list_idx_);
        // pack header fields into message in reverse-definition order
        PackFields(val, field_def_header_list_, kHeaderFieldCount);

        // encode into power
        uint8_t power_val = val % 19; val /= 19;
        uint8_t power_dbm = Wspr::GetPowerDbmList()[power_val];

        // encode into grid
        uint8_t g4_val = val % 10; val /= 10;
        uint8_t g3_val = val % 10; val /= 10;
        uint8_t g2_val = val % 18; val /= 18;
        uint8_t g1_val = val % 18; val /= 18;
        char g1 = 'A' + g1_val;
        char g2 = 'A' + g2_val;
        char g3 = '0' + g3_val;
        char g4 = '0' + g4_val;

        static const uint8_t kGrid4Len = 4;
        char grid4[kGrid4Len + 1] = { 0 };
        grid4[0] = g1;
        grid4[1] = g2;
        grid4[2] = g3;
        grid4[3] = g4;

        // encode into callsign
        uint8_t id6_val = val % 26; val /= 26;
        uint8_t id5_val = val % 26; val /= 26;
        uint8_t id4_val = val % 26; val /= 26;
        uint8_t id2_val = val % 36; val /= 36;
        char id2 = WsprMessageTelemetryCommon::EncodeBase36(id2_val);
        char id4 = 'A' + id4_val;
        char id5 = 'A' + id5_val;
        char id6 = 'A' + id6_val;

        static const uint8_t kCallsignLen = 6;
        char callsign[kCallsignLen + 1] = { 0 };
        callsign[0] = WsprMessageTelemetryCommon::GetId13()[0];
        callsign[1] = id2;
        callsign[2] = WsprMessageTelemetryCommon::GetId13()[1];
        callsign[3] = id4;
        callsign[4] = id5;
        callsign[5] = id6;

        // capture results
        WsprMessageRegularType1::SetCallsign(callsign);
        WsprMessageRegularType1::SetGrid4(grid4);
        WsprMessageRegularType1::SetPowerDbm(power_dbm);
    }

    // Decode the values of the WSPR Type 1 message fields that were set by
    // using SetCallsign(), SetGrid4(), and SetPowerDbm(). This overwrites
    // every defined field value and header field value.
    //
    // Returns true on success.
    // Returns false on error.
    //
    // An error will occur when:
    // - The HdrTelemetryType is not ExtendedTelemetry
    //
    // Even when Decode returns an error, Get() will still return the field
    // and header values which were decoded.
    // The decoded field values are stored internally and are retrieved by
    // using Get().
    bool Decode() {
        bool ret_val = true;
        // pull in inputs
        const char* callsign  = WsprMessageRegularType1::GetCallsign();
        const char* grid4     = WsprMessageRegularType1::GetGrid4();
        uint8_t     power_dbm = WsprMessageRegularType1::GetPowerDbm();

        // break callsign down
        uint8_t id2_val =
            WsprMessageTelemetryCommon::DecodeBase36(callsign[1]);
        uint8_t id4_val = callsign[3] - 'A';
        uint8_t id5_val = callsign[4] - 'A';
        uint8_t id6_val = callsign[5] - 'A';

        // break grid down
        uint8_t g1_val = grid4[0] - 'A';
        uint8_t g2_val = grid4[1] - 'A';
        uint8_t g3_val = grid4[2] - '0';
        uint8_t g4_val = grid4[3] - '0';

        // break power down
        uint8_t power_val =
            WsprMessageTelemetryCommon::DecodePowerDbmToNum(power_dbm);

        // create big number
        uint64_t val = 0;
        val *= 36; val += id2_val;
        val *= 26; val += id4_val;   // spaces aren't used, so 26 not 27
        val *= 26; val += id5_val;   // spaces aren't used, so 26 not 27
        val *= 26; val += id6_val;   // spaces aren't used, so 26 not 27
        val *= 18; val += g1_val;
        val *= 18; val += g2_val;
        val *= 10; val += g3_val;
        val *= 10; val += g4_val;
        val *= 19; val += power_val;
        // unpack header fields
        UnpackFields(val, field_def_header_list_, kHeaderFieldCount);
        // unpack application fields
        UnpackFields(val,
                     field_def_user_defined_list_,
                     field_def_user_defined_list_idx_);
        // validate only that this is Extended Telemetry
        ret_val = GetHdrTelemetryType() == 0;
        return ret_val;
    }


private:
    static constexpr double kFactor = 1000.0;  // 3 decimal places
    static const uint8_t kHeaderFieldCount = 2;
    static const uint8_t kFieldNameCapacity = 32;

    void AllocateStorage() {
        field_def_user_defined_list_ = new FieldDef[field_count_];
        // allocate parallel array of owned name buffers (one per slot)
        field_def_user_defined_name_storage_ = new char*[field_count_];
        for (uint8_t i = 0; i < field_count_; ++i) {
            field_def_user_defined_name_storage_[i] =
                new char[kFieldNameCapacity];
            field_def_user_defined_name_storage_[i][0] = '\0';
        }
    }

    void FreeStorage() {
        if (field_def_user_defined_name_storage_ != nullptr) {
            for (uint8_t i = 0; i < field_count_; ++i) {
                delete[] field_def_user_defined_name_storage_[i];
            }
            delete[] field_def_user_defined_name_storage_;
            field_def_user_defined_name_storage_ = nullptr;
        }
        delete[] field_def_user_defined_list_;
        field_def_user_defined_list_ = nullptr;
    }

    // Scale a decimal value up by kFactor (1000) into integer space, 
    // with rounding away from zero. 
    // Used to test "is this value at most 3 decimal places of precision?" 
    // and to do arithmetic without floating point error.
    static int64_t ScaleUp(double value) {
        int64_t value_scaled_up = 0;
        if (value < 0) {
            value_scaled_up =
                static_cast<int64_t>((value * kFactor) - 0.5);
        } else if (value > 0) {
            value_scaled_up =
                static_cast<int64_t>((value * kFactor) + 0.5);
        }
        return value_scaled_up;
    }

    static bool FieldIsTooPrecise(double value) {
        int64_t value_scaled_up   = ScaleUp(value);
        double  value_scaled_back = value_scaled_up / kFactor;
        double diff = fabs(value_scaled_back - value);

        bool ret_val = false;
        if (diff > 0.000000001) {  // billionth
            ret_val = true;
        }
        return ret_val;
    }

    static uint32_t GetStepCount(double low_value,
                                 double high_value,
                                 double step_size) {
        uint32_t ret_val = 0;
        int64_t low_value_scaled_up_as_int  = ScaleUp(low_value);
        int64_t high_value_scaled_up_as_int = ScaleUp(high_value);
        int64_t step_size_scaled_up_as_int  = ScaleUp(step_size);

        double step_count =
            static_cast<double>(high_value_scaled_up_as_int -
                (low_value_scaled_up_as_int) / step_size_scaled_up_as_int);

        if (step_count == static_cast<uint32_t>(step_count)) {
            ret_val = static_cast<uint32_t>(step_count);
        }
        return ret_val;
    }

    // Pack one set of fields into `val` in reverse-definition order.
    static void PackFields(uint64_t& val,
                           FieldDef* field_def_list,
                           uint8_t   field_def_list_len) {
        for (int i = static_cast<int>(field_def_list_len) - 1; i >= 0; --i) {
            // get field def
            FieldDef& fd = field_def_list[i];
            // calculate field number for packing
            uint32_t field_number = static_cast<uint32_t>(
                round((fd.value - fd.low_value) / fd.step_size));
            // pack
            val *= fd.num_values; val += field_number;
        }
    }

    // Unpack one set of fields out of `val` in definition order.
    static void UnpackFields(uint64_t& val,
                             FieldDef* field_def_list,
                             uint8_t   field_def_list_len) {
        for (int i = 0; i < static_cast<int>(field_def_list_len); ++i) {
            // get field def
            FieldDef& fd = field_def_list[i];
            // calculate field number
            uint32_t field_number = val % fd.num_values;
            // set field value
            fd.value = fd.low_value + (field_number * fd.step_size);
            // shed
            val /= fd.num_values;
        }
    }

    FieldDef* GetFieldDefFrom(const char* field_name,
                              FieldDef*   field_def_list,
                              uint8_t     field_def_list_len) {
        FieldDef* ret_val = nullptr;
        if (field_name) {
            for (int i = 0; i < static_cast<int>(field_def_list_len); ++i) {
                FieldDef& fd = field_def_list[i];
                if (strcmp(field_name, fd.name) == 0) {
                    ret_val = &fd;
                    break;
                }
            }
        }
        return ret_val;
    }

    FieldDef* GetFieldDefHeader(const char* field_name) {
        return GetFieldDefFrom(field_name,
                               field_def_header_list_,
                               kHeaderFieldCount);
    }

    FieldDef* GetFieldDefUserDefined(const char* field_name) {
        return GetFieldDefFrom(field_name,
                               field_def_user_defined_list_,
                               field_def_user_defined_list_idx_);
    }

    FieldDef* GetFieldDef(const char* field_name) {
        FieldDef* ret_val = nullptr;
        ret_val = GetFieldDefHeader(field_name);
        if (ret_val == nullptr) {
            ret_val = GetFieldDefUserDefined(field_name);
        }
        return ret_val;
    }

    bool FieldDefExists(const char* field_name) {
        return GetFieldDef(field_name) != nullptr;
    }

    bool IsOkToSet(const char* field_name) {
        bool ret_val = true;
        if (field_name) {
            // reject if field name is on the restricted list
            const char* kRestrictedFieldNameList[2] = {
                "HdrTelemetryType",
                "HdrSlot",
            };
            int restricted_len = 1;
            for (int i = 0; i < restricted_len; ++i) {
                if (strcmp(field_name, kRestrictedFieldNameList[i]) == 0) {
                    ret_val = false;
                }
            }
        } else {
            ret_val = false;
        }
        return ret_val;
    }

    bool CanFitOneMore() {
        return field_def_user_defined_list_idx_ < field_count_;
    }

    // Disallow copy / assign — we own heap storage and have not implemented
    // deep copy semantics.
    WsprMessageTelemetryExtendedCommon(
        const WsprMessageTelemetryExtendedCommon&);

    WsprMessageTelemetryExtendedCommon& operator=(
        const WsprMessageTelemetryExtendedCommon&);


private:
    // configured at construction time
    uint8_t field_count_;

    // heap-allocated user-defined field array, sized to field_count_
    FieldDef* field_def_user_defined_list_;
    uint16_t  field_def_user_defined_list_idx_;

    // Heap-allocated parallel array of owned name buffers 
    // (one buffer per slot), 
    // Callers do not need to keep their field-name string alive
    // after DefineField() returns.
    char** field_def_user_defined_name_storage_;
    const char* field_def_fail_reason_;
    double num_bits_sum_;

    // never changes count, but values can change
    FieldDef field_def_header_list_[kHeaderFieldCount];
};

#endif  // WSPR_MESSAGE_TELEMETRY_EXTENDED_COMMON_H
