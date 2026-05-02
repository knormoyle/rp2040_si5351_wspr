#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdlib>   // strtoul
#include <ctime>     // time (default seed)

#include "WsprEncoded_CT.h"


// Track pass/fail across the whole run instead of bailing on first error,
// so a single failure does not hide downstream issues.
static int g_fail_count = 0;
static int g_check_count = 0;
// Verbose mode prints the encoded callsign/grid/power for each successful
// encode step. Off by default so a passing run produces a one-line
// summary; on (via `-v`) for debugging encode/decode mismatches.
static bool g_verbose = false;

/////////////////////////////////////////////////////////////////
// Deterministic LCG — Numerical Recipes parameters (32-bit).
//
// Using our own generator rather than rand() makes the sequence
// identical across platforms regardless of libc implementation,
// and lets the caller inject a seed for reproducing failures.
//
// Pass --seed <N> on the command line to replay a specific run.
// The seed in use is always printed so a failing run can be
// reproduced exactly.
/////////////////////////////////////////////////////////////////
static uint32_t g_rand_state = 0;

static uint32_t RandNext() {
    // LCG: x_{n+1} = (1664525 * x_n + 1013904223) mod 2^32
    g_rand_state = g_rand_state * 1664525u + 1013904223u;
    return g_rand_state;
}

// Returns a uniformly distributed integer in [lo, hi] (inclusive).
// Precondition: lo <= hi and (hi - lo + 1) fits in uint32_t.
static int32_t RandRange(int32_t lo, int32_t hi) {
    uint32_t range = static_cast<uint32_t>(hi - lo) + 1u;
    return lo + static_cast<int32_t>(RandNext() % range);
}

// Returns a double in [lo, hi] rounded to `decimals` decimal places.
// decimals == 0 gives an integer-valued double.
// Precondition: lo <= hi, both representable with <= `decimals` decimal
// places, and (hi - lo) * 10^decimals fits in int32_t.
static double RandDouble(double lo, double hi, int decimals) {
    double scale = 1.0;
    for (int i = 0; i < decimals; ++i) { scale *= 10.0; }
    int32_t lo_int = static_cast<int32_t>(std::round(lo * scale));
    int32_t hi_int = static_cast<int32_t>(std::round(hi * scale));
    return static_cast<double>(RandRange(lo_int, hi_int)) / scale;
}

// Pick a random element from a fixed-size C array.
template<typename T, int N>
static const T& RandPick(const T (&arr)[N]) {
    return arr[static_cast<int>(RandNext() % static_cast<uint32_t>(N))];
}

static constexpr int kRandomRuns = 20;

#define VPRINTF(...)                                                        \
    do { if (g_verbose) { printf(__VA_ARGS__); } } while (0)

#define CHECK(cond, msg)                                                    \
    do {                                                                    \
        ++g_check_count;                                                    \
        if (!(cond)) {                                                      \
            ++g_fail_count;                                                 \
            printf("  FAIL: %s (line %d)\n", (msg), __LINE__);              \
        }                                                                   \
    } while (0)

#define CHECK_NEAR(actual, expected, tol, msg)                              \
    do {                                                                    \
        ++g_check_count;                                                    \
        double check_a = (actual);                                          \
        double check_e = (expected);                                        \
        if (std::fabs(check_a - check_e) > (tol)) {                         \
            ++g_fail_count;                                                 \
            printf("  FAIL: %s: got %f, want %f (tol %f) (line %d)\n",      \
                   (msg), check_a, check_e,                                 \
                   static_cast<double>(tol), __LINE__);                     \
        }                                                                   \
    } while (0)


// Copy the encoded WSPR Type 1 fields (callsign, grid4, power_dbm) from
// `src` to `dst`. This is the wire-equivalent of "transmit, receive".
static void TransferWire(const WsprMessageRegularType1& src,
                         WsprMessageRegularType1& dst) {
    dst.SetCallsign(src.GetCallsign());
    dst.SetGrid4(src.GetGrid4());
    dst.SetPowerDbm(src.GetPowerDbm());
}


/////////////////////////////////////////////////////////////////
// Basic Telemetry round-trip
/////////////////////////////////////////////////////////////////

static void TestBasicRoundTrip(const char* label,
                               const char* id13,
                               const char* grid56,
                               int32_t     altitude_meters,
                               int32_t     temperature_celsius,
                               double      voltage_volts,
                               int32_t     speed_knots,
                               bool        gps_is_valid) {
    printf("[basic] %s\n", label);

    WsprMessageTelemetryBasic enc;
    CHECK(enc.SetId13(id13), "SetId13");
    CHECK(enc.SetGrid56(grid56), "SetGrid56");
    CHECK(enc.SetAltitudeMeters(altitude_meters), "SetAltitudeMeters");
    CHECK(enc.SetTemperatureCelsius(temperature_celsius),
          "SetTemperatureCelsius");
    CHECK(enc.SetVoltageVolts(voltage_volts), "SetVoltageVolts");
    CHECK(enc.SetSpeedKnots(speed_knots), "SetSpeedKnots");
    CHECK(enc.SetGpsIsValid(gps_is_valid), "SetGpsIsValid");

    enc.Encode();

    VPRINTF("  encoded: call=%s grid=%s power=%u\n",
            enc.GetCallsign(), enc.GetGrid4(),
            static_cast<unsigned>(enc.GetPowerDbm()));

    // Round-trip through a fresh decoder so we are sure decode populates
    // every field and does not just observe the encoder's state.
    WsprMessageTelemetryBasic dec;
    CHECK(dec.SetId13(id13), "decoder SetId13");
    TransferWire(enc, dec);
    CHECK(dec.Decode(), "Decode");

    // Encoded altitude is rounded to nearest 20 m using std::round (which
    // rounds half away from zero).  Replicate that exactly so the expected
    // value agrees with the encoder even at half-step boundaries (e.g.
    // 5010 m: 5010/20 = 250.5 -> std::round -> 251 -> 5020 m).
    // The previous integer formula ((alt + 10) / 20) * 20 coincidentally
    // agreed for most inputs but diverges from std::round for the
    // half-step case with a negative altitude (clamped to 0 anyway) and
    // could silently mask an off-by-one in the encoder.
    int32_t expected_alt = static_cast<int32_t>(
        std::round(static_cast<double>(altitude_meters) / 20.0)) * 20;
    if (expected_alt < 0)     { expected_alt = 0; }
    if (expected_alt > 21340) { expected_alt = 21340; }

    // Encoded speed is rounded to nearest 2 knots.
    int32_t expected_speed = ((speed_knots + 1) / 2) * 2;
    if (speed_knots < 0) { expected_speed = 0; }
    if (expected_speed > 82) { expected_speed = 82; }

    CHECK(strcmp(dec.GetGrid56(), grid56) == 0, "grid56 round-trip");
    CHECK(dec.GetAltitudeMeters() == static_cast<uint16_t>(expected_alt),
          "altitude round-trip");
    CHECK(dec.GetTemperatureCelsius() ==
              static_cast<int8_t>(temperature_celsius),
          "temperature round-trip");
    // Voltage is encoded in 0.05 V steps.  Use half-step tolerance (0.026 V)
    // so the check catches an off-by-one-step encoding error.  The previous
    // tolerance of 0.05 V was a full step width and would silently pass a
    // value that decoded to an adjacent step.
    CHECK_NEAR(dec.GetVoltageVolts(), voltage_volts, 0.026,
               "voltage round-trip");
    CHECK(dec.GetSpeedKnots() == static_cast<uint8_t>(expected_speed),
          "speed round-trip");
    CHECK(dec.GetGpsIsValid() == gps_is_valid, "gps round-trip");
}


/////////////////////////////////////////////////////////////////
// Extended Telemetry (User-Defined) round-trip
/////////////////////////////////////////////////////////////////

struct ExtFieldSpec {
    const char* name;
    double      low_value;
    double      high_value;
    double      step_size;
    double      set_value;
    double      expected_value;   // after step rounding
};

static void TestExtUserDefinedRoundTrip(const char*         label,
                                        uint8_t             field_count,
                                        const ExtFieldSpec* fields,
                                        uint8_t             num_fields,
                                        uint8_t             hdr_slot,
                                        const char*         id13) {
    printf("[ext-user] %s\n", label);

    WsprMessageTelemetryExtendedUserDefined enc(field_count);

    for (uint8_t i = 0; i < num_fields; ++i) {
        bool ok = enc.DefineField(fields[i].name,
                                  fields[i].low_value,
                                  fields[i].high_value,
                                  fields[i].step_size);
        if (!ok) {
            VPRINTF("  DefineField(%s) failed: %s\n",
                    fields[i].name, enc.GetDefineFieldErr());
        }
        CHECK(ok, "DefineField");
    }

    CHECK(enc.SetId13(id13), "SetId13");
    enc.SetHdrSlot(hdr_slot);
    for (uint8_t i = 0; i < num_fields; ++i) {
        CHECK(enc.Set(fields[i].name, fields[i].set_value), "Set field");
    }

    enc.Encode();

    VPRINTF("  encoded: call=%s grid=%s power=%u\n",
            enc.GetCallsign(), enc.GetGrid4(),
            static_cast<unsigned>(enc.GetPowerDbm()));

    // Fresh decoder, same field schema, transferred wire fields.
    WsprMessageTelemetryExtendedUserDefined dec(field_count);
    for (uint8_t i = 0; i < num_fields; ++i) {
        CHECK(dec.DefineField(fields[i].name,
                              fields[i].low_value,
                              fields[i].high_value,
                              fields[i].step_size),
              "decoder DefineField");
    }
    CHECK(dec.SetId13(id13), "decoder SetId13");

    TransferWire(enc, dec);
    CHECK(dec.Decode(), "Decode");

    CHECK(dec.GetHdrSlot() == hdr_slot, "hdr_slot round-trip");

    for (uint8_t i = 0; i < num_fields; ++i) {
        // Use a tolerance of half the step size to absorb the rounding
        // that happens during encode (set value -> nearest representable).
        double tol = fields[i].step_size / 2.0 + 1e-9;
        CHECK_NEAR(dec.Get(fields[i].name),
                   fields[i].expected_value,
                   tol,
                   fields[i].name);
    }
}


int main(int argc, char** argv) {
    // Default seed: current time, so unseeded runs explore different values
    // each invocation. Pass --seed <N> to replay a specific failing run.
    uint32_t seed = static_cast<uint32_t>(time(nullptr));
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-v") == 0 ||
            strcmp(argv[i], "--verbose") == 0) {
            g_verbose = true;
        } else if (strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
            seed = static_cast<uint32_t>(strtoul(argv[i + 1], nullptr, 10));
            ++i;
        }
    }
    g_rand_state = seed;
    printf("RNG seed: %u  (replay with --seed %u)\n", seed, seed);

    // ---------------------------------------------------------------
    // Wspr band/power lookups
    // ---------------------------------------------------------------
    {
        printf("[wspr] band/power lookups\n");
        CHECK(Wspr::GetDialFreqFromBandStr("20m") == 14095600UL,
              "20m dial freq");
        CHECK(Wspr::PowerDbmInSet(23),  "power 23 in set");
        CHECK(!Wspr::PowerDbmInSet(24), "power 24 not in set");
        CHECK(strcmp(Wspr::GetDefaultBandIfNotValid("nope"), "20m") == 0,
              "default band fallback");

        // Random: every band in the table must map to a non-zero dial freq,
        // and every valid power value must pass PowerDbmInSet().
        printf("[wspr] random band/power lookups (%d runs)\n", kRandomRuns);
        {
            const Wspr::BandData* band_list = Wspr::GetBandDataList();
            uint8_t band_count = Wspr::GetBandDataCount();
            const uint8_t* power_list = Wspr::GetPowerDbmList();
            uint8_t power_count = Wspr::GetPowerDbmCount();
            // Values that must NOT be in the power set (gaps between valid dBm).
            // The valid set uses steps of 3 or 4; pick a gap value that is
            // always invalid: N+1 where N is a valid value not at the top.
            for (int run = 0; run < kRandomRuns; ++run) {
                // Random band lookup
                uint8_t bi = static_cast<uint8_t>(RandNext() % band_count);
                const char* band = band_list[bi].band;
                uint32_t expected_freq = band_list[bi].freq;
                CHECK(Wspr::GetDialFreqFromBandStr(band) == expected_freq,
                      "random band dial freq");
                // Random valid power lookup
                uint8_t pi = static_cast<uint8_t>(RandNext() % power_count);
                uint8_t valid_pwr = power_list[pi];
                CHECK(Wspr::PowerDbmInSet(valid_pwr), "random valid power in set");
                // A power value of valid+1 is never valid (the minimum gap
                // between consecutive entries is 3, so +1 always lands in a gap),
                // unless valid_pwr is already 60 (the maximum). Skip that edge.
                if (valid_pwr < 60) {
                    CHECK(!Wspr::PowerDbmInSet(static_cast<uint8_t>(valid_pwr + 1)),
                          "random valid+1 power not in set");
                }
                // GetDefaultBandIfNotValid must return the same pointer for a
                // known band, and "20m" for any unknown string.
                CHECK(strcmp(Wspr::GetDefaultBandIfNotValid(band), band) == 0,
                      "random known band returned by default check");
            }
        }
    }

    // ---------------------------------------------------------------
    // ChannelMap
    // ---------------------------------------------------------------
    {
        printf("[channel-map] channel 0 on 20m\n");
        WsprChannelMap::ChannelDetails cd =
            WsprChannelMap::GetChannelDetails("20m", 0);
        VPRINTF("  id13=%s min=%u lane=%u dial=%u\n",
               cd.id13,
               static_cast<unsigned>(cd.min),
               static_cast<unsigned>(cd.lane),
               static_cast<unsigned>(cd.freq_dial));
        CHECK(cd.freq_dial == 14095600UL, "channel dial freq");

        // Random: for 20 random (band, channel) pairs verify that:
        //   - freq_dial matches GetDialFreqFromBandStr
        //   - freq is within the 200 Hz WSPR window around dial+1500
        //   - lane is in [1,4] (band 3 is the reserved centre band, excluded)
        //   - min is one of {0,2,4,6,8}
        //   - id13 chars are consistent with the id1/id3 fields
        printf("[channel-map] random channel details (%d runs)\n", kRandomRuns);
        {
            const Wspr::BandData* band_list = Wspr::GetBandDataList();
            uint8_t band_count = Wspr::GetBandDataCount();
            for (int run = 0; run < kRandomRuns; ++run) {
                uint8_t  bi      = static_cast<uint8_t>(RandNext() % band_count);
                uint16_t channel = static_cast<uint16_t>(RandNext() % 600);
                const char* band = band_list[bi].band;
                WsprChannelMap::ChannelDetails rcd =
                    WsprChannelMap::GetChannelDetails(band, channel);
                uint32_t dial = Wspr::GetDialFreqFromBandStr(band);
                // freq_dial must match the band's canonical dial frequency.
                CHECK(rcd.freq_dial == dial, "random channel dial freq matches band");
                // tx freq must sit inside the ±100 Hz WSPR window.
                CHECK(rcd.freq >= dial + 1400 && rcd.freq <= dial + 1600,
                      "random channel tx freq inside WSPR window");
                // lane 1..4 (band 3 is reserved and never assigned).
                CHECK(rcd.lane >= 1 && rcd.lane <= 4,
                      "random channel lane in [1,4]");
                // minute offset must be even and in [0,8].
                CHECK(rcd.min == 0 || rcd.min == 2 || rcd.min == 4 ||
                      rcd.min == 6 || rcd.min == 8,
                      "random channel minute is valid even offset");
                // id13 string must agree with id1 and id3 characters.
                CHECK(rcd.id13[0] == rcd.id1 && rcd.id13[1] == rcd.id3 &&
                      rcd.id13[2] == '\0',
                      "random channel id13 consistent with id1/id3");
                // channel number must survive the round-trip through the map.
                CHECK(rcd.channel == channel, "random channel number round-trip");
                VPRINTF("  [run %d] band=%s ch=%u lane=%u min=%u freq=%u\n",
                        run, band, channel,
                        static_cast<unsigned>(rcd.lane),
                        static_cast<unsigned>(rcd.min),
                        static_cast<unsigned>(rcd.freq));
            }
        }
    }

    // ---------------------------------------------------------------
    // Regular Type1 validation
    // ---------------------------------------------------------------
    {
        printf("[type1] validation\n");
        WsprMessageRegularType1 m;
        CHECK(m.SetCallsign("KD2EPF"), "valid callsign");
        CHECK(m.SetGrid4("FN20"),      "valid grid");
        CHECK(m.SetPowerDbm(23),       "valid power");
        CHECK(!m.SetCallsign("kd2epf"), "reject lowercase callsign");
        CHECK(!m.SetPowerDbm(24),       "reject bad power");

        // Random valid callsigns, grids, and power values must all be
        // accepted; deliberately invalid values must all be rejected.
        printf("[type1] random validation (%d runs)\n", kRandomRuns);
        {
            // Alphabet pools
            static const char kUpper[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
            static const char kDigit[] = "0123456789";
            // Valid grid field-1/2 chars: A-R (18 letters)
            static const char kGridAR[] = "ABCDEFGHIJKLMNOPQR";
            const uint8_t* power_list  = Wspr::GetPowerDbmList();
            uint8_t power_count = Wspr::GetPowerDbmCount();

            for (int run = 0; run < kRandomRuns; ++run) {
                WsprMessageRegularType1 rm;

                // ---- valid callsign: 4-6 uppercase alphanumeric chars ----
                // Build a guaranteed-valid callsign: digit in pos 2 (index 1
                // in a 0-based 6-char string per WSPR convention is not
                // strictly required here, but we just need uppercase + length).
                // Simplest: pick a length in [4,6] and fill with uppercase.
                int cslen = RandRange(4, 6);
                char cs[7] = {0};
                for (int ci = 0; ci < cslen; ++ci) {
                    cs[ci] = kUpper[RandNext() % 26];
                }
                CHECK(WsprMessageRegularType1::CallsignIsValid(cs),
                      "random uppercase callsign accepted");
                CHECK(rm.SetCallsign(cs), "random SetCallsign succeeds");

                // ---- valid grid: A-R, A-R, 0-9, 0-9 ----
                char gr[5] = {0};
                gr[0] = kGridAR[RandNext() % 18];
                gr[1] = kGridAR[RandNext() % 18];
                gr[2] = kDigit[RandNext() % 10];
                gr[3] = kDigit[RandNext() % 10];
                CHECK(WsprMessageRegularType1::Grid4IsValid(gr),
                      "random grid accepted");
                CHECK(rm.SetGrid4(gr), "random SetGrid4 succeeds");

                // ---- valid power ----
                uint8_t vpwr = power_list[RandNext() % power_count];
                CHECK(rm.SetPowerDbm(vpwr), "random valid power accepted");

                // ---- invalid: lowercase callsign ----
                char lcs[7] = {0};
                for (int ci = 0; ci < cslen; ++ci) {
                    // force at least one lowercase letter
                    lcs[ci] = (ci == 0)
                        ? static_cast<char>('a' + (RandNext() % 26))
                        : kUpper[RandNext() % 26];
                }
                CHECK(!WsprMessageRegularType1::CallsignIsValid(lcs),
                      "random lowercase callsign rejected");

                // ---- invalid: callsign too short (3 chars) ----
                char short_cs[4] = {
                    kUpper[RandNext()%26], kUpper[RandNext()%26],
                    kUpper[RandNext()%26], '\0'
                };
                CHECK(!WsprMessageRegularType1::CallsignIsValid(short_cs),
                      "random 3-char callsign rejected");

                // ---- invalid power: pick a value not in the set ----
                // The gaps between valid values are always >= 3. N+1 for
                // any non-max valid value is guaranteed to be invalid.
                uint8_t base_pwr = power_list[RandNext() % (power_count - 1)];
                CHECK(!rm.SetPowerDbm(static_cast<uint8_t>(base_pwr + 1)),
                      "random invalid power rejected");

                // ---- invalid grid: S-Z in first char (valid only A-R) ----
                char bad_gr[5] = {0};
                bad_gr[0] = static_cast<char>('S' + (RandNext() % 8)); // S..Z
                bad_gr[1] = kGridAR[RandNext() % 18];
                bad_gr[2] = kDigit[RandNext() % 10];
                bad_gr[3] = kDigit[RandNext() % 10];
                CHECK(!WsprMessageRegularType1::Grid4IsValid(bad_gr),
                      "random out-of-range grid char rejected");

                VPRINTF("  [run %d] cs=%s gr=%s pwr=%u\n",
                        run, cs, gr, static_cast<unsigned>(vpwr));
            }
        }
    }

    // ---------------------------------------------------------------
    // Basic telemetry round-trips: nominal, low extremes, high extremes,
    // mid range, and step-rounding cases
    // ---------------------------------------------------------------
    TestBasicRoundTrip("nominal",
                       "Q3", "AB", 5000, -10, 3.7, 40, true);

    // Low end of every range
    TestBasicRoundTrip("low extremes",
                       "00", "AA", 0, -50, 3.0, 0, false);

    // High end of every range
    TestBasicRoundTrip("high extremes",
                       "19", "XX", 21340, 39, 4.95, 82, true);

    // Mid range with gps invalid
    TestBasicRoundTrip("mid, gps invalid",
                       "Q5", "MN", 10000, 0, 4.0, 20, false);

    // Step rounding: altitude 5010 -> 5000 or 5020; speed 41 -> 40 or 42
    TestBasicRoundTrip("step rounding",
                       "Q0", "GH", 5010, 15, 3.85, 41, true);

    // Random: 20 independent round-trips with random values drawn uniformly
    // from each field's valid range.  Voltage is sampled at 0.05 V
    // resolution to stay on a representable step.
    printf("[basic] random round-trips (%d runs)\n", kRandomRuns);
    {
        static const char* kId13List[] = {
            "00","01","02","03","04","05","06","07","08","09",
            "10","11","12","13","14","15","16","17","18","19",
            "Q0","Q1","Q2","Q3","Q4","Q5","Q6","Q7","Q8","Q9"
        };
        static const char kGrid5Chars[] = "ABCDEFGHIJKLMNOPQRSTUVWX"; // A-X
        for (int run = 0; run < kRandomRuns; ++run) {
            const char* id13 = RandPick(kId13List);
            char grid56[3];
            grid56[0] = kGrid5Chars[RandNext() % 24];
            grid56[1] = kGrid5Chars[RandNext() % 24];
            grid56[2] = '\0';
            int32_t alt   = RandRange(0, 1067) * 20;       // exact 20 m steps
            int32_t temp  = RandRange(-50, 39);
            double  volts = RandDouble(3.0, 4.95, 2);      // 0.01 V resolution;
                                                            // encoder rounds to
                                                            // nearest 0.05 V
            int32_t speed = RandRange(0, 41) * 2;          // exact 2-knot steps
            bool    gps   = (RandNext() & 1u) != 0u;
            char label[64];
            snprintf(label, sizeof(label), "random run %d", run);
            TestBasicRoundTrip(label, id13, grid56, alt, temp, volts,
                               speed, gps);
        }
    }

    // ---------------------------------------------------------------
    // Extended user-defined: small schema
    // ---------------------------------------------------------------
    {
        ExtFieldSpec fields[] = {
            { "battery", 3.0,   4.5,    0.05, 3.7,   3.7   },
            { "temp",   -50.0,  50.0,   1.0,  22.0,  22.0  },
            { "counter", 0.0,   1023.0, 1.0,  500.0, 500.0 },
        };
        TestExtUserDefinedRoundTrip("3 fields, slot 2",
                                    /*field_count=*/5,
                                    fields, 3,
                                    /*hdr_slot=*/2,
                                    "Q3");
    }

    // ---------------------------------------------------------------
    // Extended user-defined: low and high boundary values
    // ---------------------------------------------------------------
    {
        // At low boundaries
        ExtFieldSpec lows[] = {
            { "a", 0.0,   10.0, 1.0,    0.0,  0.0 },
            { "b", -1.0,  1.0,  0.05,  -1.0, -1.0 },
        };
        TestExtUserDefinedRoundTrip("low boundaries",
                                    /*field_count=*/4,
                                    lows, 2,
                                    /*hdr_slot=*/0,
                                    "00");

        // At high boundaries
        ExtFieldSpec highs[] = {
            { "a", 0.0,   10.0, 1.0,   10.0, 10.0 },
            { "b", -1.0,  1.0,  0.05,  1.0,  1.0  },
        };
        TestExtUserDefinedRoundTrip("high boundaries",
                                    /*field_count=*/4,
                                    highs, 2,
                                    /*hdr_slot=*/4,
                                    "19");
    }

    // ---------------------------------------------------------------
    // Extended user-defined: many small fields packed close to the
    // 29.180-bit budget
    // ---------------------------------------------------------------
    {
        ExtFieldSpec many[] = {
            { "f0",  0.0, 7.0,  1.0, 5.0,  5.0  },   // 3 bits
            { "f1",  0.0, 7.0,  1.0, 0.0,  0.0  },   // 3 bits
            { "f2",  0.0, 15.0, 1.0, 9.0,  9.0  },   // 4 bits
            { "f3",  0.0, 15.0, 1.0, 15.0, 15.0 },   // 4 bits
            { "f4",  0.0, 31.0, 1.0, 17.0, 17.0 },   // 5 bits
            { "f5",  0.0, 1.0,  1.0, 1.0,  1.0  },   // 1 bit
            { "f6",  0.0, 1.0,  1.0, 0.0,  0.0  },   // 1 bit
            // total ~21 bits, well under budget
        };
        TestExtUserDefinedRoundTrip("7 packed fields",
                                    /*field_count=*/8,
                                    many, 7,
                                    /*hdr_slot=*/1,
                                    "Q5");
    }

    // ---------------------------------------------------------------
    // Extended user-defined: step rounding inside Set()
    // (set 3.72 with step 0.05 -> nearest is 3.70)
    // ---------------------------------------------------------------
    {
        ExtFieldSpec rounding[] = {
            { "battery", 3.0, 4.5, 0.05, 3.72, 3.70 },
        };
        TestExtUserDefinedRoundTrip("step rounding",
                                    /*field_count=*/2,
                                    rounding, 1,
                                    /*hdr_slot=*/0,
                                    "Q0");
    }

    // ---------------------------------------------------------------
    // Extended user-defined: random schemas
    //
    // Each run picks a random number of fields (1-4), random ranges and
    // step sizes (kept to 1-3 decimal places to satisfy DefineField's
    // precision check), random values within those ranges, a random id13,
    // and a random hdr_slot.  The expected decoded value is the set_value
    // rounded to the nearest step.
    // ---------------------------------------------------------------
    printf("[ext-user] random schemas (%d runs)\n", kRandomRuns);
    {
        // Step sizes we may use: all representable in <= 2 decimal places
        // and guaranteed to divide typical integer ranges evenly.
        static const double kStepChoices[] = { 1.0, 2.0, 5.0, 10.0, 0.5, 0.25 };
        static const char* kId13List[] = {
            "00","01","09","10","19","Q0","Q5","Q9"
        };

        for (int run = 0; run < kRandomRuns; ++run) {
            int num_fields = RandRange(1, 4);
            // field_count must be at least num_fields + 1 (for the header)
            // but we pass it as the user-field capacity, so just use num_fields.
            WsprMessageTelemetryExtendedUserDefined enc(
                static_cast<uint8_t>(num_fields));
            WsprMessageTelemetryExtendedUserDefined dec(
                static_cast<uint8_t>(num_fields));

            // Build fields
            struct RandField {
                char   name[16];
                double low;
                double high;
                double step;
                double set_val;
                double expected;
            };
            // Use a local fixed-size array (max 4 fields).
            RandField rfields[4];
            bool schema_ok = true;
            for (int fi = 0; fi < num_fields; ++fi) {
                snprintf(rfields[fi].name, sizeof(rfields[fi].name), "f%d", fi);
                double step = RandPick(kStepChoices);
                // low in [0, 9], high = low + step * (2..9)
                int32_t low_int  = RandRange(0, 9);
                int32_t n_steps  = RandRange(2, 9);
                double  low      = static_cast<double>(low_int);
                double  high     = low + step * static_cast<double>(n_steps);
                // set_value: pick a random step index and compute exactly
                int32_t step_idx = RandRange(0, n_steps);
                double  set_val  = low + step * static_cast<double>(step_idx);
                // Round expected to nearest step (same as encoder).
                double  expected = low +
                    std::round((set_val - low) / step) * step;
                rfields[fi].low      = low;
                rfields[fi].high     = high;
                rfields[fi].step     = step;
                rfields[fi].set_val  = set_val;
                rfields[fi].expected = expected;
                bool ok_enc = enc.DefineField(rfields[fi].name, low, high, step);
                bool ok_dec = dec.DefineField(rfields[fi].name, low, high, step);
                if (!ok_enc || !ok_dec) {
                    schema_ok = false;
                    VPRINTF("  [ext-random run %d] DefineField(%s) failed: %s\n",
                            run, rfields[fi].name, enc.GetDefineFieldErr());
                }
            }
            if (!schema_ok) {
                // Bit budget exceeded for this random schema — skip rather
                // than CHECK-fail; the bit-budget rejection is already
                // covered by the capacity rejection test.
                VPRINTF("  [ext-random run %d] schema skipped (budget)\n", run);
                continue;
            }

            const char* id13     = RandPick(kId13List);
            uint8_t     hdr_slot = static_cast<uint8_t>(RandRange(0, 4));
            CHECK(enc.SetId13(id13), "random ext SetId13");
            enc.SetHdrSlot(hdr_slot);
            for (int fi = 0; fi < num_fields; ++fi) {
                CHECK(enc.Set(rfields[fi].name, rfields[fi].set_val),
                      "random ext Set");
            }
            enc.Encode();

            VPRINTF("  [ext-random run %d] call=%s grid=%s power=%u\n",
                    run, enc.GetCallsign(), enc.GetGrid4(),
                    static_cast<unsigned>(enc.GetPowerDbm()));

            CHECK(dec.SetId13(id13), "random ext decoder SetId13");
            TransferWire(enc, dec);
            CHECK(dec.Decode(), "random ext Decode");
            CHECK(dec.GetHdrSlot() == hdr_slot, "random ext hdr_slot round-trip");
            for (int fi = 0; fi < num_fields; ++fi) {
                double tol = rfields[fi].step / 2.0 + 1e-9;
                CHECK_NEAR(dec.Get(rfields[fi].name), rfields[fi].expected, tol,
                           "random ext field round-trip");
            }
        }
    }

    // ---------------------------------------------------------------
    // Capacity rejection
    // ---------------------------------------------------------------
    {
        printf("[ext] capacity rejection\n");
        WsprMessageTelemetryExtendedUserDefined m(2);
        CHECK(m.DefineField("a", 0, 10, 1), "1st field");
        CHECK(m.DefineField("b", 0, 10, 1), "2nd field");
        CHECK(!m.DefineField("c", 0, 10, 1), "3rd rejected");
        VPRINTF("  reason: %s\n", m.GetDefineFieldErr());
    }

    // ---------------------------------------------------------------
    // Regression: B1 + B2 — public Set() must reject reserved header
    // names ("HdrTelemetryType", "HdrSlot"), but SetHdrSlot() must
    // succeed and have an observable effect on the encoded output.
    //
    // History: the original IsOkToSet() had `restricted_len = 1` while
    // the array had 2 entries, so Set("HdrSlot", ...) silently
    // succeeded, undermining the guard. After fixing the off-by-one,
    // SetHdrSlot() (which used to call Set() and now would have been
    // rejected by the guard) was rewired through an internal write
    // path so it can still write the reserved field.
    // ---------------------------------------------------------------
    {
        printf("[regression] reserved-field guard (B1/B2)\n");
        WsprMessageTelemetryExtendedUserDefined m(2);
        CHECK(m.DefineField("x", 0.0, 100.0, 1.0), "DefineField x");

        // Public Set() must reject both reserved names.
        CHECK(!m.Set("HdrSlot", 3),
              "Set(\"HdrSlot\", ...) rejected via public guard");
        CHECK(!m.Set("HdrTelemetryType", 1),
              "Set(\"HdrTelemetryType\", ...) rejected via public guard");
        // And nullptr.
        CHECK(!m.Set(nullptr, 0),
              "Set(nullptr, ...) rejected");
        // A regular user-defined field must still be settable.
        CHECK(m.Set("x", 42.0), "Set on user-defined field still works");

        // SetHdrSlot must actually update the slot. Encode/decode
        // round-trip is the strongest proof: change slot, encode, copy
        // wire fields, decode in a fresh instance with the same schema,
        // and observe GetHdrSlot() returning the new value.
        m.SetHdrSlot(3);
        m.Encode();

        WsprMessageTelemetryExtendedUserDefined dec(2);
        CHECK(dec.DefineField("x", 0.0, 100.0, 1.0),
              "decoder DefineField x");
        TransferWire(m, dec);
        CHECK(dec.Decode(), "Decode");
        CHECK(dec.GetHdrSlot() == 3,
              "SetHdrSlot value survives round-trip");
    }

    // ---------------------------------------------------------------
    // Regression: B3 — WsprChannelMap::GetMinuteListForBand must
    // string-compare the band name, not pointer-compare it.
    //
    // History: the inner loop did `if (band_list[i].band == band)`,
    // which only worked because Wspr::GetDefaultBandIfNotValid()
    // returns a pointer into the same static array, making the
    // pointer comparison coincidentally correct. Pass a heap-copied
    // string with identical contents and the original code would walk
    // the loop without matching, leaving idx == band_count and
    // selecting the wrong rotation.
    //
    // We compare results obtained via (a) a string literal known to
    // match the static array's pointer, and (b) a heap-copied string
    // with the same bytes. Both must produce identical minute lists.
    // ---------------------------------------------------------------
    {
        printf("[regression] channel-map band lookup (B3)\n");
        const char* literal = "40m";
        // Heap-copy the same band name. `dup` has a different address
        // from any pointer in Wspr::GetBandDataList(), so a pointer
        // comparison would fail.
        char dup[8];
        strncpy(dup, "40m", sizeof(dup) - 1);
        dup[sizeof(dup) - 1] = '\0';
        CHECK(literal != dup, "test setup: pointers really do differ");

        uint8_t list_lit[5];
        uint8_t list_dup[5];
        WsprChannelMap::GetMinuteListForBand(literal, list_lit);
        WsprChannelMap::GetMinuteListForBand(dup,     list_dup);

        bool same = true;
        for (int i = 0; i < 5; ++i) {
            if (list_lit[i] != list_dup[i]) { same = false; }
        }
        CHECK(same, "minute list identical for literal vs heap-copied band");

        // Also positively verify against a known answer for 40m. 40m is
        // at index 5 in the band list (2190m, 630m, 160m, 80m, 60m, 40m),
        // so rotation = kRotationList[5 % 5] = kRotationList[0] = 4.
        // Base list {8, 0, 2, 4, 6} rotated right by 4 -> {0, 2, 4, 6, 8}.
        uint8_t expected[5] = { 0, 2, 4, 6, 8 };
        bool matches_expected = true;
        for (int i = 0; i < 5; ++i) {
            if (list_lit[i] != expected[i]) { matches_expected = false; }
        }
        CHECK(matches_expected, "minute list for 40m matches expected rotation");
    }

    // ---------------------------------------------------------------
    // Regression: bound checks on Encode/Decode base-36 helpers.
    // EncodeBase36(out-of-range) and DecodeBase36(non-base36 char)
    // must clamp/return-zero rather than producing silent garbage.
    //
    // The helpers are protected, so we expose them via a thin
    // test-only subclass. This avoids the need for a `friend`
    // declaration in the production header.
    // ---------------------------------------------------------------
    {
        printf("[regression] base-36 bounds\n");
        struct Base36Probe : public WsprMessageTelemetryCommon {
            using WsprMessageTelemetryCommon::EncodeBase36;
            using WsprMessageTelemetryCommon::DecodeBase36;
        };
        // EncodeBase36: in-range values must round-trip.
        CHECK(Base36Probe::EncodeBase36(0)  == '0', "encode 0  -> '0'");
        CHECK(Base36Probe::EncodeBase36(9)  == '9', "encode 9  -> '9'");
        CHECK(Base36Probe::EncodeBase36(10) == 'A', "encode 10 -> 'A'");
        CHECK(Base36Probe::EncodeBase36(35) == 'Z', "encode 35 -> 'Z'");
        // Out-of-range input must clamp to the highest valid digit.
        CHECK(Base36Probe::EncodeBase36(36)  == 'Z', "encode 36  clamps to 'Z'");
        CHECK(Base36Probe::EncodeBase36(255) == 'Z', "encode 255 clamps to 'Z'");

        // DecodeBase36: in-range values must round-trip.
        CHECK(Base36Probe::DecodeBase36('0') == 0,  "decode '0' -> 0");
        CHECK(Base36Probe::DecodeBase36('9') == 9,  "decode '9' -> 9");
        CHECK(Base36Probe::DecodeBase36('A') == 10, "decode 'A' -> 10");
        CHECK(Base36Probe::DecodeBase36('Z') == 35, "decode 'Z' -> 35");
        // Non-base36 inputs must return 0, not underflow into a huge
        // uint8_t value (the original code did `c - '0'` unconditionally).
        CHECK(Base36Probe::DecodeBase36(' ') == 0, "decode ' ' returns 0, not 240");
        CHECK(Base36Probe::DecodeBase36('!') == 0, "decode '!' returns 0");
        CHECK(Base36Probe::DecodeBase36('a') == 0, "decode 'a' (lowercase) returns 0");
        // High-byte input on a signed-char platform: must not produce
        // a sign-extended garbage value.
        CHECK(Base36Probe::DecodeBase36(static_cast<char>(0xC3)) == 0,
              "decode 0xC3 returns 0 (no sign extension)");
    }

    // ---------------------------------------------------------------
    // Summary
    // ---------------------------------------------------------------
    printf("\n%d/%d checks passed, %d failed\n",
           g_check_count - g_fail_count, g_check_count, g_fail_count);

    return g_fail_count == 0 ? 0 : 1;
}
