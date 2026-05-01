#pragma once

#include <array>
#include <cstdint>
#include <iostream>


namespace TestUtl
{
    template <typename T, size_t SIZE>
    inline std::ostream& operator<<(std::ostream& os, const std::array<T, SIZE> arr) {
        os << "[";
        string sep = "";
        for (const auto &val : arr)
        {
            os << sep << val;

            sep = ", ";
        }
        os << "]";

        return os;
    }

    template <size_t SIZE>
    inline std::ostream& operator<<(std::ostream& os, const std::array<uint8_t, SIZE> arr) {
        os << "uint8_t[";
        string sep = "";
        for (const auto &val : arr)
        {
            os << sep << (int)val;

            sep = ", ";
        }
        os << "]";

        return os;
    }

    template <typename T1, typename T2>
    struct InputTest
    {
        T1     input;
        T2     expectedRetVal;
        string comment;
    };

    template <typename T1, typename T2>
    static bool CheckErr(const InputTest<T1, T2> &test, T2 testRetVal)
    {
        bool retVal = true;

        if (testRetVal != test.expectedRetVal)
        {
            retVal = false;

            cout << "ERR: Got " << testRetVal << ", but expected " << test.expectedRetVal << " (" << test.comment << ")" << endl;
        }

        return retVal;
    }
};