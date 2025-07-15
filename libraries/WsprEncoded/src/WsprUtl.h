#pragma once

#include <array>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <cctype>


namespace WsprUtl
{


class CString
{
public:
    CString() {}

    CString(const char *str, size_t bufCapacity)
    {
        Target((char *)str, bufCapacity);
    }

    void Target(char *buf, size_t bufCapacity)
    {
        if (buf && bufCapacity)
        {
            buf_ = buf;
            bufCapacity_ = bufCapacity;
        }
    }

    void Clear()
    {
        if (bufCapacity_)
        {
            memset(buf_, 0, bufCapacity_);
        }
    }

    void Set(const char *str)
    {
        if (str && bufCapacity_)
        {
            Clear();
            strncpy(buf_, str, bufCapacity_ - 1);
        }
    }

    void ToUpper()
    {
        if (bufCapacity_)
        {
            char *p = buf_;
            while (*p != '\0')
            {
                *p = std::toupper(*p);
                ++p;
            }
        }
    }

    bool IsPaddedLeft()
    {
        return bufCapacity_ && buf_[0] == ' ';
    }

    bool IsPaddedRight()
    {
        bool retVal = false;

        size_t len = Len();

        if (len)
        {
            retVal = buf_[len - 1] == ' ';
        }

        return retVal;
    }

    bool IsUppercase()
    {
        // empty string considered uppercase
        // non-alpha chars are considered uppercase
        
        bool retVal = false;

        if (bufCapacity_)
        {
            if (buf_[0] == '\0')
            {
                retVal = true;
            }
            else
            {
                char *p = buf_;

                while (*p != '\0' && *p == toupper(*p))
                {
                    ++p;
                }

                retVal = *p == '\0';
            }
        }

        return retVal;
    }

    bool IsEqual(const char *str)
    {
        bool retVal = false;

        if (str && bufCapacity_)
        {
            retVal = strcmp(buf_, str) == 0;
        }

        return retVal;
    }

    size_t Len()
    {
        size_t retVal = 0;

        if (bufCapacity_)
        {
            retVal = strlen(buf_);
        }

        return retVal;
    }

    // shift left any spaces
    void TrimLeft()
    {
        if (bufCapacity_)
        {
            // find first non-space char
            size_t idxFirst = 0;
            while (buf_[idxFirst] == ' ')
            {
                ++idxFirst;
            }

            if (buf_[idxFirst] == '\0')
            {
                // whole string is whitespace

                // null terminate the start of string
                Clear();
            }
            else if (idxFirst == 0)
            {
                // nothing to do
            }
            else
            {
                // there was some whitespace, shift
                size_t len = Len();

                memmove(buf_, &buf_[idxFirst], len - idxFirst);
                buf_[len - idxFirst] = '\0';
            }
        }
    }

    void TrimRight()
    {
        if (bufCapacity_)
        {
            size_t len = Len();

            for (int i = len - 1; i >= 0; --i)
            {
                if (buf_[i] == ' ')
                {
                    buf_[i] = '\0';
                }
                else
                {
                    break;
                }
            }
        }
    }

    void Trim()
    {
        TrimRight();
        TrimLeft();
    }

    const char *Get() const
    {
        return buf_;
    }


private:
    char *buf_ = nullptr;
    size_t bufCapacity_ = 0;
};


// positive values mean rotate right
// negative values mean rotate left
// zero results in no rotation
template <typename T, size_t SIZE>
static std::array<T, SIZE> &Rotate(std::array<T, SIZE> &valList, int count)
{
    if (count > 0)
    {
        count = count % valList.size();
        std::rotate(valList.begin(), valList.begin() + (valList.size() - count), valList.end());
    }
    else
    {
        count = -count;
        count = count % valList.size();

        std::rotate(valList.begin(), valList.begin() +  count, valList.end());
    }

    return valList;
}




};

