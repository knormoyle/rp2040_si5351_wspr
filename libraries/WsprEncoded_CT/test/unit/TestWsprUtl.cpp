#include <array>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>
using namespace std;

#include "TestUtl.h"
using namespace TestUtl;

#include "WsprUtl.h"
using namespace WsprUtl;


const size_t CAPACITY = 4;

struct ParamPackOperations
{
    array<char, CAPACITY> arr;

    struct OpDef
    {
        string op;
        string arg = "";
    };

    vector<OpDef> opDefList;
};

template <typename T>
bool DoOperationsTest(const string &testName, vector<T> &testList)
{
    bool retVal = true;
    for (auto &test : testList)
    {
        CString cs(reinterpret_cast<const char *>(test.input.arr.data()), CAPACITY);

        for (const auto &opDef : test.input.opDefList)
        {
            if (opDef.op == "clear")
            {
                cs.Clear();
            }
            else if (opDef.op == "set")
            {
                cs.Set(opDef.arg.c_str());
            }
            else if (opDef.op == "toupper")
            {
                cs.ToUpper();
            }
            else if (opDef.op == "trimleft")
            {
                cs.TrimLeft();
            }
            else if (opDef.op == "trimright")
            {
                cs.TrimRight();
            }
        }

        retVal &= CheckErr(test, test.input.arr);
    }

    cout << testName << ": " << retVal << endl;

    return retVal;
}

bool TestCStringClear()
{
    vector<InputTest<ParamPackOperations, array<char, CAPACITY>>> testList = {
        {
            {               // ParamPack
                {  },       // input array
                {           // opDefList
                    { "clear", "arg" },
                }
            },
            { 0, 0, 0 },    // result array
            "clear from undefined",      // comment
        },
        {
            {
                { 1, 2, 3, 4 },
                {
                    { "clear" },
                }
            },
            { 0, 0, 0, 0 },
            "clear from defined",
        },
    };

    return DoOperationsTest("TestCStringClear", testList);
}

bool TestCStringSet()
{
    vector<InputTest<ParamPackOperations, array<char, CAPACITY>>> testList = {
        {
            {
                { 1, 2, 3, 4 },
                {
                    { "set", "h" },
                }
            },
            { "h" },
            "set, underfill, from defined",
        },
        {
            {
                { 1, 2, 3, 4 },
                {
                    { "set", "hi" },
                }
            },
            { "hi" },
            "set, from defined",
        },
        {
            {
                { 1, 2, 3, 4 },
                {
                    { "set", "hips" },
                }
            },
            { "hip" },
            "set, overfill and truncate, from defined",
        },
    };

    return DoOperationsTest("TestCStringSet", testList);
}

bool TestCStringToUpper()
{
    vector<InputTest<ParamPackOperations, array<char, CAPACITY>>> testList = {
        {
            {
                { 1, 2, 3, 4 },
                {
                    { "set", "hips" },
                    { "toupper" },
                }
            },
            { "HIP" },
            "set, toupper",
        },
        {
            {
                { 1, 2, 3, 4 },
                {
                    { "set", "" },
                    { "toupper" },
                }
            },
            { "" },
            "empty string, toupper",
        },
    };

    return DoOperationsTest("TestCStringToUpper", testList);
}

bool TestCStringTrimLeft()
{
    vector<InputTest<ParamPackOperations, array<char, CAPACITY>>> testList = {
        {
            {
                { 1, 2, 3, 4 },
                {
                    { "set", "hip" },
                    { "trimleft" },
                }
            },
            { "hip" },
            "set with no padding, trimleft",
        },
        {
            {
                { 1, 2, 3, 4 },
                {
                    { "set", "hi " },
                    { "trimleft" },
                }
            },
            { "hi " },
            "set with right padding, trimleft",
        },
        {
            {
                { 1, 2, 3, 4 },
                {
                    { "set", " i " },
                    { "trimleft" },
                }
            },
            { "i " },
            "set with both padding, trimleft",
        },
        {
            {
                { 1, 2, 3, 4 },
                {
                    { "set", " ip" },
                    { "trimleft" },
                }
            },
            { "ip" },
            "set with left padding, trimleft",
        },
        {
            {
                { 1, 2, 3, 4 },
                {
                    { "set", "" },
                    { "trimleft" },
                }
            },
            { "" },
            "empty string, trimleft",
        },
        {
            {
                { 1, 2, 3, 4 },
                {
                    { "set", "   " },
                    { "trimleft" },
                }
            },
            { "" },
            "spaces, trimleft",
        },
    };

    return DoOperationsTest("TestCStringTrimLeft", testList);
}

bool TestCStringTrimRight()
{
    vector<InputTest<ParamPackOperations, array<char, CAPACITY>>> testList = {
        {
            {
                { 1, 2, 3, 4 },
                {
                    { "set", "hip" },
                    { "trimright" },
                }
            },
            { "hip" },
            "set with no padding, trimright",
        },
        {
            {
                { 1, 2, 3, 4 },
                {
                    { "set", "hi " },
                    { "trimright" },
                }
            },
            { "hi" },
            "set with right padding, trimright",
        },
        {
            {
                { 1, 2, 3, 4 },
                {
                    { "set", " ip" },
                    { "trimright" },
                }
            },
            { " ip" },
            "set with left padding, trimright",
        },
        {
            {
                { 1, 2, 3, 4 },
                {
                    { "set", " i " },
                    { "trimright" },
                }
            },
            { " i" },
            "set with both padding, trimright",
        },
        {
            {
                { 1, 2, 3, 4 },
                {
                    { "set", "" },
                    { "trimright" },
                }
            },
            { "" },
            "empty string, trimright",
        },
        {
            {
                { 1, 2, 3, 4 },
                {
                    { "set", "   " },
                    { "trimright" },
                }
            },
            { "" },
            "spaces, trimright",
        },
    };

    return DoOperationsTest("TestCStringTrimRight", testList);
}


struct ParamPackBool
{
    array<char, CAPACITY> arr;

    string op;
    const char *cStr = "";
};

template <typename T>
bool DoCStringBoolTest(const string &testName, vector<T> &testList)
{
    bool retVal = true;
    for (auto &test : testList)
    {
        CString cs(reinterpret_cast<const char *>(test.input.arr.data()), CAPACITY);

        if (test.input.op == "ispaddedleft")
        {
            retVal &= CheckErr(test, cs.IsPaddedLeft());
        }
        else if (test.input.op == "ispaddedright")
        {
            retVal &= CheckErr(test, cs.IsPaddedRight());
        }
        else if (test.input.op == "isuppercase")
        {
            retVal &= CheckErr(test, cs.IsUppercase());
        }
        else if (test.input.op == "isequal")
        {
            retVal &= CheckErr(test, cs.IsEqual(test.input.cStr));
        }
    }

    cout << testName << ": " << retVal << endl;

    return retVal;
}

bool TestCStringIsPaddedLeft()
{
    vector<InputTest<ParamPackBool, bool>> testList = {
        { { { ""   }, "ispaddedleft" }, false, "empty string" },
        { { { "h"  }, "ispaddedleft" }, false, "no padding"   },
        { { { " "  }, "ispaddedleft" }, true,  "just space"   },
        { { { " i" }, "ispaddedleft" }, true,  "left padded"  },
        { { { "h " }, "ispaddedleft" }, false, "right padded" },
    };

    return DoCStringBoolTest("TestCStringIsPaddedLeft", testList);
}

bool TestCStringIsPaddedRight()
{
    vector<InputTest<ParamPackBool, bool>> testList = {
        { { { ""   }, "ispaddedright" }, false, "empty string" },
        { { { "h"  }, "ispaddedright" }, false, "no padding"   },
        { { { " "  }, "ispaddedright" }, true,  "just space"   },
        { { { " i" }, "ispaddedright" }, false, "left padded"  },
        { { { "h " }, "ispaddedright" }, true,  "right padded" },
    };

    return DoCStringBoolTest("TestCStringIsPaddedRight", testList);
}

bool TestCStringIsUppercase()
{
    vector<InputTest<ParamPackBool, bool>> testList = {
        { { { ""    }, "isuppercase" }, true,  "empty string"           },
        { { { " "   }, "isuppercase" }, true,  "just space"             },
        { { { "h"   }, "isuppercase" }, false, "lowercase"              },
        { { { " i"  }, "isuppercase" }, false, "left padded lowercase"  },
        { { { "h "  }, "isuppercase" }, false, "right padded lowercase" },
        { { { "H"   }, "isuppercase" }, true,  "uppercase"              },
        { { { " H"  }, "isuppercase" }, true,  "left padded uppercase"  },
        { { { "H "  }, "isuppercase" }, true,  "right padded uppercase" },
        { { { "1"   }, "isuppercase" }, true,  "1 char"                 },
        { { { "!"   }, "isuppercase" }, true,  "! char"                 },
        { { { "HI1" }, "isuppercase" }, true,  "HI1"                    },
        { { { "HI!" }, "isuppercase" }, true,  "HI!"                    },
    };

    return DoCStringBoolTest("TestCStringIsUppercase", testList);
}

bool TestCStringIsEqual()
{
    vector<InputTest<ParamPackBool, bool>> testList = {
        { { { ""   }, "isequal", nullptr }, false, "empty vs nullptr"   },
        { { { ""   }, "isequal", ""      }, true,  "empty vs empty"     },
        { { { " "  }, "isequal", ""      }, false, "non-empty vs empty" },
        { { { ""   }, "isequal", " "     }, false, "empty vs non-empty" },
        { { { " "  }, "isequal", " "     }, true,  "space v space"      },
        { { { " "  }, "isequal", "  "    }, false, "space v 2 space"    },
        { { { "  " }, "isequal", "  "    }, true,  "2 space v 2 space"  },
        { { { "  " }, "isequal", " "     }, false, "2 space v space"    },
    };

    return DoCStringBoolTest("TestCStringIsEqual", testList);
}


bool TestRotation()
{
    struct ParamPack
    {
        array<uint8_t, 3> arr;
        int               count;
    };

    vector<InputTest<ParamPack, array<uint8_t, 3>>> testList = {
        { { { 1, 2, 3 }, -4 },  { 2, 3, 1 },  "-4 rotation", },
        { { { 1, 2, 3 }, -3 },  { 1, 2, 3 },  "-3 rotation", },
        { { { 1, 2, 3 }, -2 },  { 3, 1, 2 },  "-2 rotation", },
        { { { 1, 2, 3 }, -1 },  { 2, 3, 1 },  "-1 rotation", },
        { { { 1, 2, 3 },  0 },  { 1, 2, 3 },   "0 rotation", },
        { { { 1, 2, 3 },  1 },  { 3, 1, 2 },   "1 rotation", },
        { { { 1, 2, 3 },  2 },  { 2, 3, 1 },   "2 rotation", },
        { { { 1, 2, 3 },  3 },  { 1, 2, 3 },   "3 rotation", },
        { { { 1, 2, 3 },  4 },  { 3, 1, 2 },   "4 rotation", },
    };

    bool retVal = true;
    for (auto &test : testList)
    {
        retVal &= CheckErr(test, Rotate(test.input.arr, test.input.count));
    }

    cout << "TestRotation: " << retVal << endl;

    return retVal;
}


int main()
{
    bool retVal = true;

    retVal &= TestCStringClear();
    retVal &= TestCStringSet();
    retVal &= TestCStringToUpper();
    retVal &= TestCStringTrimLeft();
    retVal &= TestCStringTrimRight();
    retVal &= TestCStringIsPaddedLeft();
    retVal &= TestCStringIsPaddedRight();
    retVal &= TestCStringIsUppercase();
    retVal &= TestCStringIsEqual();

    retVal &= TestRotation();

    return !retVal;
}