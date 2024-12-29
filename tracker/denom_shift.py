import sys
import random
print("find the num and denom for a target PLL freq, with band-specific mult/div and tcxo that gives wspr shifts")

wsprShift = 12000/8192 # 1.486..
print("wsprShift", wsprShift)

wsprShift1 = wsprShift
wsprShift2 = wsprShift * 2
wsprShift3 = wsprShift * 3

print("round wsprShift3 to 4 decimal places and use that for optimization")
wsprShift1 = round(wsprShift1, 4)
wsprShift2 = round(wsprShift2, 4)
wsprShift3 = round(wsprShift3, 4)

print("wsprShift1", wsprShift1)
print("wsprShift2", wsprShift2)
print("wsprShift3", wsprShift3)

band = 10
tcxo = 26000000

print("")
print("these band-specific mult/div were created trying to target a 500Mhz PLL freq")
print("assume we want to generate the symbol 0 freq at the mid point of channel 599 bin for the band")

if band==10:
    # 390Mhz pll
    # mult = 14
    # div = 15
    # 416Mhz pll
    mult = 15
    div = 14
    # from WSPRDenom-modified.xlsx

    # 500Mhz pll
    # mult = 19
    # div = 18
    # 700Mhz pll
    # mult = 25
    # div = 24
    desiredSymbolFreq = 28126180
elif band==12:
    mult = 20
    div = 19
    desiredSymbolFreq = 24926180
elif band==15:
    mult = 19
    div = 24
    desiredSymbolFreq = 21096180
elif band==17:
    mult = 19
    div = 28
    desiredSymbolFreq = 18106180
elif band==20:
    mult = 19
    div = 36
    desiredSymbolFreq = 14097180

pll_freq_min = (mult * tcxo) 
pll_freq_max = ((mult + 1) * tcxo) 

print("")
print("tcxo", tcxo, "band", band, "mult", mult, "div", div, 
    "pll_freq_min", pll_freq_min, "pll_freq_max", pll_freq_max)
print("")

denomMax = 1048575

step = 50 
negstep = -1 * step
posstep = step
print("the good num/denom pair are still somewhat good within a range")
print("brute force search by stepping", "%d" % step, "on each. then step by 1 for each around a range of 100")

print("desiredSymbolFreq", "%.4f" % desiredSymbolFreq)

# end value is not included
cnt = 0
absoluteErrorMax = 50

sys.stdout.flush()
# don't bother with j < 10,000 ?
last_absolute_error = 1e9

# for j in range(denomMax, 50000, negstep):
# can't divide by 0
# i has to be < j
# don't bother with < 1000 ?
# for i in range(1000, j + 1, posstep):

# for k in range(10000000):
while True:
    j = random.randint(10000, denomMax) # inclusive
    for l in range(1000):
        i = random.randint(2500, j) # inclusive
        cnt+=1
        fractionalReal = i/j
        fractionalRealDenomShift1 = i/(j-1) # smaller denom so it's a bigger real (positive shift)
        # just print 8 decimal digits precision
        # print("cnt", "%d" % cnt, "%d" % i, "%d" % j, "fractionalReal", "%.8f" % fractionalReal)

        pll_freq = (mult  + fractionalReal) * tcxo
        output_freq = pll_freq / div

        shifted_pll_freq = (mult + fractionalRealDenomShift1) * tcxo
        shifted_output_freq = shifted_pll_freq / div
        # this is close enough..the symbols only shift 1.486 hz
        absoluteError = abs(desiredSymbolFreq - shifted_output_freq)

        shift = abs(shifted_output_freq - output_freq)

        # print("shifted_output_freq", "%.4f" % shifted_output_freq, "desiredSymbolFreq", "%.4f" % desiredSymbolFreq);
        if False:
            print("shift", "%.4f" % shift, "%d" % i, "%d" % j, 
                "output_freq", "%.4f" % output_freq, "absoluteError", "%.4f" % absoluteError);
        
        
        shiftErrMax = 5e-2
        # look for the shift being wsprShift to 4 decimal places
        shift1Error = abs(wsprShift - shift)
        # this will be the max absolute error
        last_absolute_error = 2.5

        # if absoluteError <= last_absolute_error: 
        if False and (shift1Error < shiftErrMax) and (absoluteError <= 5):
            print("GOOD absoluteError. shiftError", "%.4f" % shift1Error, 
                "(1 * shift)", "%.4f" % (1 * shift), "%d" % i, "%d" % j, 
                "output_freq", "%.4f" % output_freq, "absoluteError", "%.4f" % absoluteError);
            sys.stdout.flush()
            # last_absolute_error = absoluteError
        
        if (shift1Error < shiftErrMax) and absoluteError < last_absolute_error:
            print("GOOD 1 shift found. shiftError", "%.4f" % shift1Error, 
                "(1 * shift)", "%.4f" % (1 * shift), "%d" % i, "%d" % j, 
                "output_freq", "%.4f" % output_freq, "absoluteError", "%.4f" % absoluteError);
            sys.stdout.flush()
            # last_absolute_error = absoluteError
        # can shift more than 1 denom?
        shift2Error = abs(wsprShift - (2 * shift))
        if (shift2Error < shiftErrMax) and absoluteError < last_absolute_error:
            print("GOOD 2 shift found. shiftError", "%.4f" % shift2Error, 
                "(2 * shift)", "%.4f" % (2 * shift), "%d" % i, "%d" % j, 
                "output_freq", "%.4f" % output_freq, "absoluteError", "%.4f" % absoluteError);
            sys.stdout.flush()
            # last_absolute_error = absoluteError

        shift3Error = abs(wsprShift - (3 * shift))
        if (shift3Error < shiftErrMax) and absoluteError < last_absolute_error:
            print("GOOD 3 shift found. shiftError", "%.4f" % shift3Error, 
                "(3 * shift)", "%.4f" % (3 * shift), "%d" % i, "%d" % j, 
                "output_freq", "%.4f" % output_freq, "absoluteError", "%.4f" % absoluteError);
            sys.stdout.flush()
            # last_absolute_error = absoluteError

        # FIX! we should look for low absolute err. but we continue the loop on absolute > 5 above
        




