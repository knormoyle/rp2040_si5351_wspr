#include <iostream>
#include <cmath>
#include <string>
#include <getopt.h>
#include <cassert>
#include <stdexcept>

//****************************
// from https://app.codeconvert.ai/code-converter?inputLang=Python&outputLang=C%2B%2B
// Notes
// The translation maintains the core logic of the original Python code while 
// adapting it to C++ idioms and patterns.
// Key changes made in the translation:
// Replaced Python's tuple returns with a custom RationalResult struct
// Used C++'s getopt_long for command line argument parsing
// Replaced Python's dynamic typing with appropriate C++ types
// Used C++'s exception handling mechanism
// Replaced Python's print statements with C++'s iostream and printf
// Used proper C++ numeric types (long instead of int for larger numbers)
// The error handling is more explicit in the C++ version
// Added proper type casting where needed to prevent potential numeric issues
// Used C++'s standard library features where appropriate
// Maintained the original algorithm's logic and mathematical operations
//****************************
// https://github.com/alidasdan/best-rational-approximation/blob/master/ad_rat_by_fast_farey.py


// find the best rational approximation n/d where d <= l to the given
// target number t. here the best means the one with the smallest
// absolute error between t and n/d. 

// this code uses "the binary search" (where the "mid point" is the
// mediant, over the farey sequence but with a speedup technique to
// skip many iterations. note that the mediant of two fractions a/b and
// c/d (where a/b < c/d) is (a+c)/(b+d), which is guaranteed to lie
// between a/b and c/d.

// naming convention: n=numerator, d=denominator, l=left, r=right

// author: ali dasdan
// C++ translation

void show_usage(const char* program_name) {
    std::cout << program_name << " -h/--help [-e/--error=float>=0] -l/--limit=int>1 -t/--target=float or quoted math expr returning float\n";
}

void at_exit(const std::string& msg) {
    if (!msg.empty()) {
        std::cerr << "Error: " << msg << std::endl;
    }
    show_usage(nullptr);
    exit(0);
}

struct RationalResult {
    // this struct has to be in this order because there are returns that set some?
    // return {err, n, d, niter};
    double error; // kbn err..for consistency with other algo
    long numerator; // kbn n..for consistency with other algo
    long denominator; // kbn d..for consistency with other algo
    int iterations; // kbn niter..for consistency with other algo

    // not used..just match the continued fractions algo
    double err;
    int n; // the two algos vary: int vs long
    int d; // the two algos vary: int vs long
    int niter;

    double actual_real;  // kbn
    double actual_error; // kbn
    double target; // kbn

};

RationalResult find_best_rat(long l, double t) {
    assert(l >= 1);

    // kbn Should we assume it's between 0 and 1 to start?
    // double t_int;
    // double t_frac = modf(t, &t_int);
    double t_frac = t; // kbn
    double t_int = 0; // kbn

    // handle the trivial case
    // kbn: it should return 0 for target == 0.0
    // was returning a numerator halfway..so getting bad error
    // if (t_frac <= 0) {
    // Should compare to smallest number?
    if (t_frac <= 1e-15) {
        return {0, static_cast<long>(t), 1, 0};
    }
    // kbn: what about the trival max case. not legal?
    // assert below will detect t_frac 1.0 and fail?
    // just return l (in case of our random 0,1 range checking
    // if (t_frac == 1.0) {
    //    return {l, static_cast<long>(t), 1, 0};
    //}

    // begin the processing for t_frac
    assert(0 < t_frac && t_frac < 1);

    // start with the endpoints 0/1 and 1/1
    long nl = 0, dl = 1;
    long nr = 1, dr = 1;
    int loc = 0;
    int niter = 0;

    long br;
    long bl;
    while (dl <= l && dr <= l) {
        niter++;

        // find the mediant med=nm/dm with dm <= 1
        double ntmp = nl - t_frac * dl;
        double dtmp = t_frac * dr - nr;
        br = static_cast<long>(floor(ntmp / dtmp));
        bl = static_cast<long>(floor(dtmp / ntmp));
        int side = 0; // left:-1, init:0, right:1

        if (bl == 0) {
            bl = 1;
            side = -1;
        }
        if (br == 0) {
            br = 1;
            side = 1;
        }
        long dm = dl * bl + dr * br;

        if (dm > l) {
            if (side == -1) {
                br = std::max(1L, static_cast<long>(floor(static_cast<double>(l - dl) / dr)));
            }
            else if (side == 1) {
                bl = std::max(1L, static_cast<long>(floor(static_cast<double>(l - dr) / dl)));
            }
        }

        long nm = nl * bl + nr * br;
        dm = dl * bl + dr * br;
        if (dm > l) break;
        double med = static_cast<double>(nm) / dm;

        if (fabs(t_frac - med) < 1e-15) {
            loc = 0;
            break;
        }
        else if (t_frac < med) {
            loc = -1;
            nr = nm;
            dr = dm;
        }
        else {
            loc = 1;
            nl = nm;
            dl = dm;
        }
    }

    long n, d;
    if (loc == 0) {
        n = nl * bl + nr * br;
        d = dl * bl + dr * br;
    }
    else {
        double errl = fabs(t_frac - static_cast<double>(nl) / dl);
        double errr = fabs(t_frac - static_cast<double>(nr) / dr);
        if (errl <= errr) {
            n = nl;
            d = dl;
        }
        else {
            n = nr;
            d = dr;
        }
    }

    n += static_cast<long>(t_int) * d;
    double err = (t - static_cast<double>(n) / d);
    return {err, n, d, niter};
}

RationalResult find_best_rat_with_err_bound(double err_in, long l, double t) {
    long l_curr = 1;
    int sum_niter = 0;
    auto result = find_best_rat(l_curr, t);
    
    while (fabs(result.error) > err_in && l_curr < l) {
        l_curr *= 10;
        sum_niter += result.iterations;
        result = find_best_rat(l_curr, t);
    }
    
    result.iterations = sum_niter;
    return result;
}

int main2(int argc, char* argv[]) {
    double eps = -1;
    long l = -1;
    double t = -1;

    static struct option long_options[] = {
        {"help", no_argument, 0, 'h'},
        {"error", required_argument, 0, 'e'},
        {"limit", required_argument, 0, 'l'},
        {"target", required_argument, 0, 't'},
        {0, 0, 0, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "he:l:t:", long_options, nullptr)) != -1) {
        try {
            switch (opt) {
                case 'h':
                    show_usage(argv[0]);
                    return 0;
                case 'e':
                    eps = std::stod(optarg);
                    if (eps <= 0) throw std::invalid_argument("Error must be positive");
                    break;
                case 'l':
                    l = std::stol(optarg);
                    if (l < 1) throw std::invalid_argument("Limit must be greater than 0");
                    break;
                case 't':
                    t = std::stod(optarg);
                    if (t <= 0) throw std::invalid_argument("Target must be positive");
                    break;
                default:
                    at_exit("Invalid option");
            }
        }
        catch (const std::exception& e) {
            at_exit(e.what());
        }
    }

    if (t == -1 || l == -1) {
        at_exit("Target and limit args are required");
    }

    RationalResult result;
    if (eps == -1) {
        result = find_best_rat(l, t);
        printf("target= %f best_rat= %ld / %ld max_denom= %ld err= %g abs_err= %g niter= %d\n",
               t, result.numerator, result.denominator, l, result.error, 
               fabs(result.error), result.iterations);
    }
    else {
        result = find_best_rat_with_err_bound(eps, l, t);
        printf("target= %f best_rat= %ld / %ld max_denom= %ld err= %g abs_err= %g abs_err/error= %g niter= %d\n",
               t, result.numerator, result.denominator, l, result.error, 
               fabs(result.error), fabs(result.error)/eps, result.iterations);
    }

    return 0;
}
