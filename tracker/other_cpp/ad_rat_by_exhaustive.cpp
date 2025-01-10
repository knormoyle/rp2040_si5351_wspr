#include <iostream>
#include <cstdlib>
#include <cmath>
#include <string>
#include <getopt.h>
#include <cassert>
#include <cinttypes>
#include <vector>
// #include <iostream>

// Notes
// From Ali Dasdan
// https://github.com/alidasdan/best-rational-approximation/blob/master/ad_rat_by_exhaustive.py
// converted with
// https://app.codeconvert.ai/code-converter?inputLang=Python&outputLang=C%2B%2B
// The Python code has been translated to C++ while maintaining the same functionality and logic.
// Key changes and adaptations include:
// Replaced Python's sys.argv handling with C++'s getopt_long
// Created a RationalResult struct to handle multiple return values (since C++ can't return multiple values directly like Python)
// Used C++'s exception handling mechanism instead of Python's
// Replaced Python's dynamic typing with C++'s static typing
// Used C++'s standard library functions instead of Python's math module
// Implemented proper C++ memory management and type safety
// The mathematical evaluation feature for the target parameter has been simplified as it's more complex to implement a safe eval() in C++
// Used printf for formatted output to maintain similar output format to the Python version
// Added proper C++ headers and namespace usage
// Maintained the same algorithmic approach for finding rational approximations

char program_name[64] = "ad_rat_by_exhaustive.cpp";

void show_usage(const char* program_name) {
    std::cout << program_name << " -h/--help [-e/--error=float>=0] -l/--limit=int=>1 -t/--target=float" << std::endl;
}

void at_exit(const std::string& msg) {
    if (!msg.empty()) {
        std::cout << "Error: " << msg << std::endl;
    }
    show_usage(program_name);
    exit(0);
}

// Find the best rational approximation n/d where d <= l to the given target number t
struct RationalResult {
    double error;
    int numerator;
    int denominator;
    uint64_t iterations;
};

//**********************************************
// https://www.quora.com/How-does-one-run-a-for-loop-in-parallel-i-e-allow-multiple-iterations-of-the-same-loop-to-be-executed-simultaneously-in-C++

// #include "tbb/tbb.h" 
// #include <tbb>
// using namespace tbb; 
// parallel_for(size_t(0),n,size_t(1),[=](size_t i) {Foo(a[i]);}); 
double best_err = 1.0;
int best_n = 0;
int best_d = 1;

void Foo(int d, int l, double t, uint64_t *niter) {
    uint64_t niter_here = *niter;
    // can't be more than the denominator
    for (int n = 1; n <= d; ++n) {
        niter_here++;
        if ((niter_here % 100000000) == 0) 
            printf("d %10d n %10d niter %12" PRIu64 " best_n %d best_d %d best_err %.16f\n",
                d, n, niter_here, best_n, best_d, best_err);

        double err = std::abs(t - static_cast<double>(n) / d);
        // could we parallize and just assume no collisions in updating around same time
        if (err < best_err) {
            best_err = err;
            best_n = n;
            best_d = d;
        }
    }
    *niter = niter_here;
}

//**********************************************
RationalResult find_best_rat(int l, double t) {
    assert(l >= 1 && t <= 1);

    // Handle the odd case
    if (t <= 0) {
        return {0, static_cast<int>(t), 1, 0};
    }

    best_err = t;
    best_n = 0; 
    best_d = 1;
    // kbn: hmm this has to be a 64-bit!
    uint64_t niter = 0;

    if (true) {
        // niter will sum all
        for (int d = 1; d <= l; ++d) {
            Foo(d, l, t, &niter);
        }
    }
    else {
        // https://github.com/uxlfoundation/oneTBB

        // https://chryswoods.com/parallel_c++/parallel_for.html
        // this will include d==0 ?
        // niter will sum oddly
        /*
        auto values = std::vector<int>(l);
        tbb::parallel_for(tbb::blocked_range<int>(0, values.size()), [&] (tbb::blocked_range<int> r) {
            for (int d = r.begin(); d < r.end(); d++) { Foo(d, l, t, &niter) }
        }
        */
    }
    
    return {best_err, best_n, best_d, niter};
}

RationalResult find_best_rat_with_err_bound(double err_in, int l, double t) {
    int l_curr = 1;
    uint64_t sum_niter = 0;
    RationalResult result = find_best_rat(l_curr, t);
    
    while (result.error > err_in && l_curr < l) {
        l_curr *= 10;
        sum_niter += result.iterations;
        result = find_best_rat(l_curr, t);
    }
    
    result.iterations = sum_niter;
    return result;
}

int main(int argc, char* argv[]) {
    double eps = -1;  // Using -1 to indicate unset
    int l = -1;
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
                    at_exit("");
                    break;
                case 'e':
                    eps = std::stod(optarg);
                    if (eps <= 0) throw std::runtime_error("Error must be > 0");
                    break;
                case 'l':
                    l = std::stoi(optarg);
                    if (l < 1) throw std::runtime_error("Limit must be >= 1");
                    break;
                case 't':
                    t = std::stod(optarg);
                    if (t <= 0) throw std::runtime_error("Target must be > 0");
                    break;
                default:
                    at_exit("Invalid option");
            }
        } catch (const std::exception& e) {
            at_exit(e.what());
        }
    }

    if (t == -1 || l == -1) {
        at_exit("Target and limit args are required");
    }

    double t_int;
    double t_frac = modf(t, &t_int);
    RationalResult result;
    
    if (eps == -1) {
        result = find_best_rat(l, t_frac);
    } else {
        result = find_best_rat_with_err_bound(eps, l, t_frac);
    }

    result.numerator += static_cast<int>(t_int * result.denominator);
    double err = (t - static_cast<double>(result.numerator) / result.denominator);

    if (eps == -1) {
        printf("target= %.16f best_rat= %d / %d max_denom= %d err= %.16g abs_err= %.16g niter= %" PRIu64 "\n",
               t, result.numerator, result.denominator, l, err, std::abs(err), result.iterations);
    } else {
        printf("target= %.16f best_rat= %d / %d max_denom= %d err= %.16g abs_err= %.16g abs_err/error= %.16g niter= %" PRIu64 "\n",
               t, result.numerator, result.denominator, l, err, std::abs(err), 
               std::abs(err) / eps, result.iterations);
    }

    return 0;
}
