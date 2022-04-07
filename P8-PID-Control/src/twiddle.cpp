#include <numeric>
#include <iostream>
#include "twiddle.h"

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

vector<double> Twiddle::GetCurrentParams() {
    return p;
}

vector<double> Twiddle::GetBestParams() {
    return best_params;
}


vector<double> Twiddle::GetDP() {
    return dp;
}

bool Twiddle::IsDone() {
    double sum = std::accumulate(dp.begin(), dp.end(), 0.0);
    //std::cout << "sum: " << sum << std::endl;
    return  sum < limit;
}

void Twiddle::Update(double err) {
    //std::cout << "in state: " << state << std::endl;
    //std::cout << "err: " << err << ", " << "best_err: " << best_err <<std::endl;
    switch (state)
    {
    case INIT:
        best_err = err;
        best_params.assign(p.begin(), p.end());
        p[idx] += dp[idx];
        state = FIRST_CHECK;
        break;
    case FIRST_CHECK:
        if (err < best_err) {
            best_err = err;
            best_params.assign(p.begin(), p.end());
            dp[idx] *= 1.1;
            idx += 1;
            Increment();
            p[idx] += dp[idx];
            state = FIRST_CHECK;
        } else {
            p[idx] -= 2 * dp[idx];
            state = SECOND_CHECK;
        }
        break;
    case SECOND_CHECK:
        if (err < best_err) {
            best_err = err;
            //std::cout << "new best: " << best_err << std::endl;
            best_params.assign(p.begin(), p.end());
            dp[idx] *= 1.1;
        } else {
            p[idx] += dp[idx];
            dp[idx] *= 0.9;
        }
        idx += 1;
        Increment();
        p[idx] += dp[idx];
        state = FIRST_CHECK;
        break;
    default:
        break;
    }
    //std::cout << "out state: " << state << std::endl;
}

void Twiddle::Increment() {
    if (idx > 2) {
        idx = 0;
        std::cout <<iteration << " - best_err: " << best_err << ", dp: " << std::accumulate(dp.begin(), dp.end(), 0.0) << std::endl;
        iteration += 1;
    }
}