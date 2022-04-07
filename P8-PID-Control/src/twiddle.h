#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>

using std::vector;

class Twiddle {
    enum State {
        INIT,
		FIRST_CHECK,
		SECOND_CHECK
	};

    public:
        Twiddle();
        virtual ~Twiddle();
        vector<double> GetCurrentParams();
        vector<double> GetBestParams();
        vector<double> GetDP();
        bool IsDone();
        void Update(double err);

    private:
        vector<double> p = {0.0, 0.0, 0.0};
        vector<double> dp = {1.0, 1.0, 1.0};
        double limit = 0.2;
        State state = INIT;
        int idx = 0;
        int iteration = 0;
        double best_err = -1;
        vector<double> best_params;
};

#endif // TWIDDLE_H
