
#ifndef PARAMETERS_H
#define PARAMETERS_H

namespace project_namespace {
    class Parameters {
        private:
        protected:
        public:
            static constexpr float forwards_alpha = 0.0;
            static constexpr float backwards_alpha = 1.0;
            static constexpr float lateral_alpha = 0.36;

            static constexpr float roll_gain = 0.0;
            static constexpr float pitch_gain = 0.0;
            static constexpr float roll_rate_gain = 0.0;
            static constexpr float pitch_rate_gain = 0.0;
    };
}
#endif