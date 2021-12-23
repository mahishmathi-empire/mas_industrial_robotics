#ifndef OPTIMISER_H
#define OPTIMISER_H

#include <functional>
#include <vector>
#include <chrono>

typedef std::function<float (const std::vector<float>& )> CostFunction;

class Optimiser
{
    public:

        static void calculateOptimalUGD(const float& time_threshold,
                                        CostFunction calculateCost,
                                        std::vector<float>& u,
                                        float h = 1e-4f,
                                        float alpha = 1e-5f,
                                        float cost_threshold = 0.1f);

        static void calculateOptimalULS(const float& time_threshold,
                                        CostFunction calculateCost,
                                        std::vector<float>& u,
                                        float h = 1e-4f,
                                        float starting_alpha = 1e-8,
                                        float ending_alpha = 0.1f,
                                        float cost_threshold = 0.1f);

        static void calculateOptimalUCGD(const float& time_threshold,
                                         CostFunction calculateCost,
                                         std::vector<float>& u,
                                         float h = 1e-4f,
                                         float starting_alpha = 1e-8,
                                         float ending_alpha = 0.1f,
                                         float cost_threshold = 0.1f);

        static void calculateGradient(const std::vector<float>& u,
                                      float h,
                                      CostFunction calculateCost,
                                      float current_cost,
                                      std::vector<float>& gradient);

        static float calculateCostWithAlpha(const std::vector<float>& u,
                                            CostFunction calculateCost,
                                            const std::vector<float>& gradient,
                                            float alpha,
                                            std::vector<float>& new_u);

        static float calculateOptimalAlphaQN(const std::vector<float>& u,
                                             CostFunction calculateCost,
                                             float current_cost,
                                             const std::vector<float>& gradient,
                                             float smallest_alpha = 1e-8f,
                                             float cost_threshold = 0.1f);

        static float calculateOptimalAlpha(const std::vector<float>& u,
                                           CostFunction calculateCost,
                                           float current_cost,
                                           const std::vector<float>& gradient,
                                           float starting_alpha,
                                           float ending_alpha,
                                           float cost_threshold);
};

#endif // MPCN_OPTIMISER_H
