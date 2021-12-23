#include <mir_joint_space_controller/optimiser.h>
#include <mir_joint_space_controller/utils.h>

void Optimiser::calculateOptimalUGD(const float& time_threshold,
                                    CostFunction calculateCost,
                                    std::vector<float>& u,
                                    float h,
                                    float alpha,
                                    float cost_threshold)
{
    std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();
    std::chrono::duration<float> timeout_duration(time_threshold);

    float current_cost = calculateCost(u);
    std::vector<float> gradient(u.size());
    size_t itr_num = 0;
    std::cout << "itr " << itr_num << " " << current_cost << std::endl;

    while ( true )
    {
        itr_num ++;

        // calculate gradient
        Optimiser::calculateGradient(u, h, calculateCost, current_cost, gradient);

        std::cout << "gradient" << std::endl;
        for ( int i = 0; i < gradient.size(); ++i )
        {
            std::cout << gradient[i] << " ";
        }
        std::cout << std::endl;
        /* update u based on gradient and provided alpha */
        for ( int i = 0; i < u.size(); ++i )
        {
            u[i] -= alpha * gradient[i];
        }
        float new_cost = calculateCost(u);
        float delta_cost = current_cost - new_cost;
        current_cost = new_cost;
        std::cout << "itr " << itr_num << " " << current_cost << std::endl;

        if ( fabs(delta_cost) < cost_threshold ||
             std::chrono::system_clock::now() - start_time > timeout_duration )
        {
            break;
        }
    }
}

void Optimiser::calculateOptimalULS(const float& time_threshold,
                                    CostFunction calculateCost,
                                    std::vector<float>& u,
                                    float h,
                                    float starting_alpha,
                                    float ending_alpha,
                                    float cost_threshold)
{
    std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();
    std::chrono::duration<float> timeout_duration(time_threshold);

    float current_cost = calculateCost(u);
    std::vector<float> gradient(u.size());
    size_t itr_num = 0;
    std::cout << "itr " << itr_num << " " << current_cost << std::endl;

    while ( true )
    {
        itr_num ++;
        // calculate gradient
        Optimiser::calculateGradient(u, h, calculateCost, current_cost, gradient);

        // calculate new u with "loosely" optimal alpha
        float alpha = Optimiser::calculateOptimalAlpha(u, calculateCost,
                                                       current_cost, gradient,
                                                       starting_alpha, ending_alpha,
                                                       cost_threshold);
        // std::cout << "alpha " << alpha << std::endl;
        for ( int i = 0; i < u.size(); ++i )
        {
            u[i] -= alpha * gradient[i];
        }
        float new_cost = calculateCost(u);
        float delta_cost = current_cost - new_cost;
        current_cost = new_cost;
        std::cout << "itr " << itr_num << " " << current_cost << std::endl;

        if ( fabs(delta_cost) < cost_threshold ||
             std::chrono::system_clock::now() - start_time > timeout_duration )
        {
            break;
        }
    }
}

void Optimiser::calculateOptimalUCGD(const float& time_threshold,
                                     CostFunction calculateCost,
                                     std::vector<float>& u,
                                     float h,
                                     float starting_alpha,
                                     float ending_alpha,
                                     float cost_threshold)
{
    std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();
    std::chrono::duration<float> timeout_duration(time_threshold);

    float current_cost = calculateCost(u);
    std::vector<float> prev_gradient;
    std::vector<float> prev_conjugate_direction;
    std::vector<float> gradient(u.size());
    size_t itr_num = 0;
    // std::cout << "itr " << itr_num << " " << current_cost << std::endl;

    while ( true )
    {
        itr_num ++;
        // calculate gradient
        Optimiser::calculateGradient(u, h, calculateCost, current_cost, gradient);

        // calculate conjugate direction with polak ribiere method
        std::vector<float> conjugate_direction;
        if ( prev_conjugate_direction.size() == 0 && prev_gradient.size() == 0 )
        {
            conjugate_direction = std::vector<float>(gradient);
        }
        else
        {
            float numerator = 0.0f;
            float denominator = 0.0f;
            for ( size_t i = 0; i < u.size(); ++i )
            {
                numerator += gradient[i] * (gradient[i] - prev_gradient[i]);
                denominator += prev_gradient[i] * prev_gradient[i];
            }
            // float beta_ratio = std::max(0.0f, numerator/denominator);
            float beta_ratio = Utils::clip(numerator/denominator, 1.0f, 0.0f);
            // std::cout << "beta " << beta_ratio << std::endl;
            if ( std::isnan(beta_ratio) )
            {
                break;
            }

            conjugate_direction = std::vector<float>(gradient);
            for ( size_t i = 0; i < u.size(); ++i )
            {
                conjugate_direction[i] -= beta_ratio * prev_conjugate_direction[i];
            }
        }
        prev_gradient = std::vector<float>(gradient);
        prev_conjugate_direction = std::vector<float>(conjugate_direction);

        // calculate new u with "loosely" optimal alpha
        float alpha = Optimiser::calculateOptimalAlpha(u, calculateCost,
                                                       current_cost, conjugate_direction,
                                                       starting_alpha, ending_alpha,
                                                       cost_threshold);

        // std::cout << "alpha " << alpha << std::endl;
        for ( size_t i = 0; i < u.size(); ++i )
        {
            u[i] -= alpha * conjugate_direction[i];
        }
        float new_cost = calculateCost(u);
        float delta_cost = current_cost - new_cost;
        current_cost = new_cost;

        // std::cout << "itr " << itr_num << " " << current_cost << std::endl;
        if ( fabs(delta_cost) < cost_threshold ||
             std::chrono::system_clock::now() - start_time > timeout_duration )
        {
            break;
        }
    }
}

void Optimiser::calculateGradient(const std::vector<float>& u,
                                  float h,
                                  CostFunction calculateCost,
                                  float current_cost,
                                  std::vector<float>& gradient)
{
    for (int i = 0; i < u.size(); ++i)
    {
        std::vector<float> new_u(u);
        new_u[i] += h;
        float new_cost = calculateCost(new_u);
        // gradient.push_back((new_cost - current_cost)/h);
        gradient[i] = (new_cost - current_cost)/h;
    }
}

float Optimiser::calculateCostWithAlpha(const std::vector<float>& u,
                                        CostFunction calculateCost,
                                        const std::vector<float>& gradient,
                                        const float alpha,
                                        std::vector<float>& new_u)
{
    for ( int i = 0; i < new_u.size(); ++i )
    {
        new_u[i] = u[i] - (alpha * gradient[i]);
    }
    return calculateCost(new_u);
}

float Optimiser::calculateOptimalAlphaQN(const std::vector<float>& u,
                                         CostFunction calculateCost,
                                         float current_cost,
                                         const std::vector<float>& gradient,
                                         float smallest_alpha,
                                         float cost_threshold)
{
    std::vector<float> new_u(u.size());

    float new_cost = Optimiser::calculateCostWithAlpha(
            u, calculateCost, gradient, 1.0f, new_u);
    if ( new_cost < current_cost )
    {
        return 1.0f;
    }

    return Optimiser::calculateOptimalAlpha(
            u, calculateCost, current_cost, gradient,
            smallest_alpha, 1.0f, cost_threshold);

}

float Optimiser::calculateOptimalAlpha(const std::vector<float>& u,
                                       CostFunction calculateCost,
                                       float current_cost,
                                       const std::vector<float>& gradient,
                                       float starting_alpha,
                                       float ending_alpha,
                                       float cost_threshold)
{
    float small_alpha = 0.0f;
    float big_alpha = ending_alpha;
    float small_alpha_cost = current_cost;
    float big_alpha_cost = 0.0f;
    std::vector<float> new_u(u.size());

    for (float alpha = starting_alpha; alpha <=ending_alpha; alpha*=10.0f)
    {
        float new_cost = Optimiser::calculateCostWithAlpha(
                u, calculateCost, gradient, alpha, new_u);
        // std::cout << "bracketing " << alpha << " " << new_cost << std::endl;
        if ( new_cost > small_alpha_cost )
        {
            big_alpha = alpha;
            big_alpha_cost = new_cost;
            break;
        }

        float delta_alpha = alpha * 0.1f;
        float alpha_gradient_cost = Optimiser::calculateCostWithAlpha(
                u, calculateCost, gradient, alpha+delta_alpha, new_u);
        float alpha_gradient = (alpha_gradient_cost-new_cost)/delta_alpha;

        if ( alpha_gradient > 0.0f )
        {
            big_alpha = alpha;
            big_alpha_cost = new_cost;
            break;
        }

        small_alpha = alpha;
        small_alpha_cost = new_cost;
    }

    if ( big_alpha == starting_alpha )
    {
        return 0.0f;
    }

    if ( fabs(small_alpha - ending_alpha) < 1e-8 )
    {
        return small_alpha;
    }

    float alpha_threshold = starting_alpha;

    for ( size_t itr_num = 0; itr_num < 10; itr_num++ )
    {
        float alpha = (small_alpha + big_alpha) / 2;
        float new_cost = Optimiser::calculateCostWithAlpha(
                u, calculateCost, gradient, alpha, new_u);

        // std::cout << "selection " << alpha << " " << new_cost << std::endl;
        if ( new_cost > small_alpha_cost )
        {
            big_alpha = alpha;
            big_alpha_cost = new_cost;
        }
        else
        {
            float delta_alpha = alpha * 0.1f;
            float alpha_gradient_cost = Optimiser::calculateCostWithAlpha(
                    u, calculateCost, gradient, alpha+delta_alpha, new_u);
            float alpha_gradient = (alpha_gradient_cost-new_cost)/delta_alpha;
            if ( alpha_gradient > 0.0f )
            {
                big_alpha = alpha;
                big_alpha_cost = new_cost;
            }
            else
            {
                small_alpha = alpha;
                small_alpha_cost = new_cost;
            }
        }

        if ( big_alpha - small_alpha < alpha_threshold || 
             fabs(big_alpha_cost - small_alpha_cost) < cost_threshold )
        {
            break;
        }
    }
    return small_alpha;
}
