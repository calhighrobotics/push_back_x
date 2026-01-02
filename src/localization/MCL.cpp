#include "MCL.h"
#include "globals.h"

namespace MCL {
    double X = 0, Y = 0, theta = 0;
    std::mt19937 Random(123);
    pros::Mutex particle_mutex;
    std::vector<Particle> Particles(num_particles);
    std::vector<Particle> Resampled(num_particles);
    std::vector<double> CDF(num_particles);
    std::vector<MCLDistanceSensor> Sensors; 
    std::vector<int> activeSensorIndices; 
    Field field_;
    lemlib::Pose lastPose(0,0,0);
    bool isInitialized = false;
    static constexpr double sqrt_2_pi = 2.506628275;
    static constexpr double inv_num_particles = 1.0 / num_particles;
    double getAvgVelocity(void) { return 0.0; }
}

Field::Field() {
    Goals.push_back(Goal(Point(-24, -48 + 2.5), 2.5));
    Goals.push_back(Goal(Point(24, -48 + 2.5), 2.5));
    Goals.push_back(Goal(Point(-24, 48 - 2.5), 2.5));
    Goals.push_back(Goal(Point(24, 48 - 2.5), 2.5));
    Goals.push_back(Goal(Point(-72 + 2.5,-48), 2.3));
    Goals.push_back(Goal(Point(-72 + 2.5,48), 2.3));
    Goals.push_back(Goal(Point(72 - 2.5,-48), 2.3));
    Goals.push_back(Goal(Point(72 - 2.5,48), 2.3));
    Goals.push_back(Goal(Point(0,0), 4));
}

float Field::get_sensor_distance(const Particle& p, const MCLDistanceSensor& Sensor) const {
    Point offsetCopy = Sensor.Offset; 
    Point sensor_position = Point(p.x, p.y) + offsetCopy.rotate(p.step.x, p.step.y);
    Point step_vector = p.step.rotate(this->direction_to_cosine[Sensor.Dir], this->direction_to_sine[Sensor.Dir]);
    float min_distance = 1e10;
    bool Intersection = false;

    for (const Goal &G : this->Goals) {
        Point goalPos = G.Position; 
        Point v = goalPos - sensor_position;

        if (((step_vector.x < 0) == (v.x < 0) && (step_vector.y < 0) == (v.y < 0)) || v.norm_squared() <= G.RadiusSquared) {
            double proj = v.dot(step_vector);
            double perp_squared = v.norm_squared() - proj * proj;

            if (perp_squared <= G.RadiusSquared) {
                auto t = proj - std::sqrt(G.RadiusSquared - perp_squared);
                if (t >= 0 && t <= min_distance) { 
                    Intersection = true;
                    min_distance = t;
                }   
            }
        }
    }

    if (Intersection) return min_distance;
    
    float wall_distance = 1e10;
    
    if (std::abs(step_vector.x) > 1e-4f) {
        const float wall_x = step_vector.x > 0 ? this->HalfSize : -this->HalfSize;
        float t = (wall_x - sensor_position.x) / step_vector.x;
        if (t > 0 && std::abs(t * step_vector.y + sensor_position.y) <= this->HalfSize) {
             wall_distance = t;
        }
    }

    if (std::abs(step_vector.y) > 1e-4f) {
        const float wall_y = step_vector.y > 0 ? this->HalfSize : -this->HalfSize;
        float t = (wall_y - sensor_position.y) / step_vector.y;
        if (t > 0 && t < wall_distance && std::abs(t * step_vector.x + sensor_position.x) <= this->HalfSize) {
            wall_distance = t;
        }
    }
    
    return wall_distance;
}

inline double getMetricDistanceSq(const Particle& a, const Particle& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return (dx*dx + dy*dy);
}

void MCL::StartMCL(double x_, double y_, double theta_) {
    particle_mutex.take();
    X = x_; Y = y_; theta = theta_;
    isInitialized = true;
    lastPose = chassis.getPose(true); 
    Sensors.clear();
    Sensors.emplace_back(pros::Distance(13), Point(-5.5, -2), FRONT);
    Sensors.emplace_back(pros::Distance(12), Point(-5, 0), LEFT);
    Sensors.emplace_back(pros::Distance(6), Point(5, 0), RIGHT);
    Sensors.emplace_back(pros::Distance(11), Point(-4.75, -7.5), BACK);

    if(Particles.size() != num_particles) Particles.resize(num_particles); 
    if(Resampled.size() != num_particles) Resampled.resize(num_particles);
    if(CDF.size() != num_particles) CDF.resize(num_particles);

    std::normal_distribution<double> dist_x(x_, 1.0);
    std::normal_distribution<double> dist_y(y_, 1.0);
    
    double standard_angle = M_PI_2 - theta_;
    Point initial_step(cos(standard_angle), sin(standard_angle));

    for (auto& p : Particles) {
        p.x = dist_x(Random);
        p.y = dist_y(Random);
        p.theta = theta_; 
        p.step = initial_step; 
        p.weight = inv_num_particles;
        p.sigma = MCLConfig::SIGMA_MIN; 
    }

    particle_mutex.give();

    static bool taskStarted = false;
    if (!taskStarted) {
        pros::Task mcl_task([]{ MonteCarlo(); }, "MCL Task");
        taskStarted = true;
        printf("MCL Started. IMU Trusted Mode.\n");
    }
}

void MCL::MonteCarlo(void) {
    using namespace MCLConfig;

    constexpr uint32_t FAST_LOOP_DELAY = 10; 
    constexpr uint32_t SENSOR_DELAY = 40; 
    
    uint32_t last_sensor_update = 0;
    uint32_t last_log_time = 0;
    double current_ESS = Particles.size(); 

    while (true) {
        if (!isInitialized) { pros::delay(100); continue; }

        uint32_t now = pros::millis();
        bool doSensorUpdate = (now - last_sensor_update >= SENSOR_DELAY);

        lemlib::Pose currentPose = chassis.getPose(true);
        if (doSensorUpdate) {
            for (auto& sensor : Sensors) sensor.Measure(); 
        }

        particle_mutex.take();

        double dX = currentPose.x - lastPose.x;
        double dY = currentPose.y - lastPose.y;
        double currentTheta = currentPose.theta; 
        lastPose = currentPose; 

        double distMoved = std::sqrt(dX*dX + dY*dY);
        double motionNoiseStd = std::max(0.05, distMoved * 0.10); 
        std::normal_distribution<double> noise_dist(0, motionNoiseStd);

        double rot_theta = M_PI_2 - currentTheta; 
        Point step_vec(cosf(rot_theta), sinf(rot_theta));

        for (auto& p : Particles) {
            p.x += dX + noise_dist(Random);
            p.y += dY + noise_dist(Random);
            p.theta = currentTheta;
            p.step = step_vec;
            p.x = std::clamp(p.x, -70.0, 70.0);
            p.y = std::clamp(p.y, -70.0, 70.0);
        }

        if (doSensorUpdate) {
            activeSensorIndices.clear();
            for (int i = 0; i < Sensors.size(); i++) {
                if (Sensors[i].measurement > 0 && Sensors[i].measurement < Sensors[i].Range) {
                     activeSensorIndices.push_back(i);
                }
            }

            if (!activeSensorIndices.empty()) {
                const int STRIDE = 20; 
                for (int i = 0; i < Particles.size(); ++i) {
                    double min_dist_sq = 1e9;
                    for (int j = 0; j < Particles.size(); j += STRIDE) {
                        if (i == j) continue;
                        double d2 = getMetricDistanceSq(Particles[i], Particles[j]);
                        if (d2 < min_dist_sq) min_dist_sq = d2;
                    }
                    double r = std::sqrt(min_dist_sq) * 0.5;
                    Particles[i].sigma = std::clamp(ADAPTIVE_ALPHA * r, SIGMA_MIN, SIGMA_MAX);
                }

                double max_log_weight = -1e9;
                for (auto& P : Particles) {
                    double log_w = 0.0;
                    double var = P.sigma * P.sigma;
                    double log_norm_const = -std::log(P.sigma); 

                    for (int idx : activeSensorIndices) {
                        const auto& sensor = Sensors[idx];
                        double pred = field_.get_sensor_distance(P, sensor);
                        double diff = 0;
                        if (pred >= 1000.0) {
                            if (sensor.measurement < (sensor.Range - 5.0)) diff = 100.0; 
                        } else {
                            diff = pred - sensor.measurement;
                        }
                        log_w += log_norm_const - (0.5 * diff * diff / var);
                    }
                    P.weight = log_w;
                    if (log_w > max_log_weight) max_log_weight = log_w;
                }

                double weights_sum = 0.0;
                double sq_weights_sum = 0.0;
                for (auto& P : Particles) {
                    P.weight = std::exp(P.weight - max_log_weight);
                    weights_sum += P.weight;
                }

                if (weights_sum < 1e-10) {
                    double uniform = inv_num_particles;
                    for (auto &p : Particles) p.weight = uniform;
                    sq_weights_sum = uniform; 
                } else {
                    double inv_sum = 1.0 / weights_sum;
                    for (auto &p : Particles) {
                        p.weight *= inv_sum;
                        sq_weights_sum += (p.weight * p.weight);
                    }
                }

                current_ESS = 1.0 / sq_weights_sum;
                
                if (current_ESS < (Particles.size() * ESS_RATIO)) {
                    CDF[0] = Particles[0].weight;
                    for (int i = 1; i < Particles.size(); ++i) CDF[i] = CDF[i - 1] + Particles[i].weight;
                    
                    std::uniform_real_distribution<double> dist(0.0, inv_num_particles);
                    double threshold = dist(Random); 
                    int index = 0;
                    for (int i = 0; i < Particles.size(); ++i) {
                        while (threshold > CDF[index] && index < Particles.size() - 1) index++;
                        Resampled[i] = Particles[index];
                        Resampled[i].weight = inv_num_particles;
                        threshold += inv_num_particles;
                    }
                    Particles = Resampled;
                    current_ESS = Particles.size(); 
                }
            }
            last_sensor_update = now;
        }

        double new_x = 0.0, new_y = 0.0, total_weight = 0.0;
        for (const auto& p : Particles) {
            new_x += p.x * p.weight;
            new_y += p.y * p.weight;
            total_weight += p.weight;
        }
        if (total_weight > 0) {
            X = new_x / total_weight;
            Y = new_y / total_weight;
        }
        theta = currentPose.theta; 

        particle_mutex.give();

        if (now - last_log_time >= 500) {
            printf("MCL State: X: %.2f | Y: %.2f | ESS: %.1f\n", X, Y, current_ESS);
            last_log_time = now;
        }

        pros::Task::delay(FAST_LOOP_DELAY);
    }
}
