#include "MCL.h"
#include "globals.h"

// --- Globals ---
namespace MCL {
    // Current Global Estimate
    double X = 0, Y = 0, theta = 0;
    
    std::mt19937 Random(123); // Fixed seed for reproducibility
    pros::Mutex particle_mutex;
    
    // Particle Containers
    std::vector<Particle> Particles(num_particles);
    std::vector<Particle> Resampled(num_particles);
    std::vector<double> CDF(num_particles);
    
    // Sensors & Field
    std::vector<MCLDistanceSensor> Sensors; 
    // We use a list of indices to track active sensors to avoid copying full objects
    std::vector<int> activeSensorIndices; 
    Field field_;
    
    // Tracking Variables
    lemlib::Pose lastPose(0,0,0);
    bool isInitialized = false;

    // Constants
    static constexpr double sqrt_2_pi = 2.506628275;
    static constexpr double inv_num_particles = 1.0 / num_particles;

    // Placeholder
    double getAvgVelocity(void) { return 0.0; }
}

// --- Field Implementation ---
Field::Field() {
    //Longgoals
    Goals.push_back(Goal(Point(-24, -48 + 2.5), 2.5));
    Goals.push_back(Goal(Point(24, -48 + 2.5), 2.5));
    Goals.push_back(Goal(Point(-24, 48 - 2.5), 2.5));
    Goals.push_back(Goal(Point(24, 48 - 2.5), 2.5));
    // Match Loaders
    Goals.push_back(Goal(Point(-72 + 2.5,-48), 2.3));
    Goals.push_back(Goal(Point(-72 + 2.5,48), 2.3));
    Goals.push_back(Goal(Point(72 - 2.5,-48), 2.3));
    Goals.push_back(Goal(Point(72 - 2.5,48), 2.3));
    //Center goal
    Goals.push_back(Goal(Point(0,0), 4));
}

// Raycasting Logic
float Field::get_sensor_distance(const Particle& p, const MCLDistanceSensor& Sensor) const {
    // Local copies for const correctness
    Point offsetCopy = Sensor.Offset; 
    Point stepCopy = p.step; 

    // 1. Calculate Sensor World Position
    //    Rotate offset by particle heading (p.step is pre-calculated cos/sin)
    Point sensor_position = Point(p.x, p.y) + offsetCopy.rotate(p.step.x, p.step.y);
    
    // 2. Calculate Sensor Look Vector
    //    Rotate the particle's heading by the sensor's mounting direction
    Point step_vector = stepCopy.rotate(this->direction_to_cosine[Sensor.Dir], this->direction_to_sine[Sensor.Dir]);
    
    float min_distance = 1e10;
    bool Intersection = false;

    // Check Goals
    for (const Goal &G : this->Goals) {
        Point goalPos = G.Position; 
        Point v = goalPos - sensor_position;

        // Optimization: Only check goals roughly in front of the ray
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
    
    // Check Walls
    float wall_distance = 1e10;
    
    // X-Walls
    if (std::abs(step_vector.x) > 1e-4f) {
        const float wall_x = step_vector.x > 0 ? this->HalfSize : -this->HalfSize;
        float t = (wall_x - sensor_position.x) / step_vector.x;
        if (t > 0 && std::abs(t * step_vector.y + sensor_position.y) <= this->HalfSize) {
             wall_distance = t;
        }
    }

    // Y-Walls 
    if (std::abs(step_vector.y) > 1e-4f) {
        const float wall_y = step_vector.y > 0 ? this->HalfSize : -this->HalfSize;
        float t = (wall_y - sensor_position.y) / step_vector.y;
        if (t > 0 && t < wall_distance && std::abs(t * step_vector.x + sensor_position.x) <= this->HalfSize) {
            wall_distance = t;
        }
    }
    
    return wall_distance;
}

// --- Helper: Metric Distance ---
inline double getMetricDistanceSq(const Particle& a, const Particle& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return (dx*dx + dy*dy);
}

// --- Start Function ---
void MCL::StartMCL(double x_, double y_, double theta_) {
    X = x_; Y = y_; theta = theta_;
    isInitialized = true;

    // Reset Odometry Tracker
    lastPose = chassis.getPose(true); 

    Sensors.clear();
    // === CONFIGURE SENSORS HERE ===
    // Sensors.emplace_back(pros::Distance(10), Point(5.0, 0.0), FRONT);
    // ==============================

    // Resize if needed
    if(Particles.size() != num_particles) Particles.resize(num_particles); 
    if(Resampled.size() != num_particles) Resampled.resize(num_particles);
    if(CDF.size() != num_particles) CDF.resize(num_particles);

    std::normal_distribution<double> dist_x(x_, 1.0);
    std::normal_distribution<double> dist_y(y_, 1.0);
    
    double fixed_theta = theta_; 

    for (auto& p : Particles) {
        p.x = dist_x(Random);
        p.y = dist_y(Random);
        p.theta = fixed_theta; 
        p.step = Point(cos(p.theta), sin(p.theta));
        p.weight = inv_num_particles;
        p.sigma = MCLConfig::SIGMA_MIN; 
    }

    pros::Task mcl_task([]{ MonteCarlo(); }, "MCL Task");
    printf("MCL Started. IMU Trusted Mode.\n");
}

// --- Main MCL Loop ---
void MCL::MonteCarlo(void) {
    using namespace MCLConfig;

    constexpr uint32_t FAST_LOOP_DELAY = 10; 
    constexpr uint32_t SENSOR_DELAY = 40; 
    
    uint32_t last_sensor_update = 0;

    while (true) {
        if (!isInitialized) { pros::delay(100); continue; }

        uint32_t now = pros::millis();
        bool doSensorUpdate = (now - last_sensor_update >= SENSOR_DELAY);

        // ==========================================
        // 1. DATA GATHERING (UNLOCKED)
        // ==========================================
        // Perform hardware interactions here to avoid blocking the mutex
        
        // A. Odometry
        lemlib::Pose currentPose = chassis.getPose(true);
        
        // B. Sensors
        // We clear the index list, but we don't modify the `Sensors` vector structure,
        // we only modify the `measurement` member of the elements.
        if (doSensorUpdate) {
            // Note: We don't clear activeSensorIndices here, we do it in the locked section
            // to ensure the filter logic uses the fresh data.
            for (auto& sensor : Sensors) {
                sensor.Measure(); 
            }
        }

        // ==========================================
        // 2. PARTICLE FILTER (LOCKED)
        // ==========================================
        particle_mutex.take();

        // --- A. Prediction (Motion Model) ---
        double dX = currentPose.x - lastPose.x;
        double dY = currentPose.y - lastPose.y;
        double currentTheta = currentPose.theta; 
        
        lastPose = currentPose; 

        double distMoved = std::sqrt(dX*dX + dY*dY);
        double motionNoiseStd = std::max(0.05, distMoved * 0.10); 
        std::normal_distribution<double> noise_dist(0, motionNoiseStd);

        // Precompute Trig for Trusted IMU
        double rot_theta = M_PI_2 - currentTheta; // Convert to Std Math (0=East)
        float cos_t = cosf(rot_theta);
        float sin_t = sinf(rot_theta);
        Point step_vec(cos_t, sin_t);

        for (auto& p : Particles) {
            p.x += dX + noise_dist(Random);
            p.y += dY + noise_dist(Random);
            
            p.theta = currentTheta;
            p.step = step_vec;

            // Clamp to field
            p.x = std::clamp(p.x, -70.0, 70.0);
            p.y = std::clamp(p.y, -70.0, 70.0);
        }

        // --- B. Correction (Sensor Model) ---
        if (doSensorUpdate) {
            activeSensorIndices.clear();
            for (int i = 0; i < Sensors.size(); i++) {
                if (Sensors[i].measurement > 0) activeSensorIndices.push_back(i);
            }

            if (!activeSensorIndices.empty()) {
                
                // 1. Adaptive Sigma (Stride Optimization)
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

                // 2. Likelihood
                double max_log_weight = -1e9;

                for (auto& P : Particles) {
                    double log_w = 0.0;
                    double var = P.sigma * P.sigma;
                    double log_norm_const = -std::log(P.sigma); 

                    for (int idx : activeSensorIndices) {
                        const auto& sensor = Sensors[idx];
                        double pred = field_.get_sensor_distance(P, sensor);
                        double diff = 0;

                        // Raycast infinite (miss) check
                        if (pred >= 1800.0) {
                            // If raycast sees nothing, but sensor sees something close -> Bad
                            // If raycast sees nothing, and sensor sees max range -> Good (Both see nothing)
                            double maxRangeInches = sensor.Range; 
                            if (sensor.measurement < (maxRangeInches - 5.0)) {
                                diff = 100.0; // Penalty
                            } else {
                                diff = 0.0; // Match (Both void)
                            }
                        } else {
                            diff = pred - sensor.measurement;
                        }

                        log_w += log_norm_const - (0.5 * diff * diff / var);
                    }
                    
                    P.weight = log_w;
                    if (log_w > max_log_weight) max_log_weight = log_w;
                }

                // 3. Normalize
                double weights_sum = 0.0;
                double sq_weights_sum = 0.0;

                for (auto& P : Particles) {
                    P.weight = std::exp(P.weight - max_log_weight);
                    weights_sum += P.weight;
                }

                if (weights_sum < 1e-10) {
                    // Kidnapped recovery
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

                // 4. Resampling
                double ESS = 1.0 / sq_weights_sum;
                
                if (ESS < (Particles.size() * ESS_RATIO)) {
                    CDF[0] = Particles[0].weight;
                    for (int i = 1; i < Particles.size(); ++i) {
                        CDF[i] = CDF[i - 1] + Particles[i].weight;
                    }

                    // Systemic Resampling
                    std::uniform_real_distribution<double> dist(0.0, inv_num_particles);
                    double threshold = dist(Random); 
                    int index = 0;

                    for (int i = 0; i < Particles.size(); ++i) {
                        while (threshold > CDF[index] && index < Particles.size() - 1) {
                            index++;
                        }
                        Resampled[i] = Particles[index];
                        Resampled[i].weight = inv_num_particles;
                        threshold += inv_num_particles;
                    }
                    Particles = Resampled;
                }
            }
            last_sensor_update = now;
        }

        // --- C. Estimation ---
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
        theta = currentPose.theta; // Trusted IMU

        particle_mutex.give();
        pros::Task::delay(FAST_LOOP_DELAY);
    }
}