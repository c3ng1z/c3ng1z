#include <iostream>
#include <thread>
#include <mutex>
#include <deque>
#include <vector>
#include <random>
#include <atomic>
#include <chrono>
#include <fstream>
#include <memory>

/**
 * @file main.cpp
 * @brief Simple multi-threaded simulation of spacecraft sensors,
 *        processing and basic FDIR monitoring.
 */

using Clock = std::chrono::steady_clock;

struct Measurement {
    Clock::time_point timestamp;
    double value;
};

/**
 * @brief Base class for a threaded sensor.
 *
 * Derived sensors periodically generate measurements on their own thread.
 * The class also allows injecting failures by specifying a time window
 * where no output should be produced.
 */
class SensorBase {
public:
    SensorBase(std::string name, double freq_hz)
        : name_(std::move(name)),
          period_(std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(1.0 / freq_hz))),
          fail_after_(std::chrono::seconds(1000)),
          fail_duration_(std::chrono::seconds(0)) {}

    virtual ~SensorBase() { stop(); }

    void start() {
        start_time_ = Clock::now();
        stop_signal_ = false;
        thread_ = std::thread(&SensorBase::run, this);
    }

    void stop() {
        stop_signal_ = true;
        if (thread_.joinable()) thread_.join();
    }

    void setFailure(Clock::duration after, Clock::duration duration) {
        fail_after_ = after;
        fail_duration_ = duration;
    }

    std::vector<Measurement> fetch() {
        std::lock_guard<std::mutex> lock(mtx_);
        std::vector<Measurement> out(queue_.begin(), queue_.end());
        queue_.clear();
        return out;
    }

    Clock::time_point lastOutput() const { return last_output_; }
    Clock::duration period() const { return period_; }
    const std::string& name() const { return name_; }

protected:
    /// Generate a single measurement value
    virtual double generateValue() = 0;

private:
    void run() {
        std::default_random_engine eng(static_cast<unsigned>(std::hash<std::string>{}(name_)));
        std::normal_distribution<double> noise(0.0, 0.1);
        while (!stop_signal_) {
            auto now = Clock::now();
            if (now - start_time_ >= fail_after_ && now - start_time_ < fail_after_ + fail_duration_) {
                std::this_thread::sleep_for(period_);
                continue; // fail - no output
            }
            Measurement m{now, generateValue() + noise(eng)};
            {
                std::lock_guard<std::mutex> lock(mtx_);
                queue_.push_back(m);
            }
            last_output_ = now;
            std::this_thread::sleep_for(period_);
        }
    }

    std::string name_;
    Clock::duration period_;
    std::thread thread_;
    std::atomic<bool> stop_signal_{false};

    mutable std::mutex mtx_;
    std::deque<Measurement> queue_;
    Clock::time_point last_output_{};
    Clock::time_point start_time_{};

    // failure control
    Clock::duration fail_after_;
    Clock::duration fail_duration_;
};

/**
 * @brief Inertial measurement unit producing attitude rate at 100 Hz.
 */
class IMUSensor : public SensorBase {
public:
    IMUSensor(std::string name) : SensorBase(std::move(name), 100.0) {}
protected:
    double generateValue() override { return 1.0; } // some nominal rate
};

/**
 * @brief GNSS receiver producing position data at 20 Hz.
 */
class GnssSensor : public SensorBase {
public:
    GnssSensor(std::string name) : SensorBase(std::move(name), 20.0) {}
protected:
    double generateValue() override { return 1000.0; } // some nominal position
};

/**
 * @brief Result of one processing cycle.
 */
struct ProcessingOutput {
    Clock::time_point timestamp;
    double attitude{0.0};
    bool attitude_valid{false};
    double position{0.0};
    bool position_valid{false};
};

/**
 * @brief Aggregates sensor data at 50 Hz and writes filtered results.
 */
class Processing {
public:
    Processing(std::vector<std::unique_ptr<IMUSensor>>& imus,
               std::vector<std::unique_ptr<GnssSensor>>& gnss,
               const std::string& logfile)
        : imus_(imus), gnss_(gnss), logfile_(logfile) {}

    ~Processing() { stop(); }

    void start() {
        stop_signal_ = false;
        thread_ = std::thread(&Processing::run, this);
    }

    void stop() {
        stop_signal_ = true;
        if (thread_.joinable()) thread_.join();
    }

    ProcessingOutput lastOutput() const {
        std::lock_guard<std::mutex> lock(output_mtx_);
        return last_output_;
    }

private:
    void run() {
        std::ofstream log(logfile_);
        auto period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / 50.0)); // 50Hz
        auto last_tick = Clock::now();
        while (!stop_signal_) {
            auto now = Clock::now();
            // IMU processing
            double sum = 0.0;
            int count = 0;
            for (auto& imu : imus_) {
                auto data = imu->fetch();
                for (const auto& m : data) {
                    sum += m.value;
                    ++count;
                }
            }
            ProcessingOutput out{};
            out.timestamp = now;
            if (count > 0) {
                out.attitude = sum / count;
                out.attitude_valid = true;
            } else {
                out.attitude_valid = false;
            }
            // GNSS processing: keep the most recent measurement from any receiver
            for (auto& g : gnss_) {
                auto data = g->fetch();
                if (!data.empty()) {
                    last_gnss_ = data.back();
                }
            }
            if (last_gnss_.timestamp.time_since_epoch().count() != 0 &&
                now - last_gnss_.timestamp < std::chrono::seconds(1)) {
                out.position = last_gnss_.value;
                out.position_valid = true;
            } else {
                out.position_valid = false;
            }
            {
                std::lock_guard<std::mutex> lock(output_mtx_);
                last_output_ = out;
            }
            // log
            log << std::chrono::duration<double>(out.timestamp.time_since_epoch()).count()
                << "," << out.attitude << "," << out.attitude_valid
                << "," << out.position << "," << out.position_valid << std::endl;

            std::this_thread::sleep_until(last_tick + period);
            last_tick += period;
        }
    }

    std::vector<std::unique_ptr<IMUSensor>>& imus_;
    std::vector<std::unique_ptr<GnssSensor>>& gnss_;
    std::thread thread_;
    std::atomic<bool> stop_signal_{false};
    mutable std::mutex output_mtx_;
    ProcessingOutput last_output_{};
    Measurement last_gnss_{};
    std::string logfile_;
};

/**
 * @brief Fault detection logic monitoring sensors and processing output.
 *
 * Raises alarms if a sensor stops producing data or if processing lacks
 * valid output.
 */
class FDIR {
public:
    FDIR(std::vector<SensorBase*>& sensors, Processing& proc, const std::string& logfile)
        : sensors_(sensors), proc_(proc), logfile_(logfile) {}
    ~FDIR() { stop(); }

    void start() {
        stop_signal_ = false;
        thread_ = std::thread(&FDIR::run, this);
    }

    void stop() {
        stop_signal_ = true;
        if (thread_.joinable()) thread_.join();
    }

private:
    void run() {
        std::ofstream log(logfile_);
        auto period = std::chrono::duration_cast<Clock::duration>(std::chrono::milliseconds(100)); // 10Hz monitoring
        auto last_tick = Clock::now();
        while (!stop_signal_) {
            auto now = Clock::now();
            for (auto* s : sensors_) {
                auto delta = now - s->lastOutput();
                if (delta > 3 * s->period()) {
                    log << std::chrono::duration<double>(now.time_since_epoch()).count()
                        << ",SENSOR_ALARM," << s->name() << std::endl;
                }
            }
            ProcessingOutput out = proc_.lastOutput();
            if (!(out.attitude_valid && out.position_valid)) {
                log << std::chrono::duration<double>(now.time_since_epoch()).count()
                    << ",PROC_ALARM" << std::endl;
            }
            std::this_thread::sleep_until(last_tick + period);
            last_tick += period;
        }
    }

    std::vector<SensorBase*>& sensors_;
    Processing& proc_;
    std::thread thread_;
    std::atomic<bool> stop_signal_{false};
    std::string logfile_;
};

/**
 * @brief Helper to configure sensors and run different scenarios.
 */
class Simulation {
public:
    Simulation(const std::string& name) : name_(name) {}

    void run_nominal() {
        setup();
        startAll();
        std::this_thread::sleep_for(std::chrono::seconds(10));
        stopAll();
    }

    void run_imu_fail() {
        setup();
        imus_[0]->setFailure(std::chrono::seconds(2), std::chrono::seconds(100));
        imus_[1]->setFailure(std::chrono::seconds(4), std::chrono::seconds(100));
        imus_[2]->setFailure(std::chrono::seconds(6), std::chrono::seconds(100));
        startAll();
        std::this_thread::sleep_for(std::chrono::seconds(10));
        stopAll();
    }

    void run_gnss_dropout() {
        setup();
        for (auto& g : gnss_) {
            g->setFailure(std::chrono::seconds(5), std::chrono::milliseconds(500));
        }
        startAll();
        std::this_thread::sleep_for(std::chrono::seconds(10));
        stopAll();
    }

private:
    void setup() {
        imus_.clear();
        gnss_.clear();
        sensors_.clear();

        for (int i = 0; i < 3; ++i) {
            imus_.push_back(std::make_unique<IMUSensor>("IMU" + std::to_string(i + 1)));
        }
        for (int i = 0; i < 2; ++i) {
            gnss_.push_back(std::make_unique<GnssSensor>("GNSS" + std::to_string(i + 1)));
        }
        for (auto& imu : imus_) sensors_.push_back(imu.get());
        for (auto& g : gnss_) sensors_.push_back(g.get());

        proc_ = std::make_unique<Processing>(imus_, gnss_, name_ + "_proc.log");
        fdir_ = std::make_unique<FDIR>(sensors_, *proc_, name_ + "_fdir.log");
    }

    void startAll() {
        for (auto* s : sensors_) s->start();
        proc_->start();
        fdir_->start();
    }

    void stopAll() {
        for (auto* s : sensors_) s->stop();
        proc_->stop();
        fdir_->stop();
        sensors_.clear();
        imus_.clear();
        gnss_.clear();
    }

    std::string name_;
    std::vector<std::unique_ptr<IMUSensor>> imus_;
    std::vector<std::unique_ptr<GnssSensor>> gnss_;
    std::vector<SensorBase*> sensors_;
    std::unique_ptr<Processing> proc_;
    std::unique_ptr<FDIR> fdir_;
};

/// Entry point. Selects a scenario and runs the simulation.
int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " [nominal|imu_fail|gnss_fail]" << std::endl;
        return 0;
    }
    std::string mode = argv[1];
    Simulation sim(mode);
    if (mode == "nominal") sim.run_nominal();
    else if (mode == "imu_fail") sim.run_imu_fail();
    else if (mode == "gnss_fail") sim.run_gnss_dropout();
    else {
        std::cerr << "Unknown mode" << std::endl;
        return 1;
    }
    return 0;
}

