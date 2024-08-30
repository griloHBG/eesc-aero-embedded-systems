//
// Created by grilo on 20/07/23.
//

#ifndef MANOPLA_SYSTEM_IDENTIFICATION_TEST_MANOPLALELYBBB_H
#define MANOPLA_SYSTEM_IDENTIFICATION_TEST_MANOPLALELYBBB_H
#include <iostream>
#include <cmath>
#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/io2/can_rt.hpp>
#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/master.hpp>
#include <lely/ev/co_task.hpp>

#include "utils.h"
#include <functional>
#include <memory>
#include <iomanip>

#ifndef M_PIf32
    # define M_PIf32	3.141592653589793238462643383279502884f /* pi */
#endif //M_PIf32

using namespace std::chrono_literals;
namespace manopla {

    enum class MotionMode {
        // Profile Position Mode
        ProfilePositionMode,
        // Homing Mode - Not Implemented
        HomingMode, //Not Implemented
        // Profile Velocity Mode
        ProfileVelocityMode,
        // Position Mode
        PositionMode,
        // Velocity Mode
        VelocityMode,
        // Current Mode
        CurrentMode,
    };

    struct Time {
        // time in microseconds from last OnSync
        uint64_t last_us = 0;
        // current absolute time in microseconds according to the Operating System (OS)
        uint64_t current_us = 0;
        // biggest ever sampling time for OnSync
        uint64_t max_dt_us = 0;
        // smallest ever sampling time for OnSync
        uint64_t min_dt_us = 9999999999;
        // total sum of all past sampling times (useful to calculate average sampling time)
        uint64_t sum_total_dt = 0;
        // current sampling time (time between current call of OnSync and the last one)
        uint64_t current_dt_us = 0;
        // amount of times that OnSync was called until now (useful to calculate average sampling time)
        uint64_t dt_count = 0;
    };

    struct MotorInfo {
        int32_t currentPulses; //[qc]
        int32_t prevPulses; //[qc]
        float currentAngle; //[rad]
        float prevAngle; //[rad]
        int32_t currentRotationUnfiltered; //[rad/s]
        int32_t prevRotationUnfiltered; //[rad/s]
        int32_t currentRotationFiltered; //[rad/s]
        int32_t prevRotationFiltered; //[rad/s]
        int16_t currentCurrent; //[mA]
        int16_t prevCurrent; //[mA]
    };

    class MyDriver;

    using SyncCallback = std::function<void(const Time&, const MotorInfo&, MyDriver& driver)>;

    class ManoplaLelyBBB {
    public:
        explicit ManoplaLelyBBB(const std::string&);
        void prepare(int bitrate,
                     const std::string& dcf_path,
                     const std::string& bin_path,
                     MotionMode motionMode);
        void installOnSyncCallback(const SyncCallback& onSyncCallback);

        void start_loop();

        void stop();

    private:
        std::string can_interface_name;
        // Initialize the I/O library. This is required on Windows, but a no-op on
        // Linux (for now).
        lely::io::IoGuard io_guard;
        // Create an I/O context to synchronize I/O services during shutdown.
        lely::io::Context ctx;
        // Create an platform-specific I/O polling instance to monitor the CAN bus, as
        // well as timers and signals.
        lely::io::Poll poll;
        // Create a polling event loop and pass it the platform-independent polling
        // interface. If no tasks are pending, the event loop will poll for I/O
        // events.
        lely::ev::Loop loop;
    public:
        // I/O devices only need access to the executor interface of the event loop.
        lely::ev::Executor exec;
    private:
        // Create a timer using a monotonic clock, i.e., a clock that is not affected
        // by discontinuous jumps in the system time.
        lely::io::Timer timer;
        // Create a virtual SocketCAN CAN controller and channel, and do not modify
        // the current CAN bus state or bitrate.
        lely::io::CanController ctrl;
        lely::io::CanChannel chanCANopenMaster;
        // Create a CANopen master with node-ID 2. The master is asynchronous, which
        // means every user-defined callback for a CANopen event will be posted as a
        // task on the event loop, instead of being invoked during the event
        // processing by the stack
        //std::unique_ptr<lely::canopen::AsyncMaster> master;
        std::shared_ptr<lely::canopen::AsyncMaster> master;
        lely::io::SignalSet sigset;

        std::unique_ptr<MyDriver> driver;

    };

    using namespace lely::canopen;

    class MyDriver : public FiberDriver {
        using FiberDriver::FiberDriver;
    public:
        MyDriver(ev_exec_t* exec, AsyncMaster& master, uint8_t id, MotionMode motionMode);

        void start();

    private:
        int maxVel = 2000;
        int minVel = -2000;

        int counter = 0;
        int maxCounter = 50;

        float prevPosition = 0; // previous position [rad]
        MotionMode motionMode;
        SyncCallback onSyncCallback;


    public:
        float velocity = static_cast<float>(maxVel);
        float direction = 3;
        int32_t currentPulses;
        float currentPosition;
        float currentVelocity;
        float motorzaoTorqueConstant = 38.5f * 1e-3; // 0.0385 Nm/A
        float gear = 3.5f;
        float controlSignal = 0;
        float motorCurrent_mA = 0;
        float pulsesPerTurn = 2000;

        float angularCorrection = 0;

        void setOnSyncCallback(const SyncCallback& onSyncCallback);

    private:
        //This function gets called when the boot-up process of the slave completes.
        // The 'st' parameter contains the last known NMT state of the slave
        // (typically pre-operation), 'es' the error code (0 on success), and 'what'
        // a description of the error, if any.
        void
        OnBoot(NmtState /*st*/, char es,
               const std::string &what) noexcept override;

        // This function gets called during the boot-up process for the slave. The
        // 'res' parameter is the function that MUST be invoked when the configuration
        // is complete. Because this function runs as a task inside a coroutine, it
        // can suspend itself and wait for an asynchronous function such as an SDO
        // request, to complete.
        void
        OnConfig(std::function<void(std::error_code ec)> res) noexcept override;

        void SetProfilePositionMode(uint32_t maxFollowingError = 2000, /* qc 0x6065.00 */
                                    int32_t minPositionLimit = -2147483648, /* qc 0x607D.01 */
                                    int32_t maxPositionLimit = 2147483647, /* qc 0x607D.02 */
                                    uint32_t maxProfileVelocity = 25000, /* rpm 0x607F.00 */
                                    uint32_t profileVelocity = 1000, /* rpm 0x6081.00 */
                                    uint32_t profileAcceleration = 10000, /* rpm/s 0x6083.00 */
                                    uint32_t profileDeceleration = 10000, /* rpm/s 0x6084.00 */
                                    uint32_t quickstopDeceleration = 10000, /* rpm/s 0x6085.00 */
                                    int16_t motionProfileType = 0 /* 0 for linear or 1 for sin² 0x6086.00 */);


        void SetCurrentMode(uint16_t continuousCurrentLimit = 5000, /* mA 0x6410.01 */
                            uint16_t maxSpeed = 9500, /* rpm 0x6410.04 */
                            uint16_t thermalTimeConstantWinding = 70 /* ? 0x6410.05 */
        );

        void SetPositionMode(uint32_t maxFollowingError = 2000, /* qc 0x6065.00 */
                             int32_t minPositionLimit = -2147483648, /* qc 0x607D.01 */
                             int32_t maxPositionLimit = 2147483647 /* qc 0x607D.02 */);

        void SetProfileVelocityMode(uint32_t maxProfileVelocity = 25000, /* rpm */
                                    uint32_t profileAcceleration = 10000, /* rpm/s */
                                    uint32_t profileDeceleration = 10000, /* rpm/s */
                                    uint32_t quickStopDeceleration = 10000, /* rpm/s */
                                    uint8_t motionProfileType = 0 /* 0: linear, 1: sin^2 */);

        void Shutdown();

        void SwitchOn();

        void StartAbsPositioningImmediate();

        void OnSync(uint8_t cnt, const time_point &t) noexcept override;

        // This function is similar to OnConfig(), but it gets called by the
        // AsyncDeconfig() method of the master.
        void
        OnDeconfig(std::function<void(std::error_code ec)> res) noexcept override;
    };

#endif //MANOPLA_SYSTEM_IDENTIFICATION_TEST_MANOPLALELYBBB_H
} // namespace manopla