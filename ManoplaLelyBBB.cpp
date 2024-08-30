//
// Created by grilo on 20/07/23.
//

#include "ManoplaLelyBBB.h"
#include <bitset>

namespace manopla {

    MotorInfo motorInfo;
    Time time{};

    manopla::ManoplaLelyBBB::ManoplaLelyBBB(const std::string& can_interface_name):
    can_interface_name(can_interface_name),
    poll(lely::io::Poll(ctx)),
    loop(poll.get_poll()),
    exec(loop.get_executor()),
    timer(poll,exec,CLOCK_MONOTONIC),
    ctrl(can_interface_name.c_str()),
    chanCANopenMaster(poll, exec),
    sigset(poll, exec)
    {}

    void manopla::ManoplaLelyBBB::prepare(
            int bitrate,
            const std::string &dcf_path,
            const std::string &bin_path,
            MotionMode motionMode) {

        if(can_interface_name.substr(0,4) != "vcan") {
            //ctrl.set_bitrate(bitrate);
        }

        chanCANopenMaster.open(ctrl);

        // Create a CANopen master with node-ID 2. The master is asynchronous, which
        // means every user-defined callback for a CANopen event will be posted as a
        // task on the event loop, instead of being invoked during the event
        // processing by the stack
        //master = std::make_unique<lely::canopen::AsyncMaster>(timer, chanCANopenMaster, dcf_path, bin_path, 2);
        master = std::make_shared<lely::canopen::AsyncMaster>(timer, chanCANopenMaster, dcf_path, bin_path, 2);

        // Create a driver for the slave with node-ID 1.
        driver = std::make_unique<MyDriver>(exec, *master, 1, motionMode);

        //Watch for Ctrl+C or process termination.
        sigset.insert(SIGHUP);
        sigset.insert(SIGINT);
        sigset.insert(SIGTERM);

        //Submit a task to be executed when a signal is raised. We don't care which.
        sigset.submit_wait([&](int signo) {
            // If the signal is raised again, terminate immediately.
            sigset.clear();
            // Tell the master to start de deconfiguration process for node 1, and
            // submit a task to be executed once that process completes.
            master->AsyncDeconfig(1).submit(exec, [&]() {
                // Perform a clean shutdown.
                std::cout << std::endl << "LIVELY 1111111!" << std::endl;
                ctx.shutdown();
                std::cout << std::endl << "LIVELY 2222222!" << std::endl;
            });
        });

        //Start the NMT service of the master by pretending to receive a 'reset
        // node' command.
        std::cout << "BEFORE MASTER RESET" << std::endl;
        master->Reset();
        std::cout << "AFTER MASTER RESET" << std::endl;

        master->Command(lely::canopen::NmtCommand::RESET_NODE);
        master->Command(lely::canopen::NmtCommand::RESET_COMM);
        master->Command(lely::canopen::NmtCommand::STOP);
        master->Command(lely::canopen::NmtCommand::ENTER_PREOP);

        // making TPDO 1 synchronous
        master->Write<uint8_t>(0x1800, 2, 0x01);
        // making TPDO 2 synchronous
        master->Write<uint8_t>(0x1801, 2, 0x01);
        // making TPDO 3 synchronous
        master->Write<uint8_t>(0x1802, 2, 0x01);
        // making TPDO 4 synchronous
        //master.Write<uint8_t>(0x1803, 2, 0x01);

        time.last_us = getCurrentEpochMicroseconds();
        time.sum_total_dt = 0;

        std::cout << "end of ManoplaLelyBBB Ctor!" << std::endl;
    }

    void ManoplaLelyBBB::installOnSyncCallback(const SyncCallback& onSyncCallback) {
        driver->setOnSyncCallback(onSyncCallback);
    }

    void ManoplaLelyBBB::start_loop() {
        std::cout << "Start of loop.run()\n";
        loop.run();
        std::cout << "End of loop.run()\n";
    }

    void ManoplaLelyBBB::stop() {
        master->AsyncDeconfig(1).submit(exec, [&]() {
            // Perform a clean shutdown.
            std::cout << std::endl << "[stop()] LIVELY 1111111!" << std::endl;
            ctx.shutdown();
            std::cout << std::endl << "[stop()] LIVELY 2222222!" << std::endl;
        });
    }

    manopla::MyDriver::MyDriver(ev_exec_t *exec, lely::canopen::AsyncMaster &master, uint8_t id,
                                manopla::MotionMode motionMode)
            : lely::canopen::FiberDriver(exec, master, id), motionMode(motionMode), onSyncCallback(nullptr) {
    }

    //This function gets called when the boot-up process of the slave completes.
    // The 'st' parameter contains the last known NMT state of the slave
    // (typically pre-operation), 'es' the error code (0 on success), and 'what'
    // a description of the error, if any.
    void
    MyDriver::OnBoot(NmtState /*st*/, char es,
           const std::string &what) noexcept {
        if (!es || es == 'L') {
            std::cout << "slave " << static_cast<int>(id()) << " booted successfully"
                      << std::endl;
        } else {
            std::cout << "slave " << static_cast<int>(id())
                      << " failed on boot: " << what << std::endl;
        }
    }

    // This function gets called during the boot-up process for the slave. The
    // 'res' parameter is the function that MUST be invoked when the configuration
    // is complete. Because this function runs as a task inside a coroutine, it
    // can suspend itself and wait for an asynchronous function such as an SDO
    // request, to complete.
    void
    MyDriver::OnConfig(std::function<void(std::error_code ec)> res) noexcept {
        std::cout << "\n\nSTARTED MyDriver::OnConfig\n\n" << std::endl;
        try {
            // Perform a few SDO write requests to configure the slave. The
            // AsyncWrite() function returns a future which becomes ready once the
            // request completes, and the Wait()     function suspends the coroutine for
            // this task until the future is ready

            // Configure the slave to monitor the heartbeat of the master (node-ID 2)
            // with a timeout of 2000 ms. about 2 << 16 : 2 is the node-ID
            //Wait(AsyncWrite<uint32_t>(0x1016, 1, (2 << 16) | 2000));
            // Configure the slave to produce a heartbeat every 1000 ms.
            //Wait(AsyncWrite<uint16_t>(0x1017, 0, 2000));
            // Configure the heartbeat consumer on the master.
            //ConfigHeartbeat(1900ms);

//            // set CAN bitrate (baud rate) to 250kbps
//            Wait(AsyncWrite<uint16_t>(0x2001, 0, 3));
//            // save all parameters
//            Wait(AsyncWrite<uint32_t>(0x1010, 1, 0x73617665));

            // set inhibit time to 10 * 100us for PDO1
            Wait(AsyncWrite<uint16_t>(0x1800, 3, 10));
            // set inhibit time to 10 * 100us for PDO2
            Wait(AsyncWrite<uint16_t>(0x1801, 3, 10));
            // set inhibit time to 10 * 100us for PDO3
            Wait(AsyncWrite<uint16_t>(0x1802, 3, 10));
            // set inhibit time to 10 * 100us for PDO4
            Wait(AsyncWrite<uint16_t>(0x1803, 3, 10));

            // set transmission type to 1 (each 1 SYNC) for PDO1
            Wait(AsyncWrite<uint8_t>(0x1800, 2, 1));
            // set transmission type to 1 (each 1 SYNC) for PDO2
            Wait(AsyncWrite<uint8_t>(0x1801, 2, 1));
            // set transmission type to 1 (each 1 SYNC) for PDO3
            Wait(AsyncWrite<uint8_t>(0x1802, 2, 1));
            // set transmission type to 1 (each 1 SYNC) for PDO4
            Wait(AsyncWrite<uint8_t>(0x1803, 2, 1));

            //SetProfileVelocityMode();
            //SetProfilePositionMode();
            //SetPositionMode();
            //SetCurrentMode();
            // Report success (empty error code).

            switch (motionMode) {
                case MotionMode::ProfilePositionMode:
                    SetProfilePositionMode();
                    break;
                    //case MotionMode::HomingMode, //Not Implemented
                case MotionMode::ProfileVelocityMode:
                    SetProfileVelocityMode();
                    break;
                case MotionMode::PositionMode:
                    std::cout << "MyDriver::OnConfig -> PositionMode\n";
                    this->SetPositionMode();
                    break;
                    //case MotionMode::VelocityMode:
                case MotionMode::CurrentMode:
                    SetCurrentMode();
                    break;
                default:
                    std::cout << "setting Position Mode motion!" << std::endl;
                    SetPositionMode();
                    break;
            }

            std::cout << "Supported drive modes: " << std::bitset<32>(static_cast<int>(Wait(AsyncRead<uint32_t>(0x6502, 0)))) << std::endl;

            std::cout << "\n\nGOOD end of OnConfig\n\n";
            res({});
        } catch (SdoError &e) {
            std::cout << "\n\nBAD end of OnConfig\n\n";
            // If one of the SDO requests resulted in an error, abort the
            // configuration and report the error code.
            res(e.code());
        }
    }

    void MyDriver::SetProfilePositionMode(uint32_t maxFollowingError, /* qc 0x6065.00 */
                                int32_t minPositionLimit, /* qc 0x607D.01 */
                                int32_t maxPositionLimit, /* qc 0x607D.02 */
                                uint32_t maxProfileVelocity, /* rpm 0x607F.00 */
                                uint32_t profileVelocity, /* rpm 0x6081.00 */
                                uint32_t profileAcceleration, /* rpm/s 0x6083.00 */
                                uint32_t profileDeceleration, /* rpm/s 0x6084.00 */
                                uint32_t quickstopDeceleration, /* rpm/s 0x6085.00 */
                                int16_t motionProfileType /* 0 for linear or 1 for sin² 0x6086.00 */) {

        master.Command(NmtCommand::STOP, id());
        master.Command(NmtCommand::ENTER_PREOP, id());

        // set profile position operation mode
        Wait(AsyncWrite<uint8_t>(0x6060, 0, 0x01));

        // set max following error
        Wait(AsyncWrite<uint32_t>(0x6065, 0, reinterpret_cast<uint32_t &&>(maxFollowingError)));
        // set min position limit
        Wait(AsyncWrite<int32_t>(0x607D, 1, reinterpret_cast<int32_t &&>(minPositionLimit)));
        // set max position limit
        Wait(AsyncWrite<int32_t>(0x607D, 2, reinterpret_cast<int32_t &&>(maxPositionLimit)));

        // set max profile velocity
        Wait(AsyncWrite<uint32_t>(0x607F, 0, reinterpret_cast<uint32_t &&>(maxProfileVelocity)));

        // set profile velocity
        Wait(AsyncWrite<uint32_t>(0x6081, 0, reinterpret_cast<uint32_t &&>(profileVelocity)));

        // set profile acceleration
        Wait(AsyncWrite<uint32_t>(0x6083, 0, reinterpret_cast<uint32_t &&>(profileAcceleration)));

        // set profile deceleration
        Wait(AsyncWrite<uint32_t>(0x6084, 0, reinterpret_cast<uint32_t &&>(profileDeceleration)));

        // set quick stop deceleration
        Wait(AsyncWrite<uint32_t>(0x6085, 0, reinterpret_cast<uint32_t &&>(quickstopDeceleration)));

        // set motion profile type
        Wait(AsyncWrite<int16_t>(0x6086, 0, reinterpret_cast<int16_t &&>(motionProfileType)));

        Shutdown();
        SwitchOn();

        master.Command(NmtCommand::START, id());

        // set target position
        Wait(AsyncWrite<int32_t>(0x607A, 0, 0));
        // set Controlword to start absolute positioning immediately
        StartAbsPositioningImmediate();
    }


    void MyDriver::SetCurrentMode(uint16_t continuousCurrentLimit, /* mA 0x6410.01 */
                        uint16_t maxSpeed, /* rpm 0x6410.04 */
                        uint16_t thermalTimeConstantWinding /* ? 0x6410.05 */
    ) {

        master.Command(NmtCommand::STOP, id());
        master.Command(NmtCommand::ENTER_PREOP, id());

        // set profile position operation mode
        Wait(AsyncWrite<int8_t>(0x6060, 0, 0xFD));


        Wait(AsyncWrite<uint16_t>(0x6410, 1, reinterpret_cast<uint16_t &&>(continuousCurrentLimit)));

        Wait(AsyncWrite<uint16_t>(0x6410, 4, reinterpret_cast<uint16_t &&>(maxSpeed)));

        Wait(AsyncWrite<uint16_t>(0x6410, 5, reinterpret_cast<uint16_t &&>(thermalTimeConstantWinding)));

        Shutdown();
        SwitchOn();

        master.Command(NmtCommand::START, id());

        // set target position
        //Wait(AsyncWrite<int16_t>(0x2030, 0, 0));
    }

    void MyDriver::SetPositionMode(uint32_t maxFollowingError, /* qc 0x6065.00 */
                         int32_t minPositionLimit, /* qc 0x607D.01 */
                         int32_t maxPositionLimit /* qc 0x607D.02 */) {
        std::cout << "\tSTART OF MyDriver::SetPositionMode function" << std::endl;
        master.Command(NmtCommand::STOP, id());
        master.Command(NmtCommand::ENTER_PREOP, id());

        std::cout << "\tset position mode (write 0xFF to 0x6060.00)" << std::endl;
        Wait(AsyncWrite<uint8_t>(0x6060, 0, 0xFF));

        std::cout << "\tset max following error (write value to 0x6065.00)" << std::endl;
        Wait(AsyncWrite<uint32_t>(0x6065, 0, reinterpret_cast<uint32_t &&>(maxFollowingError)));

        std::cout << "\tset min position limit (write value to 0x607D.01)" << std::endl;
        Wait(AsyncWrite<int32_t>(0x607D, 1, reinterpret_cast<int32_t &&>(minPositionLimit)));

        std::cout << "\tset max position limit (write value to 0x607D.02)" << std::endl;
        Wait(AsyncWrite<int32_t>(0x607D, 2, reinterpret_cast<int32_t &&>(maxPositionLimit)));

        std::cout << "Operation Mode      (0x6060.00): " << static_cast<int>(Wait(AsyncRead< uint8_t>(0x6060, 0))) << "\n"
                  << "Max Following Error (0x6065.00): " << static_cast<int>(Wait(AsyncRead<uint32_t>(0x6065, 0))) << "\n"
                  << "Min Position Limit  (0x607D.01): " << static_cast<int>(Wait(AsyncRead<uint32_t>(0x607D, 1))) << "\n"
                  << "Max Position Limit  (0x607D.02): " << static_cast<int>(Wait(AsyncRead<uint32_t>(0x607D, 2))) << "\n";

        std::cout << "\tShutdown()" << std::endl;
        Shutdown();
        std::cout << "\tSwitchOn()" << std::endl;
        SwitchOn();


        // set target position
        //std::cout << "\tset postion Mode Setting Value (write value to 0x2062.00)" << std::endl;
        //Wait(AsyncWrite<int32_t>(0x2062, 0, 0));
        master.Command(NmtCommand::START, id());
    }

    void MyDriver::SetProfileVelocityMode(uint32_t maxProfileVelocity, /* rpm */
                                uint32_t profileAcceleration, /* rpm/s */
                                uint32_t profileDeceleration, /* rpm/s */
                                uint32_t quickStopDeceleration, /* rpm/s */
                                uint8_t motionProfileType /* 0: linear, 1: sin^2 */) {

        master.Command(NmtCommand::STOP, id());
        master.Command(NmtCommand::ENTER_PREOP, id());

        // set profile velocity operation mode
        Wait(AsyncWrite<uint8_t>(0x6060, 0, 0x03));

        // set max profile  veloocity
        Wait(AsyncWrite<uint32_t>(0x607F, 0, reinterpret_cast<unsigned int &&>(maxProfileVelocity)));
        // set profile acceleration
        Wait(AsyncWrite<uint32_t>(0x6083, 0, reinterpret_cast<unsigned int &&>(profileAcceleration)));
        // set profile deceleration
        Wait(AsyncWrite<uint32_t>(0x6084, 0, reinterpret_cast<unsigned int &&>(profileDeceleration)));
        // set quick stop deceleration
        Wait(AsyncWrite<uint32_t>(0x6085, 0, reinterpret_cast<unsigned int &&>(quickStopDeceleration)));
        // set motion profile
        Wait(AsyncWrite<uint16_t>(0x6086, 0, reinterpret_cast<unsigned int &&>(motionProfileType)));

        Shutdown();
        SwitchOn();

        master.Command(NmtCommand::START, id());

        // set target velocity
        Wait(AsyncWrite<int32_t>(0x60FF, 0, 0));
        // set Controlword
        SwitchOn();
    }

    /* Shutdown by writing 0x0006 (uint16_t) to 0x6040.00*/
    void MyDriver::Shutdown() {
        Wait(AsyncWrite<uint16_t>(0x6040, 0, 0x0006));
    }

    /* SwitchOn by writing 0x000F (uint16_t) to 0x6040.00*/
    void MyDriver::SwitchOn() {
        Wait(AsyncWrite<uint16_t>(0x6040, 0, 0x000F));
    }

    /* Start Absolute Positioning Immediately by writing 0x003F (uint16_t) to 0x6040.00*/
    void MyDriver::StartAbsPositioningImmediate() {
        Wait(AsyncWrite<uint16_t>(0x6040, 0, 0x003F));
    }

    uint64_t good_print_period_ms = 300;
    uint64_t dt_sum_for_printing_ms = 0;

    void MyDriver::OnSync(uint8_t cnt, const time_point &t) noexcept {
        time.current_us = getCurrentEpochMicroseconds();
        time.current_dt_us = time.current_us - time.last_us;
        time.last_us = time.current_us;
        time.max_dt_us = std::max(time.max_dt_us, time.current_dt_us);
        if (time.dt_count > 0) [[likely]] {
            time.min_dt_us = std::min(time.min_dt_us, time.current_dt_us);
            time.sum_total_dt += time.current_dt_us;
        }
        time.dt_count++;

        motorInfo.prevPulses = motorInfo.currentPulses;
        motorInfo.currentPulses = static_cast<int32_t>(rpdo_mapped[0x6064][0]);
        motorInfo.prevAngle = motorInfo.currentAngle;
        motorInfo.currentAngle =
                (static_cast<float>(motorInfo.currentPulses) * 2.f * M_PIf32 / pulsesPerTurn) / gear;
        /*dt_sum_for_printing_ms += time.current_dt_us;
        if(dt_sum_for_printing_ms > good_print_period_ms){
            using std::setw;
            dt_sum_for_printing_ms = 0ull;
            std::cout
                    << "currentAngle: " << setw(10) << motorInfo.currentAngle
                    << " currentPulses: " << setw(10) << motorInfo.currentPulses
                    << " M_PIf32: " << setw(10) << M_PIf32
                    << " pulsesPerTurn: " << setw(10) << pulsesPerTurn
                    << " gear: " << setw(10) << gear
                    << '\n';
        }*/
        motorInfo.prevRotationUnfiltered = motorInfo.currentRotationUnfiltered;
        motorInfo.currentRotationUnfiltered = static_cast<int32_t>(rpdo_mapped[0x606c][0]);
        motorInfo.prevCurrent = motorInfo.currentCurrent;
        motorInfo.currentCurrent = static_cast<int16_t>(rpdo_mapped[0x6078][0]);
        if (onSyncCallback != nullptr) {
            onSyncCallback(time, motorInfo, *this);
        }
    }

    // This function is similar to OnConfig(), but it gets called by the
    // AsyncDeconfig() method of the master.
    void
    MyDriver::OnDeconfig(std::function<void(std::error_code ec)> res) noexcept {
        try {

            master.Command(NmtCommand::ENTER_PREOP);

            std::cout << "Fiber Deconfig\n";
            //Disable the heartbeat consumer on the master.
            ConfigHeartbeat(0ms);
            // Disable the heartbeat producer on the slave.
            Wait(AsyncWrite<uint16_t>(0x1017, 0, 0));
            // Disable the heartbeat consumer on the slave.
            Wait(AsyncWrite<uint32_t>(0x1016, 1, 0));

            master.Command(NmtCommand::RESET_NODE);

            std::cout << "max_dt_ms: " << time.max_dt_us << "\tmin_dt_ms: " << time.min_dt_us << "\tmean_dt_ms: "
                      << static_cast<float>(time.sum_total_dt) / static_cast<float>(time.dt_count) << std::endl;

            // shutdown
            Shutdown();

            res({});
        } catch (SdoError &e) {
            res(e.code());
        }

        std::cout << "End of OnDeconfig\n";
    }

    void MyDriver::setOnSyncCallback(const SyncCallback& onSyncCallback) {
        this->onSyncCallback = onSyncCallback;
    }
}