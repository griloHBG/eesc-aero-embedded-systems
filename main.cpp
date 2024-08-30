#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include "ManoplaLelyBBB.h"
#include <armadillo>
#include <tuple>
#include <vector>
#include <string>
#include <sstream>
#include <chrono>

using Clock=std::chrono::high_resolution_clock;

double event(double k, double a=16, double b=0.01){
    return a*std::pow((1-b), k);
}

double super_event(double x_norm, double matrix_norm, double k, double a=30, double b=0.001, double sigma=0.01) {
   return sigma*x_norm/matrix_norm + event(k,a,b);
}

constexpr unsigned int ARG_CAN_INTERFACE = 1;
constexpr unsigned int ARG_VERSION = 1;
constexpr unsigned int ARG_SYNCCALLBACK = 2; // position-mode, current-mode, pid, dlqr, dlqr-event

constexpr unsigned int ARG_SYNCCALLBACK_PID_KP = 3; // Kp for PID
constexpr unsigned int ARG_SYNCCALLBACK_PID_KD = 4; // Kd for PID
constexpr unsigned int ARG_SYNCCALLBACK_PID_KI = 5; // Ki for PID

constexpr unsigned int ARG_SYNCCALLBACK_DLQR_TYPE = 3; // constant-zero, sine-wave

constexpr unsigned int ARG_SYNCCALLBACK_DLQREVENT_TYPE = 3; // constant-zero, sine-wave
constexpr unsigned int ARG_SYNCCALLBACK_DLQREVENT_EVENTPRESET = 4; //minimum-update-rate, minimum-comparison-metric

constexpr float DEFAULT_KP = 1800.f;
constexpr float DEFAULT_KD = 150.f;
constexpr float DEFAULT_KI = 0.f;

// ConstantZero or SineWave
enum class ReferenceType{
    ConstantZero,
    SineWave,
};

#define CONSTANTZERO    "constant-zero"
#define SINEWAVE        "sine-wave"

std::ostream& operator<<(std::ostream& o, ReferenceType rt) {
    if(rt == ReferenceType::ConstantZero) {
        o << CONSTANTZERO;
    } else if (rt == ReferenceType::SineWave) {
        o << SINEWAVE;
    }
    return o;
}

// ConstantZero or SineWave
std::map <std::string, ReferenceType> ReferenceTypeString {
    { CONSTANTZERO, ReferenceType::ConstantZero   },
    { SINEWAVE    , ReferenceType::SineWave       },
};

const ReferenceType DEFAULT_REFERENCETYPE = ReferenceType::ConstantZero;


/* first dd (i=0, N=190) */
/*
arma::mat sys_matrix_A = {{ 2.82975412e-15 , 1.00000000e+00 },
                          {-9.80269531e-01 , 1.97873008e+00 }};

arma::colvec sys_matrix_B = { 1.84982170e-15 , 6.72172757e-03 };

arma::rowvec sys_dlqr_K = { 15.12857803, -16.25584753 };


double matrix_norm{0.1492657117991687};
*/


/* best dd (i=3300, N=100) (least amount of error)*/

arma::mat sys_matrix_A = {{  9.98609061e-01,  1.09025360e+01 },
                          { -1.13123668e-04,  9.77691957e-01 }};

arma::colvec sys_matrix_B = { 8.6104e-3, 8.0799e-4 };

arma::rowvec sys_dlqr_K = { -2.70079861, -233.77001218 };

struct EventDLQRControlPreset {
    arma::rowvec K;
    double a;
    double b;
    double sigma;
    ReferenceType ct;
};

enum class ControllerPreset{
            //  reference   q   a   b       sigma   K                   metrica_comparacao  update_rate [%]
    DR6,    //  constant    1e5 -   -       -       -89.9178, -1155.41  0.726244            -
    EDR24,  //  constant    5   16  1e-3    0.01    -1.90978, -194.491  29666.6             0.966667
    EDR83,  //  constant    1e3 10  0.2     0.01    -21.6764, -644.052  3607.52             29.1667
    DC6,    //  sin         1e5 -   -       -       -89.9178, -1155.41  311.2               -
    EDC56,  //  sin         10  30  1e-3    0.01    -2.7008,  -233.77   436463              16.1
    EDC103, //  sin         1e4 10  .2      0.01    -50.5014, -924      19347.1             48.4
};

std::ostream& operator<<(std::ostream& o, ControllerPreset cp) {
    switch (cp) {
        case ControllerPreset::DR6:
            o << "DR6"; break;
        case ControllerPreset::EDR24:
            o << "EDR24"; break;
        case ControllerPreset::EDR83:
            o << "EDR83"; break;
        case ControllerPreset::DC6:
            o << "DC6"; break;
        case ControllerPreset::EDC56:
            o << "EDC56"; break;
        case ControllerPreset::EDC103:
            o << "EDC103"; break;
    }

    return o;
}

// DR6, EDR24, EDR83, DC6, EDC56, EDC103
std::map<std::string, ControllerPreset> ControllerPresetString{
        { "DR6"    , ControllerPreset::DR6    },
        { "EDR24"  , ControllerPreset::EDR24  },
        { "EDR83"  , ControllerPreset::EDR83  },
        { "DC6"    , ControllerPreset::DC6    },
        { "EDC56"  , ControllerPreset::EDC56  },
        { "EDC103" , ControllerPreset::EDC103 }};

std::map<ControllerPreset, EventDLQRControlPreset> DLQRControlPresets{
    {   ControllerPreset::DR6,     EventDLQRControlPreset{ .K={-89.9178,    -1155.41}, .a=0,    .b=0,   .sigma=0,       .ct=ReferenceType::ConstantZero }},
    {   ControllerPreset::EDR24,   EventDLQRControlPreset{ .K={-1.90978,    -194.491}, .a=16,   .b=1e-3,.sigma=0.01,    .ct=ReferenceType::ConstantZero }},
    {   ControllerPreset::EDR83,   EventDLQRControlPreset{ .K={-21.6764,    -644.052}, .a=10,   .b=.2,  .sigma=0.01,    .ct=ReferenceType::ConstantZero }},
    {   ControllerPreset::DC6,     EventDLQRControlPreset{ .K={-89.9178,    -1155.41}, .a=0,    .b=0,   .sigma=0,       .ct=ReferenceType::SineWave     }},
    {   ControllerPreset::EDC56,   EventDLQRControlPreset{ .K={-2.7008,     -233.77 }, .a=30,   .b=1e-3,.sigma=0.01,    .ct=ReferenceType::SineWave     }},
    {   ControllerPreset::EDC103,  EventDLQRControlPreset{ .K={-50.5014,    -924    }, .a=10,   .b=.2,  .sigma=0.01,    .ct=ReferenceType::SineWave     }},
};

bool controllerPresetSelected{false};

double matrix_norm{};


/*
 * tuple DataPoint:
 *   uint64_t time_us,
 *   int32_t pulse_qc,
 *   double setpoint_current_mA,
 *   int16_t actual_current_mA,
 *   int32_t epos_velocity_unfiltered_rpm,
 *   float calculated_velocity_rad/s,
 *   float tracked_reference,
 *   float event_max_error,
 *   float event_error
*/
using DataPoint = std::tuple<uint64_t /*time_us*/,
                             int32_t /*pulse_qc*/,
                             double /*setpoint_current_mA*/,
                             int16_t /*actual_current_mA*/,
                             int32_t /*epos_velocity_unfiltered_rpm*/,
                             float /*calculated_velocity_rad/s*/,
                             float /*tracked_reference*/,
                             float /*event_max_error*/,
                             float /*event_error*/>;

class MyLog {
    std::vector<DataPoint> log;
    std::vector<std::string> logHeader;
public:
    explicit MyLog(const std::vector<std::string>& logHeader): log(), logHeader(logHeader){
    }

    void addDataPoint(const DataPoint & dp) {
        log.push_back(dp);
    }

    void saveToFile(const std::string& fileName,
                    const std::string& controllerType,
                    const ReferenceType& rt,
                    arma::mat A,
                    arma::mat B,
                    arma::mat K,
                    double a,
                    double b,
                    double sigma,
                    float refresh_rate){

        using std::setw;
        using std::setfill;
        std::stringstream ssFileName;
        auto now = Clock::to_time_t(Clock::now());
        auto localtime = std::localtime(&now);

        ssFileName << "log_" << localtime->tm_year + 1900
           << "-" << setfill('0') << setw(2) << localtime->tm_mon+1
           << "-" << setfill('0') << setw(2) << localtime->tm_mday
           << "_" << setfill('0') << setw(2) << localtime->tm_hour
           << "-" << setfill('0') << setw(2) << localtime->tm_min
           << "-" << setfill('0') << setw(2) << localtime->tm_sec
           << "_" << fileName;

        std::ofstream file{ssFileName.str()};

        file.precision(10);

        file << "[Configuration]\n"
                << "controller_type = " << controllerType << "\n"
                << "reference_type = "  << rt << "\n"

                << "A = [ [ " << std::scientific << A[0,0] << "  ,  " << std::scientific << A[0,1] << " ] , [ " << std::scientific << A[1,0] << "  ,  " << std::scientific << A[1,1] << " ] ] \n"
                << "B = [ [ " << std::scientific << B[0]   << "] , [" << std::scientific << B[1] << " ]]\n"
                << "K = [ [ " << std::scientific << K[0]   << "  ,  " << std::scientific << K[1] << " ]]\n"

                << "event_a = "     << std::scientific << a     << "\n"
                << "event_b = "     << std::scientific << b     << "\n"
                << "event_sigma = " << std::scientific << sigma << "\n"

                << "refresh_rate = " << std::scientific << refresh_rate << "\n"
                << "\n"
                << "[Table]\n";

        std::for_each(logHeader.cbegin(), logHeader.cend()-1, [&](const auto &item) {
            file << item << ',';
        });

        file << (logHeader.cend()-1)->data() << '\n';

        //file << "time[us],pulse[qc],current[mA]\n";

        for(const auto& dp : log) {
            file << setw(20) << std::get<0>(dp) << ','
                 << setw(20) << std::get<1>(dp) << ','
                 << setw(20) << std::get<2>(dp) << ','
                 << setw(20) << std::get<3>(dp) << ','
                 << setw(20) << std::get<4>(dp) << ','
                 << setw(20) << std::get<5>(dp) << ','
                 << setw(20) << std::get<6>(dp) << ','
                 << setw(20) << std::get<7>(dp) << ','
                 << setw(20) << std::get<8>(dp) << '\n';
        }
    }
};

int main(int argc, char* argv[]) {
    std::cout << "Hello, World!" << std::endl;

    std::string can_interface_name{};
    std::string synccallback{};
    float kp{0}, ki{0}, kd{0};
    ReferenceType referenceType{};
    std::string controllerArgParameter;

    double event_a{30};
    double event_b{0.001};
    double event_sigma{0.01};

    std::string version{};
    std::stringstream usage_message;
    usage_message << "Usage:\n"
                  << argv[0] << "<canbus-interface-name> <canopen-syncsallback> [reference-type]\n"
                     "  canopen-synccallback: position-mode, current-mode, pid, dlqr, dlqr-event\n"
                     "    position-mode: no arguments\n"
                     "    current-mode: no arguments\n"
                     "    pid:\n"
                     "      no arguments (default: kp=1800, kd=150, ki=0), or\n"
                     "      kp kd ki\n"
                     "    dlqr [controller-type]\n"
                     "      reference-types:\n"
                     "        no arguments (default: constant-zero), or\n"
                     "        constant-zero\n"
                     "        senoidal\n"
                     "        DR6\n"
                     "        DC6\n"
                     "    dlqr-event [controller-type]\n"
                     "      reference-types:\n"
                     "        no arguments (default: constant-zero), or\n"
                     "        constant-zero\n"
                     "        senoidal\n"
                     "        EDR24\n"
                     "        EDR83\n"
                     "        EDC56\n"
                     "        EDC103\n"
                     "\n"
                     "\n"
                     "ADVANCED: DON'T MESS WITH THIS IF YOU DON'T KNOW WHAT YOU ARE DOING!\n"
                     "environment variable:"
                     "    SYS_DLQR_K: defines the gain matrix for the dLQR controller\n"
                     "      default (and stable):\n"
                     "      SYS_DLQR_K=-2.70079861 -233.77001218\n"
                     "    EVENT_A_B_SIGMA: defines the a, b and sigma parameters for triggering the event\n"
                     "                     setting this will override the controller parameters!\n"
                     "      (menor metrica_comparacao - default\n"
                     "         EVNET_A_B_SIGMA=10 0.2 0.01\n"
                     "      (menor update_rate)\n"
                     "         EVNET_A_B_SIGMA=16 0.001 0.01\n"
                     "\n";

    if (argc >= 3) {
        can_interface_name = argv[ARG_CAN_INTERFACE];
        std::cout << "CAN interface choosed: " << can_interface_name << std::endl;

        synccallback = argv[ARG_SYNCCALLBACK];
        std::cout << "SyncCallback choosed: " << synccallback << std::endl;

        if(synccallback == "pid") {
            if (argc < 5 && argc > 3) {
                std::cout << "You must give:\n"
                          << "\t- OR exactly 3 numbers (gains P, D and I) as argument to callback pid choosed!\n"
                          << "\t- OR no arguments to use default values for the gains (P:1800, D:150, I:0)\n";
                std::exit(1);
            }

            if (argc >= 5) {
                kp = std::stof(argv[ARG_SYNCCALLBACK_PID_KP]);
                kd = std::stof(argv[ARG_SYNCCALLBACK_PID_KD]);
                ki = std::stof(argv[ARG_SYNCCALLBACK_PID_KI]);

                if (argc > 5) {
                    std::cout << "\nWarning: remaining arguments will not be used!\n";
                }
            }
            else if (argc == 3){
                std::cout << "Using default values for gains P, D and I!\n";
                kp = DEFAULT_KP;
                kd = DEFAULT_KD;
                ki = DEFAULT_KI;
            }

            std::cout << "Gain P: " << kp << "\n"
                      << "Gain D: " << kd << "\n"
                      << "Gain I: " << ki << "\n";
        } else if(synccallback == "dlqr" || synccallback == "dlqr-event") {
            if (argc == 3){
                referenceType = DEFAULT_REFERENCETYPE;
            } else if (argc >=4) {
                controllerArgParameter = std::string{argv[ARG_SYNCCALLBACK_DLQREVENT_TYPE]};
                if (ReferenceTypeString.find(controllerArgParameter) != ReferenceTypeString.end()) {
                    referenceType = ReferenceTypeString[controllerArgParameter];
                } else {
                    controllerPresetSelected = true;
                    if (synccallback == "dlqr") {
                        if (ControllerPresetString.find(controllerArgParameter) != ControllerPresetString.end()) {
                            if (controllerArgParameter != "DR6" &&
                                controllerArgParameter != "DC6")
                            {
                                std::cout << "ERROR: synccallback " << controllerArgParameter
                                          << " can only be used with DR6 or DC6!\n"
                                          << controllerArgParameter << " is not allowed!\n\n"
                                          << "EXITING!\n\n";
                                std::exit(1);
                            }
                        } else {
                            std::cout << "ERROR: synccallback " << controllerArgParameter
                                      << " can only be used with DR6 or DC6!\n"
                                      << controllerArgParameter << " is non-existent!\n\n"
                                      << "EXITING!\n\n";
                            std::exit(1);
                        }
                    } else if (synccallback == "dlqr-event") {
                        if (ControllerPresetString.find(controllerArgParameter) != ControllerPresetString.end()) {
                            if (controllerArgParameter != "EDR24" &&
                                controllerArgParameter != "EDR83" &&
                                controllerArgParameter != "EDC56" &&
                                controllerArgParameter != "EDC103")
                            {
                                std::cout << "ERROR: synccallback " << controllerArgParameter
                                          << " can only be used with EDR24, EDR83, EDC56 or EDC103!\n"
                                          << controllerArgParameter << " is not allowed!\n\n"
                                          << "EXITING!\n\n";
                                std::exit(1);
                            }
                        } else {
                            std::cout << "ERROR: synccallback " << controllerArgParameter
                                      << " can only be used with EDR24, EDR83, EDC56 or EDC103!\n"
                                      << controllerArgParameter << " is non-existent!\n\n"
                                      << "EXITING!\n\n";
                            std::exit(1);
                        }
                    }
                }
            }
        } else {
            std::cout << "ERROR: CAN'T CONTINUE!\n"
                        << "SYNCCALLBACK " << synccallback << " DOESN'T EXIST!\n"
                        << "\n"
                        << "It must be one of:\n"
                        << "position-mode, current-mode, pid, dlqr or dlqr-event\n"
                        << "\n";
            std::exit(1);
        }
    } else {
        if(argc == 2) {

            version = argv[ARG_VERSION];

            if (version == "--version" || version == "-v") {
                std::cout << "Build number: " << "nada" << "\n";
                std::exit(0);
            }
            else {
                std::cout << usage_message.str();
                std::exit(1);
            }
        }
        std::cout << usage_message.str();
        std::exit(1);
    }

    if (controllerPresetSelected) {
        event_a = DLQRControlPresets[ControllerPresetString[controllerArgParameter]].a;
        event_b = DLQRControlPresets[ControllerPresetString[controllerArgParameter]].b;
        sys_dlqr_K = DLQRControlPresets[ControllerPresetString[controllerArgParameter]].K;
        event_sigma = DLQRControlPresets[ControllerPresetString[controllerArgParameter]].sigma;
        referenceType = DLQRControlPresets[ControllerPresetString[controllerArgParameter]].ct;
    }

    std::stringstream ss;

    if(const char* env_a_b_sigma = std::getenv("EVENT_A_B_SIGMA")) {
        std::cout << "OVERRIDING A, B and SIGMA with values from environment variable EVENT_A_B_SIGMA\n"
                    << "Original values:\n"
                    << "a       = " << event_a << "\n"
                    << "b       = " << event_b << "\n"
                    << "sigma   = " << event_sigma << "\n\n";
        ss = std::stringstream{env_a_b_sigma};
        ss >> event_a;
        ss >> event_b;
        ss >> event_sigma;
        std::cout << "NEW values:\n"
                    << "a       = " << event_a << "\n"
                    << "b       = " << event_b << "\n"
                    << "sigma   = " << event_sigma << "\n\n";
        std::cout << "A, B and SIGMA were overriden by EVENT_A_B_SIGMA environment variable!\n\n";
    } else {
        if (controllerPresetSelected) {
            std::cout << "Setting values from " << controllerArgParameter << " preset for a, b and sigma!\n";
        }
    }

    ss.str("");

    if(const char* env_k = std::getenv("SYS_DLQR_K")) {
        std::cout << "OVERRIDING K controller gains with values from environment variable SYS_DLQR_K\n"
                    << "Original values:\n"
                    << "K[0,0]  = " << sys_dlqr_K[0] << "\n"
                    << "K[0,1]  = " << sys_dlqr_K[1] << "\n\n";
        ss = std::stringstream{env_k};
        ss >> sys_dlqr_K[0];
        ss >> sys_dlqr_K[1];
        std::cout << "NEW values:\n"
                    << "K[0,0]  = " << sys_dlqr_K[0] << "\n"
                    << "K[0,1]  = " << sys_dlqr_K[1] << "\n\n";
        std::cout << "K controller was overriden by SYS_DLQR_K environment variable!\n\n";
    } else {
        if (controllerPresetSelected) {
            std::cout << "Setting default values for K!\n";
        }
    }

    sys_matrix_A.print("matrix A");
    sys_matrix_B.print("matrix B");
    sys_dlqr_K.print("dLQR controller matrix K");

    matrix_norm = arma::norm(sys_matrix_B*sys_dlqr_K, 2);
    std::cout << "norm of B*K: " << matrix_norm << "\n\n";

    std::cout << "control type: " << referenceType << "\n\n";

    std::cout << "Event parameters:\n"
              << "      a = " << event_a     << "\n"
              << "      b = " << event_b     << "\n"
              << "  sigma = " << event_sigma << "\n\n";

    manopla::ManoplaLelyBBB manopla{can_interface_name};
    int dummy_counter = 0;
    int linear_position = 0;
    auto initial_time= std::chrono::duration_cast<std::chrono::microseconds>(Clock::now().time_since_epoch()).count();

    MyLog log{{"time_us", "pulse_qc", "setpoint_current_mA", "actual_current_mA", "epos_velocity_unfiltered_rpm", "calculated_velocity_rad/s", "tracked_reference", "event_max_error", "event_error"}};

    manopla::SyncCallback onSyncCallbackPositionMode = [&](const manopla::Time& t, const manopla::MotorInfo& mi, manopla::MyDriver& driver){
        using std::setw;
        using std::setprecision;
        using std::chrono::duration_cast;
        using std::chrono::microseconds;
        //keep voltage on
        driver.tpdo_mapped[0x6040][0] = static_cast<uint16_t>(0x02);
        if (t.current_dt_us > duration_cast<microseconds>(500ms).count()) {
            std::cout << setw(20) << t.sum_total_dt << setw(20) << t.current_dt_us << setw(20) << mi.currentPulses << '\n';
        }

        //if (dummy_counter == 0 && linear_position < 1000) {
            //driver.tpdo_mapped[0x2062][0] = static_cast<uint32_t>(++linear_position);
        //}
        //A*sin(omega*t+phi)
        auto omega=(8*2*M_PIf32);
        auto phi = 0;
        auto time = t.current_us - initial_time;
        auto A = 500;
        /*std::cout << A << "*sin("<< omega << "*t_us+" << phi << ") "
        << setw(20) << time
        << setw(30) << setprecision(20) << static_cast<double>(time)*omega+phi
        << setw(30) << A*std::sin(static_cast<double>(time)*omega+phi) << '\n';*/
        auto sine_position = A*std::sin(static_cast<double>(time)*omega+phi);

        driver.tpdo_mapped[0x2062][0] = static_cast<int32_t>(sine_position);
        if (time > duration_cast<microseconds>(100ms).count()) {
            std::cout << "sine: " << setw(20) << time << setw(20) << sine_position << "\n";
        }
    };

    uint64_t good_print_period_ms = 300;
    uint64_t dt_sum_for_printing_ms = 0;

    manopla::SyncCallback onSyncCallbackCurrentMode = [&](const manopla::Time& t, const manopla::MotorInfo& mi, manopla::MyDriver& driver) {
        using std::setw;
        using std::setprecision;
        using std::chrono::duration_cast;
        using std::chrono::microseconds;
        using namespace std::chrono_literals;
        //keep voltage on
        driver.tpdo_mapped[0x6040][0] = static_cast<uint16_t>(0x02);
        //std::cout << "pos: " << setw(20) << mi.currentPulses << setw(4) << " curr: " << setw(20) << mi.currentCurrent << '\n';
        //A*sin(omega*t+phi)
        auto omega=(2*M_PIf32);
        auto phi = 0;
        auto time_us = t.current_us - initial_time;
        auto time_s = static_cast<double>(time_us)/1000000.f;
        auto A = 400;
        /*std::cout << A << "*sin("<< omega << "*t_us+" << phi << ") "
        << setw(20) << time
        << setw(30) << setprecision(20) << static_cast<double>(time)*omega+phi
        << setw(30) << A*std::sin(static_cast<double>(time)*omega+phi) << '\n';*/
        auto sine_current_setpoint = A * std::sin(time_s * omega + phi);

        driver.tpdo_mapped[0x2030][0] = static_cast<int16_t>(sine_current_setpoint);
        /*if (time > duration_cast<microseconds>(100ms).count()) {
            std::cout << "sine: " << setw(20) << time << setw(20) << sine_current_setpoint << "\n";
        }*/

        float calculated_speed = (mi.currentAngle - mi.prevAngle)/(static_cast<float>(t.current_dt_us) / 1000000.f);
        dt_sum_for_printing_ms += t.current_dt_us;
        if (dt_sum_for_printing_ms > good_print_period_ms) {
            std::cout << "t.current_us: " << setw(10) << t.current_us
                        << " time_us: " << setw(10) << time_s
                        << " calculated_speed: " << setw(10) << calculated_speed
                        << " currentAngle:" << setw(10) << mi.currentAngle
                        << " prevAngle:" << setw(10) << mi.prevAngle << '\n';
            dt_sum_for_printing_ms = 0ull;
        }

        if(driver.IsReady()) {
            /*
            using DataPoint = std::tuple<uint64_t time_us,
                    int32_t pulse_qc,
                    double setpoint_current_mA,
                    int16_t actual_current_mA,
                    int32_t epos_velocity_unfiltered_rpm,
                    float calculated_velocity_rpm>;
            */
            log.addDataPoint(DataPoint{time_us, mi.currentPulses, sine_current_setpoint, mi.currentCurrent, mi.currentRotationUnfiltered, calculated_speed, 0, 0, 0});
            //'const std::tuple<unsigned long long, int, double, short, int, float>';
        }
    };

    float referencePosition = 0.f;
    float referenceVelocity = 0.f;
    float errorPosition = 0.f;
    float errorPositionSum = 0.f;
    float errorVelocity = 0.f;
    float controlActionP = 0.f;
    float controlActionI = 0.f;
    float controlActionD = 0.f;
    float controlSignal = 0.f;
    float calculatedSpeed = 0.f;

    manopla::SyncCallback onSyncCallbackPID = [&](const manopla::Time& t, const manopla::MotorInfo& mi, manopla::MyDriver& driver) {
        using std::setw;
        using std::setprecision;
        using std::chrono::duration_cast;
        using std::chrono::microseconds;
        using namespace std::chrono_literals;
        //keep voltage on
        driver.tpdo_mapped[0x6040][0] = static_cast<uint16_t>(0x02);
        //std::cout << "pos: " << setw(20) << mi.currentPulses << setw(4) << " curr: " << setw(20) << mi.currentCurrent << '\n';
        //A*sin(omega*t+phi)
        auto time_us = t.current_us - initial_time;
        auto time_s = static_cast<double>(time_us)/1000000.f;

        calculatedSpeed = (mi.currentAngle - mi.prevAngle)/(static_cast<float>(t.current_dt_us) / 1000000.f);

        errorPosition = referencePosition - mi.currentAngle;
        errorVelocity = referenceVelocity - calculatedSpeed;

        errorPositionSum += errorPosition * t.current_dt_us / 1000000.f;

        controlActionP = kp * errorPosition;
        controlActionD = kd * errorVelocity;
        controlActionI = ki * errorPositionSum;

        controlSignal = controlActionP + controlActionD + controlActionI;

        //std::cout << "controlSignal: " << setw(10) << controlSignal << " | 16bit: " << setw(10) << static_cast<int16_t>(controlSignal) <<std::endl;

        driver.tpdo_mapped[0x2030][0] = static_cast<int16_t>(controlSignal);
        /*if (time > duration_cast<microseconds>(100ms).count()) {
            std::cout << "sine: " << setw(20) << time << setw(20) << sine_current_setpoint << "\n";
        }*/

        /*dt_sum_for_printing_ms += t.current_dt_us;
        if (dt_sum_for_printing_ms > good_print_period_ms) {
            std::cout << "t.current_us: " << setw(10) << t.current_us
                        << " time_us: " << setw(10) << time_s
                        << " calculatedSpeed: " << setw(10) << calculatedSpeed
                        << " currentAngle:" << setw(10) << mi.currentAngle
                        << " prevAngle:" << setw(10) << mi.prevAngle << '\n';
            dt_sum_for_printing_ms = 0ull;
        }*/

        if(driver.IsReady()) {
            /*
            using DataPoint = std::tuple<uint64_t time_us,
                    int32_t pulse_qc,
                    double setpoint_current_mA,
                    int16_t actual_current_mA,
                    int32_t epos_velocity_unfiltered_rpm,
                    float calculated_velocity_rpm>;
            */
            log.addDataPoint(DataPoint{time_us, mi.currentPulses, controlSignal, mi.currentCurrent, mi.currentRotationUnfiltered, calculatedSpeed, 0, 0, 0});
            //'const std::tuple<unsigned long long, int, double, short, int, float>';
        }
    };

    arma::colvec x{0, 0};
    arma::mat controlInput;
    arma::colvec error_ref{x};
    arma::colvec x_ref{x};
    auto reference = [&](double time_s)->double{return 400*std::cos(2*M_PI*time_s);};
    double max_error{0};
    double error{0};

    manopla::SyncCallback onSyncCallbackDLQR = [&](const manopla::Time& t, const manopla::MotorInfo& mi, manopla::MyDriver& driver) {
        using std::setw;
        using std::setprecision;
        using std::chrono::duration_cast;
        using std::chrono::microseconds;
        using namespace std::chrono_literals;
        //keep voltage on
        driver.tpdo_mapped[0x6040][0] = static_cast<uint16_t>(0x02);
        //std::cout << "pos: " << setw(20) << mi.currentPulses << setw(4) << " curr: " << setw(20) << mi.currentCurrent << '\n';
        //A*sin(omega*t+phi)
        auto time_us = t.current_us - initial_time;
        auto time_s = static_cast<double>(time_us)/1000000.f;

        calculatedSpeed = (mi.currentAngle - mi.prevAngle)/(static_cast<float>(t.current_dt_us) / 1000000.f);

        x.at(0) = mi.currentPulses;
        x.at(1) = calculatedSpeed;

        if (referenceType == ReferenceType::SineWave) {
            x_ref = {reference(time_s), 0};
            error_ref = - (x_ref - x);
        }

        if (referenceType == ReferenceType::ConstantZero) {
            controlInput = (sys_dlqr_K * x);
        } else if (referenceType == ReferenceType::SineWave) {
            controlInput = (sys_dlqr_K * error_ref);
        }

        controlSignal = controlInput.at(0);

        driver.tpdo_mapped[0x2030][0] = static_cast<int16_t>(controlSignal);
        /*if (time > duration_cast<microseconds>(100ms).count()) {
            std::cout << "sine: " << setw(20) << time << setw(20) << sine_current_setpoint << "\n";
        }*/

        dt_sum_for_printing_ms += t.current_dt_us/1000.f;
        if (dt_sum_for_printing_ms > good_print_period_ms) {
            std::cout << "t.current_us: " << setw(10) << t.current_us
                      //<< " time_s: " << setw(10) << setprecision(2) << time_s
                      //<< " calculatedSpeed: " << setw(10) << calculatedSpeed
                      << " currentAngle:" << setw(10) << mi.currentAngle
                      //<< " currentPulses:" << setw(10) << mi.currentPulses
                      //<< " prevAngle:" << setw(10) << mi.prevAngle
                      << " controlInput: " << setw(10) << controlInput.at(0)
                      << '\n';

            dt_sum_for_printing_ms = 0ull;
        }

        if(driver.IsReady()) {
            DataPoint a;
            if (referenceType == ReferenceType::ConstantZero) {
                log.addDataPoint(DataPoint{time_us, mi.currentPulses, controlSignal, mi.currentCurrent,
                                           mi.currentRotationUnfiltered, calculatedSpeed, 0, 0, 0});
            } else if (referenceType == ReferenceType::SineWave) {
                log.addDataPoint(DataPoint{time_us, mi.currentPulses, controlSignal, mi.currentCurrent,
                                           mi.currentRotationUnfiltered, calculatedSpeed, x_ref.at(0), 0, error_ref.at(0)});
            }
        }

        /*if (time_us > 3e6) {
            manopla.stop();
        }*/
    };

    x = {0, 0};
    arma::colvec x_checkpoint{x};
    double E{0};
    uint refresh_count{0};
    uint time_count{0};
    std::vector<uint64_t> vec_refreshed{};
    controlInput = arma::zeros(1,2) * arma::zeros(2,1);

    manopla::SyncCallback onSyncCallbackDLQREvent = [&](const manopla::Time& t, const manopla::MotorInfo& mi, manopla::MyDriver& driver) {
        using std::setw;
        using std::setprecision;
        using std::chrono::duration_cast;
        using std::chrono::microseconds;
        using namespace std::chrono_literals;
        //keep voltage on
        driver.tpdo_mapped[0x6040][0] = static_cast<uint16_t>(0x02);
        //std::cout << "pos: " << setw(20) << mi.currentPulses << setw(4) << " curr: " << setw(20) << mi.currentCurrent << '\n';
        //A*sin(omega*t+phi)
        auto time_us = t.current_us - initial_time;
        auto time_s = static_cast<double>(time_us)/1000000.f;

        calculatedSpeed = (mi.currentAngle - mi.prevAngle)/(static_cast<float>(t.current_dt_us) / 1000000.f);

        x.at(0) = mi.currentPulses;
        x.at(1) = calculatedSpeed;

        if (referenceType == ReferenceType::SineWave) {
            x_ref = {reference(time_s), 0};
            error_ref = - (x_ref - x);
        }

        if (referenceType == ReferenceType::ConstantZero) {
            max_error = super_event(arma::norm(x, 2), matrix_norm, time_s, event_a, event_b, event_sigma);
        } else {
            max_error = super_event(arma::norm(error_ref, 2), matrix_norm, time_s, event_a, event_b, event_sigma);
        }

        if (error > max_error) {
            if (referenceType == ReferenceType::ConstantZero) {
                controlInput = (sys_dlqr_K * x);
                x_checkpoint = x;
            } else if (referenceType == ReferenceType::SineWave) {
                controlInput = (sys_dlqr_K * error_ref);
                x_checkpoint = error_ref;
            }
            refresh_count++;
            vec_refreshed.push_back(time_us);
        }
        time_count++;

        controlSignal = controlInput.at(0);

        if (referenceType == ReferenceType::ConstantZero) {
            error = arma::norm(x_checkpoint - x, 2);
        } else if (referenceType == ReferenceType::SineWave) {
            error = arma::norm(x_checkpoint - error_ref, 2);
        }

        driver.tpdo_mapped[0x2030][0] = static_cast<int16_t>(controlSignal);
        /*if (time > duration_cast<microseconds>(100ms).count()) {
            std::cout << "sine: " << setw(20) << time << setw(20) << sine_current_setpoint << "\n";
        }*/

        dt_sum_for_printing_ms += t.current_dt_us/1000.f;
        if (dt_sum_for_printing_ms > good_print_period_ms) {
            std::cout << "t.current_us: " << setw(10) << t.current_us
                      << " time_s: " << setw(10) << setprecision(2) << time_s
                      << " calculatedSpeed: " << setw(10) << calculatedSpeed
                      << " currentAngle:" << setw(10) << mi.currentAngle
                      << " currentPulses:" << setw(10) << mi.currentPulses
                      << " prevAngle:" << setw(10) << mi.prevAngle
                      << " error:" << setw(10) << error
                      << " max_error:" << setw(10) << max_error
                      << " controlInput: " << setw(10) << controlInput.at(0) << "(" << controlInput.n_rows << "," << controlInput.n_cols << ")"
                      << " x_ref: " << setw(10) << x_ref << "\n";

            dt_sum_for_printing_ms = 0ull;
        }

        if(driver.IsReady()) {
            DataPoint a;
            if (referenceType == ReferenceType::ConstantZero) {
                log.addDataPoint(DataPoint{time_us, mi.currentPulses, controlSignal, mi.currentCurrent,
                                           mi.currentRotationUnfiltered, calculatedSpeed, 0, max_error, error});
            } else if (referenceType == ReferenceType::SineWave) {
                log.addDataPoint(DataPoint{time_us, mi.currentPulses, controlSignal, mi.currentCurrent,
                                           mi.currentRotationUnfiltered, calculatedSpeed, x_ref.at(0), max_error, error});
            }
        }

        /*if (time_us > 3e6) {
            manopla.stop();
        }*/
    };

    manopla.prepare(250000,
            "master-dcf-motorzao-current-5000.dcf",
            "master-dcf-motorzao-current-5000.bin",
            manopla::MotionMode::CurrentMode);

    ss.str("");

    if (synccallback == "position-mode") {
        manopla.installOnSyncCallback(onSyncCallbackPositionMode);
    }
    else if (synccallback == "current-mode") {

        manopla.installOnSyncCallback(onSyncCallbackCurrentMode);
    }
    else if (synccallback == "pid") {

        manopla.installOnSyncCallback(onSyncCallbackPID);
    }
    else if (synccallback == "dlqr") {

        manopla.installOnSyncCallback(onSyncCallbackDLQR);
    }
    else if (synccallback == "dlqr-event") {

        manopla.installOnSyncCallback(onSyncCallbackDLQREvent);
    }

    /*ss << synccallback;

    if (synccallback == "dlqr" || synccallback == "dlqr-event"){
        ss << "-" << referenceType;
    }

    ss << "_" << "K" << std::scientific << sys_dlqr_K.at(0) << "_" << std::scientific << sys_dlqr_K.at(1);

    if (synccallback == "dlqr-event") {
        //ss << "-";
        //if (eventPreset == EventPreset::MinimumComparisonMetric) {
        //    ssFileName << "minimum-comparison-metric";
        //} else if (eventPreset == EventPreset::MinimumUpdateRate) {
        //    ssFileName << "minimum-update-rate";
        //}
        ss << "_";
        ss << "a" << std::scientific << event_a << "_"
           << "b" << std::scientific << event_b << "_"
           << "s" << std::scientific << event_sigma;
    }*/
    ss << synccallback << "_" << controllerArgParameter << ".csv";

    manopla.start_loop();

    float refresh_rate{};
    if(synccallback == "dlqr-event") {
        refresh_rate = (100.f * refresh_count) / (1.f * time_count);
    } else {
        refresh_rate = 100.f;
    }
    std::cout << "\n\n" << "REFRESH RATE: " << refresh_rate << " (" << refresh_count << " / " << time_count << ")\n\n";

    log.saveToFile(ss.str(), synccallback, referenceType, sys_matrix_A, sys_matrix_B, sys_dlqr_K, event_a, event_b, event_sigma, refresh_rate);

    return 0;
}
