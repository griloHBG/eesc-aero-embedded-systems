// Header para modularização inicial, separando a extensa main da também
// extensa fase de declarações, definições, entre outros.

#ifndef CONFIG_DEF_HEADER_H
#define CONFIG_DEF_HEADER_H

#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
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

// Definição de constantes para o argumento da interface CAN e de versões/sync
constexpr unsigned int ARG_CAN_INTERFACE = 1;
constexpr unsigned int ARG_VERSION = 1;
constexpr unsigned int ARG_SYNCCALLBACK = 2; // position-mode, current-mode, pid, dlqr, dlqr-event

constexpr unsigned int ARG_SYNCCALLBACK_PID_KP = 3; // Kp for PID
constexpr unsigned int ARG_SYNCCALLBACK_PID_KD = 4; // Kd for PID
constexpr unsigned int ARG_SYNCCALLBACK_PID_KI = 5; // Ki for PID

constexpr unsigned int ARG_SYNCCALLBACK_DLQR_TYPE = 3; // constant-zero, sine-wave

constexpr unsigned int ARG_SYNCCALLBACK_DLQREVENT_TYPE = 3; // constant-zero, sine-wave
constexpr unsigned int ARG_SYNCCALLBACK_DLQREVENT_EVENTPRESET = 4; //minimum-update-rate, minimum-comparison-metric

// Definição de valores padrão para constantes do PID
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

// Definição de novas matrizes para o sistema (consideradas melhores)
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

#endif // CONFIG_DEF_HEADER_H