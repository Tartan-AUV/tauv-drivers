#include <chrono>

using namespace std::chrono;

double get_time() {
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}