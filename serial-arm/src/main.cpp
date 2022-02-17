#include "../include/SerialArm.hpp"
#include <thread>
#include <sched.h>
#include <pthread.h>

int main()
{
    constexpr unsigned num_threads = 3;
    const int64_t execution_time = 2000; //ms, i.e. 2 sec

    // A mutex ensures orderly access to std::cout from multiple threads.
    std::mutex thread_mutex;
    std::vector<std::thread> threads(num_threads);

    //Initialize class object
    SerialArm robot;

    while((robot.get_counter()*10) <=  execution_time)
    {
        for (unsigned i = 0; i < num_threads; ++i) 
        {
            threads[i] = std::thread([&thread_mutex, i, &robot] {

                std::lock_guard<std::mutex> guard(thread_mutex);
                std::cout << "Thread #" << i << ": on CPU " << sched_getcpu() << "\n";

                //Main function
                robot.generate_trajectory(i);

            });

            // Create a cpu_set_t object representing a set of CPUs. Clear it and mark
            cpu_set_t cpuset;
            CPU_ZERO(&cpuset);

            //bind 2 threads to run on same CPU
            if (i == 1)
                CPU_SET(0, &cpuset);
            else
                CPU_SET(i, &cpuset);

            int rc = pthread_setaffinity_np(threads[i].native_handle(), sizeof(cpu_set_t), &cpuset);
            if (rc != 0) 
                std::cerr << "Error calling pthread_setaffinity_np: " << rc << "\n";
        }

        for (auto& t : threads) 
        {
            t.join();
        }
    }

    return 0;
}