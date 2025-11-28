#include <mutex>
#include <condition_variable>


class Semaphore {
public:
    Semaphore(int count = 0);

    void acquire();

    void release();
private:
    int count;
    std::mutex mutex;
    std::condition_variable cv;
};