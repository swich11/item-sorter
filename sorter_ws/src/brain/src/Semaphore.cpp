#include "brain/Semaphore.hpp"


Semaphore::Semaphore(int count) : count{count} {};


void Semaphore::acquire() {
    std::unique_lock<std::mutex> lock(mutex);
    cv.wait(lock, [this] {return this->count > 0;});
    count--;
}


void Semaphore::release() {
    count++;
    cv.notify_one();
}