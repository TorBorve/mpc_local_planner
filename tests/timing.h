#pragma once
#include <chrono>
#include <iostream>

namespace mpc
{

    class Timer
    {
    public:
        Timer(const std::string &text)
        {
            start_ = std::chrono::high_resolution_clock::now();
            text_ = text;
        }
        void printTime() const
        {
            auto end = std::chrono::high_resolution_clock::now();
            auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_);
            std::cout << text_ << ": " << dur.count() << "[ms]\n";
        }

        void stop()
        {
            if (!stoped_)
            {
                stoped_ = true;
                printTime();
            }
        }
        ~Timer()
        {
            if (!stoped_)
            {
                printTime();
            }
        }

    private:
        std::chrono::_V2::system_clock::time_point start_;
        std::string text_;
        bool stoped_ = false;
    };

}