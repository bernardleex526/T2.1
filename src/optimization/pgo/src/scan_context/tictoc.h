// VENDORED THIRD-PARTY FILE -- upstream https://github.com/gisbi-kim/SC-LIO-SAM
//   commit d43ca00d97a756303c10975e32e8d66bfabb337d, path SC-LIO-SAM/include/tictoc.h
//   licence: Tong Qin / Shaozu Cao, VINS-Mono (GPLv3) -- stopwatch helper, output disabled by default
//   sha256 677619225488925b0932a066427921c8e02bc1c47a4fae2085701b273ffe60a1 (as vendored, header line below added by the fork)
//   local modifications: none.
// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#pragma once

#include <ctime>
#include <iostream>
#include <string>
#include <cstdlib>
#include <chrono>

class TicToc
{
public:
    TicToc()
    {
        tic();
    }

    TicToc( bool _disp )
    {
        disp_ = _disp;
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    void toc( std::string _about_task )
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        double elapsed_ms = elapsed_seconds.count() * 1000;

        if( disp_ )
        {
          std::cout.precision(3); // 10 for sec, 3 for ms 
          std::cout << _about_task << ": " << elapsed_ms << " msec." << std::endl;
        }
    }

private:  
    std::chrono::time_point<std::chrono::system_clock> start, end;
    bool disp_ = false;
};
