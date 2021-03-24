
/*
    PiMoCo: Raspberry Pi Telescope Mount and Focuser Control
    Copyright (C) 2021 Markus Noga

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/


#ifndef PIMOCO_TIME_H
#define PIMOCO_TIME_H

#include <sys/time.h>
#include <stdint.h>


// A timestamp, queryable in milliseconds and microseconds
class Timestamp {
public:
    // Creates a new timestamp and updates it with the current system time
    Timestamp() { 
      update();
    }

    // Updates timestamp with the current system time
    void update() {
        gettimeofday(&tv, 0);
    }

    // Returns microseconds
    uint64_t us() const {
        return tv.tv_sec * (uint64_t) 1000000 + tv.tv_usec; 
    } 

    // Returns milliseconds
    uint64_t ms() const {
        return tv.tv_sec * (uint64_t) 1000 + tv.tv_usec / 1000; 
    } 

    // Returns milliseconds elapsed since the prior time t
    uint64_t msSince(const Timestamp &t) const {
        return ms() - t.ms();
    }

    // Returns microseconds elapsed since the prior time t
    uint64_t usSince(const Timestamp &t) const {
        return us() - t.us();
    }

protected:
    struct timeval tv;   
};

#endif // PIMOCO_TIME_H