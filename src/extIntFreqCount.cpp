/* FreqCount Library, for measuring frequencies
 * http://www.pjrc.com/teensy/td_libs_FreqCount.html
 * Copyright (c) 2014 PJRC.COM, LLC - Paul Stoffregen <paul@pjrc.com>
 *
 * Version 1.1
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "extIntFreqCount.h"
#include "util/FreqCountTimers.h"


void ExtIntFreqCountClass::begin(int checkPin, int pulseDir)
{
  counter_init();
  attachInterrupt(checkPin, timer_callback, pulseDir);

}

uint8_t ExtIntFreqCountClass::available(void)
{ 
	return count_ready;
}

uint32_t ExtIntFreqCountClass::read(void)
{
  count_ready = 0;
  return count_output;
}

void ExtIntFreqCountClass::end(void)
{
	timer_shutdown();
}

ExtIntFreqCountClass ExtIntFreqCount;


