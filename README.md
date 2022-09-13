# Guitar Tuner 🎸🎵
This project was created by [@CiscoEdoardo](https://github.com/EdoardoCisco) and [@CollizzolliLeonardo](https://github.com/leocolliz) for the exam of the  Embedded Software for the Internet of Things Unitn course 2021/2022. Using a microcontroller we made a simple device that you can use to tune a guitar or as a metronome.

## Hardware 🛠️
**MSP-EXP432P401R**<br/>
**BOOSTXL-EDUMKII**<br/>

## YouTube demonstration 🎥
https://youtu.be/59A3gClLraM
## Libraries 📚
- <br/> Grlib
- <br/> CristalFontz
- <br/> Arm math

## Software Description 💻
Here you have the code we wrote to make a guitar tuner using an MSP432, the idea behind the project is to use the integrated microphone in boostXL to sample fequencies, and the buzzer to make a simple methronome. The sampling of frequency is made with Fourier transform that takes volt values from microphone to frequency domain. All values taken from microphone are saved using dma to transfer data, then the launchpad starts to compute data while the microphone still saves data; when the launchpad has computed data we display on the screen of the booster pack the guitar string we suppose is being tuned and with a vertical bar the distance from the right tone. (-,+)<br/>
When metrononme is selected from main on the screen appears a number that can be changed rising or reducing each digit. The number represents how many click are played in a minute, the unit mesure used is beats per minute (BPM).
