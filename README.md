# Guitar Tuner 🎸🎵
This project was created by [@CiscoEdoardo](https://github.com/edo98) and [@CollizzolliLeonardo](https://github.com/leocolliz) for an assignment that we are currently completing for the project Embedded Software for the Internet of Things Unitn course 2021/2022. Using a microcontroller we made a simple device that you can use to tune a guitar or use it as a metronome.

## Hardware 🛠️
**MSP-EXP432P401R**<br/>
**BOOSTXL-EDUMKII**<br/>

## YouTube demonstration 🎥

## Libraries 📚
- <br/>
- <br/>
- <br/>

## Software Description 💻
Here the code using an Msp432 to make a guitar Tuner, the behind idea is to use the integrated microphone in boostXL to sample frequencies from  to Hz, and the buzzer to make a simple methronome. The sampling of frequency is made with Fourier transform to take volt values from microphone to frequency domain. All values taken from microphone is saved using dma to transfer data, then launchpad start to compute data while microfone still save data, when launchpad has computed data, we have an output from the screen of boost pack, the output say the Tune value of guitar (E-A-D-G-B-e) and with a vertical bar the diatorsion of tone. (-,+)<br/>
When metrononme is selected from main in the screen appear a number, it can be changed rising or reducing each digit. The number represents how many click is made, the unit mesure used is beats per minute (BPM).
