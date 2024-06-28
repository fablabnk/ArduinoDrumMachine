# Arduino Drum Machine

## The Goal / Spec

(Current goal is just a draft, we can make this better!)

Build a simple Arduino-based 8-bit drum player with three-channels, where:
- the user can select the three drums out of eight possible samples
- each channel will have a pitch knob
- there will be one global effect knob (i.e. distortion amount) 

So:
- 3 gate inputs (Digital Pins D2-D4)
- 3 sample-select buttons (Digital Pins D5-D8)
- 3 pitch inputs (Analog Pins A0-A2)
- 1 global effect input (Analog Pin A3)
- 4 audio outputs (one for each drum - a use case for our mixer - and one 'mixed' output)

## The Theory: Sample playback using Direct Digital Synthesis (DDR)

The code we will base our project on is the dsp-D8 [here](https://github.com/hexagon5un/jan_ostmans_synths)

The background to it is [here](https://hackaday.com/2016/02/23/a-slew-of-open-source-synthesizers/) and [here](https://hackaday.com/2016/02/12/embed-with-elliot-audio-playback-with-direct-digital-synthesis/). TL;DR is that DDR:

- uses minimal additional hardware - no need for audio WAV player shields etc
- the audio data is embedded in the code (`drum_samples.h`)
- key principle of DDR is that: given a sampled waveform, you can play nearly any frequency from a fixed clock by skipping or repeating points of the sample as necessary
- DDR removes different, evenly spaced samples with each cycle through the sampled waveform

## Getting Started: Flashing an Arduino Nano

I used [Arduino IDE 2](https://docs.arduino.cc/software/ide/) to compile and flash. 
1. Open `File Menu -> Examples -> Basics -> Blink` (try this before uploading our actual code)
2. In `Tools Menu -> Board -> Arduino AVR Boards` select `Arduino Nano`
3. Before plugging in the Nano, check `Tools Menu -> Port` to see what's there
4. After pluging in the Nano, check `Tools Menu -> Port` and select whatever port appeared i.e. `/dev/ttyUSB0`
5. Choose `Tools Menu -> Processor -> ATmega328P` (Old Bootloader)
6. Click upload button (`->` icon)

### Troubleshooting tips

A good general guide is [here](https://www.arduino.cc/en/Guide/ArduinoNano). Plus doing some of the following things allowed me to finally upload my sketch!
- [Ubuntu] If ttyUSB doesn't appear as a port, do `sudo apt remove brltty` then reboot
- Are you using the short blue cable that came with the Nano? Not all cables are created equal. Long black cable didn't work for me
- If sketch still doesn't upload, under Tools -> Processor choose:
ATmega328P (Old Bootloader)
- If the code seems to upload (RX light blinks repeatedly) but you get `avrdude: stk500_recv(): programmer is not responding` afterwards:
	- In Settings -> Preferences -> tick Show verbose output during upload
	- This magically made the board flash next time!

## Uploading the real code

1. Clone [our repo]
2. Open the Arduino IDE
3. Open the `dspD8_v1-0.ino` from within the dspD8 folder
4. Click upload button (`->` icon)

## V1.0: Triggering drums from the serial monitor (most basic version)

Our `dspD8_v1-0.ino` is the exact same code as from the original project [here](https://github.com/hexagon5un/jan_ostmans_synths). Before we get into adding physical control from the Arduino itself, we simply trigger the drums using keypresses from the Arduino IDE's serial monitor.

### Breadboarding the project

1. Power the Arduino Nano via USB
2. Mount the thonkiconn audio 3.5mm audio jack on the board as follows
We will place vertically at the bottom right of the board, between rows 26 and 30, leaving space for wires on the left
- Locate the long outer pin on the thonkiconn, place this in row 30, with the pins on the opposite side going it rows 26 and 27
3. Wire the thonkiconn 3.5 mono audio jack as follows:
- Run a wire from Arduino pin D11 to row 26
- Run a wire from row 30 to an Arduino ground pin (GND)
4. Open the Arduino serial monitor, set it to 9600 baud
- use "asdfjkl;" keys + enter to play the samples
- entering combinations of keys to trigger them simultaneously i.e. 'ad' + enter for both kick and open hat

## V1.1: Triggering the bass drum using a button

Now let's add a physical trigger for our bass drum. For this we will use a digital pin, set in INPUT_PULLUP mode.

According to ChatGPT: `When you set a pin to INPUT_PULLUP, the Arduino internally connects a pull-up resistor to the pin, which means the pin is normally HIGH (due to the connection to 5V through the pull-up resistor). Pressing the button will connect the pin to GND, making the pin read LOW.`

The code is already present to setup the digital pins as input drum triggers i.e. for the bass drum

```
pinMode(2,INPUT_PULLUP);
```

We just need to add these declarations before the `void setup()` function
```
int buttonPin = 2;  // Define the pin where the button is connected
bool lastButtonState = HIGH;  // Variable to store the last button state
```

Add this code before the `cli()` line in the `void loop()` function
```
    // Check for button press
    bool currentButtonState = digitalRead(buttonPin);

    // Trigger the drum when the button is pressed (transition from HIGH to LOW)
    if (lastButtonState == HIGH && currentButtonState == LOW) {
      samplepntBD = 0;
      samplecntBD = 2154;
    }

    // Update the last button state
    lastButtonState = currentButtonState;
```

## V1.2: Adding a pot to change the pitch

When the bass drum is triggered, we will take a reading from analogue pin 0 and map it to pitch. To do this we need to...

Add this line to our declarations:
```
const int potPin = A0;
```

Add this line to `void setup()`
```
pinMode(potPin, INPUT);  // Set the potentiometer pin as input
```

Replace the previous code we added to the `void loop()` fuction with this instead
```
    // Check for button press
    bool currentButtonState = digitalRead(buttonPin);

    // Trigger the drum when the button is pressed (transition from HIGH to LOW)
    if (lastButtonState == HIGH && currentButtonState == LOW) {
      int potValue = analogRead(potPin);  // Read the potentiometer value (0-1023)
      pitchBD = map(potValue, 0, 1023, 16, 128);  // Map the value to the desired pitch range (16-128)
      samplepntBD = 0;
      samplecntBD = 2154;
    }

    // Update the last button state
    lastButtonState = currentButtonState;
```

## V1.3: Making generic drum trigger functions

Now I wanted to use the same code whether triggering from the serial monitor or physical buttons, so I made functions such as:

```
void triggerBD()
{
    int potValue = analogRead(pitchBDPin);  // Read the potentiometer value (0-1023)
    pitchBD = map(potValue, 0, 1023, 16, 128);  // Map the value to the desired pitch range (16-128)
    samplepntBD = 0;
    samplecntBD = 2154;
}
```

## V1.4: Use arrays of pins to simplify the code

Unfortunately the above approach produces a lot of similar code. Now we should condense the code to use arrays, instead of separated named function for each drum. This can be our starting point for adding new features, with the aim being the spec stated at the beginning of this readme!

## Features To Add

(how to add as feature requests?)

- Use our own samples in `drum_samples.h`
	- how to get samples as values?
	- what is the sample rate we're using?
- Reversed playback
- Add 'global' effect i.e. simple pot controlled distortion amount
- Go breadboardless: Building pre-soldered jacks/pots/buttons that can be connect directly to Arduino pins
- Controlling gate and pitch with sensors
- Eurorack it based on https://github.com/wgd-modular/utf-8-samplified
	- gate inputs using voltage
	- including an omp to raise output level to eurorack level

## Guide to Loading Samples

- install Audacity
- load your sample to an empty audacity project
- click on file->export audio on the menu bar
- use the following settings
  filename: "sample.raw"
  folder: choose a suitable folder
  format: Other uncompressed file
  channels: mono
  sample_rate: 40k
  header: RAW (headerless)
  encoding: Unsigned 8 bit
- copy the resulting file to the utilities folder 
- run renderer.py
- now there is a resulting sample.h file that includes an array of unsigned ints
- TODO: edit renderer.py so that it prints the size of the array
- copy the contents of the array to the memory slot you want to replace
- voila! 