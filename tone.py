import config
import numpy as np

class Tone:
    # Store data for each tone to be generated:
    # ratio = harmonic ratio (float) to fundamental frequency
    # amplitude = relative loudness of this Tone (float 0-0.5 usually)
    #
    # automatically updated:
    # phase, frequency (both variable)

    # Create shared class/static variable for fundamental frequency
    fund_freq = config.INITIAL_FREQUENCY

    # Constructor
    def __init__(self, ratio=1, amplitude=0.1):
        self.ratio = ratio
        self.amplitude = amplitude

        # Check if this is the first tone that has been instantiated
        if ratio == 1:
            # If it's the first, set initial frequency
            self.frequency = config.INITIAL_FREQUENCY
        else:
            # If not, apply ratio to determine initial frequency
            self.frequency = Tone.fund_freq * self.ratio

        # print(f'My freq = {self.frequency}')

        # Generate initial phase array from 0 to the number of frames per buffer
        # spaced by the appropriate rate
        phaseStep = 2 * np.pi * self.frequency/config.RATE
        self.phaseArray = np.linspace(0,config.FRAMES_PER_BUFFER * phaseStep, config.FRAMES_PER_BUFFER)

    # Update frequency with respect to fundamental
    def updateFrequency(self):
        self.frequency = Tone.fund_freq * self.ratio
        # print('my freq ', self.frequency)
        # If this Tone is the fundamental, update the shared fund_freq variable
        if self.ratio == 1:
            Tone.fund_freq = self.frequency

    def updatePhaseArray(self):
        last_val = self.phaseArray[-1]
        phaseStep = 2 * np.pi * self.frequency/config.RATE

        np.copyto(self.phaseArray,np.linspace(last_val, last_val + (config.FRAMES_PER_BUFFER * phaseStep), config.FRAMES_PER_BUFFER))


# Instantiate a list of tone objects with relative harmonic ratios and
# amplitudes
# ClassTones = [Tone(1, 0.3), Tone(1.5, 0.2), Tone(2, 0.05)]
ClassTones = [Tone(1, 0.3), Tone(1.5, 0.18), Tone(2, 0.3)]
