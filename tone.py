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
    fundFreq = config.INITIAL_FREQUENCY

    # Constructor
    def __init__(self, ratio, amplitude):
        self.ratio = ratio
        self.amplitude = amplitude
        self.phase = 0

        # Check if this is the first tone that has been instantiated
        # if isinstance(tone, type):
        if ratio == 1:
            # If not, apply ratio to determine initial frequency
            self.frequency = config.INITIAL_FREQUENCY * self.ratio
        else:
            # If it's the first, set initial frequency
            self.frequency = config.INITIAL_FREQUENCY

        # Generate initial phase array from 0 to the number of frames per buffer
        # spaced by the appropriate rate
        self.phaseArray = np.arange(0,config.FRAMES_PER_BUFFER * 2 * np.pi * self.frequency/config.RATE, 2 * np.pi * self.frequency/config.RATE)
        # print(self.phaseArray)


    def updatePhase(self):
        self.phase += 2 * np.pi * self.frequency/config.RATE

    # Update frequency with respect to fundamental
    def updateFrequency(self):
        self.frequency = Tone.fundFreq * self.ratio

        # If this Tone is the fundamental, update the shared fundFreq variable
        if self.ratio == 1:
            Tone.fundFreq = self.frequency

    def updatePhaseArray(self):
        self.phaseArray += 2 * np.pi * self.frequency/config.RATE
