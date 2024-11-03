#!/usr/bin/python3.5


# plot filters



import math
import signal
import tkinter as tk

WINDOW_W = 640
WINDOW_H = 400
STEP = 10
GRAPH_W = WINDOW_W - 30
GRAPH_H = WINDOW_H - 30
GRAPH_X = WINDOW_W - GRAPH_W
GRAPH_Y = 10
SAMPLERATE = 100
#MIN_FREQ = 1
MIN_FREQ = .1
MAX_FREQ = 10
PRINT_FREQS = [ .1, .2, .3, .4, .5, .7, 1, 2, 3, 4, 5, 7, 10 ]
PRINT_POWERS = [ 1, .7, .5, .4, .3, .2, .1 ]

ORDER = 2
class Filter:
    def __init__(self, bandwidth):
        self.prev_output = []
        self.prev_input = []
        for i in range(ORDER):
            self.prev_output.append(0)
            self.prev_input.append(0)
        self.result = 0
        self.bandwidth = bandwidth

    def highpass(self, value):
        self.result = value
        for i in range(ORDER):
            self.result = self.bandwidth * (self.prev_output[i] + value - self.prev_input[i])
            self.prev_input[i] = value
            self.prev_output[i] = self.result
            value = self.result
        return self.result

    def lowpass(self, value):
        self.result = value
        for i in range(ORDER):
            self.result = self.prev_output[i] + self.bandwidth * (value - self.prev_output[i])
            self.prev_input[i] = value
            self.prev_output[i] = self.result
            value = self.result
        return self.result


class NotchFilter:
    def __init__(self, center, width, period):
        


# test filter
test_lowpass = Filter(.05)
test_highpass = Filter(.97)
def test_function(v):
#    low = test_lowpass.lowpass(v) * 100
    high = test_highpass.highpass(v) * 100
    return high

#    result = low + leash_highpass.highpass(low) * 200
#    result = low + leash_highpass.highpass(v) * 40
#    result = leash_lowpass.lowpass(v) * leash_highpass.highpass(v)
#    return result

# leash filter
leash_p = 230
#leash_p = 0
leash_p2 = 1000
leash_lowpass = Filter(.08)
leash_lowpass2 = Filter(.1)
leash_highpass = Filter(.9)
#leash_p = 200
#leash_p2 = 250
#leash_lowpass = Filter(.08)
#leash_highpass = Filter(.96)
def leash_function(v):
    low = leash_lowpass.lowpass(v)
#    high = leash_highpass.highpass(low)
    low2 = leash_lowpass2.lowpass(v)
    high = leash_highpass.highpass(low2)
    result = low * leash_p + high * leash_p2
    return result

# steering filter
steering_lowpass = Filter(.02)
steering_highpass = Filter(.91)
def steering_function(v):
    low = steering_lowpass.lowpass(v)
    high = steering_highpass.highpass(low)
    result = low * 250 + high * 7700
    return result

min_x = math.log10(MIN_FREQ)
max_x = math.log10(MAX_FREQ)
def freq_to_x(f):
    return (math.log10(f) - min_x) / (max_x - min_x) * GRAPH_W


def x_to_freq(x):
    x3 = x / WINDOW_W * (max_x - min_x) + min_x
    return pow(10, x3)

min_y = math.log10(.1)
max_y = math.log10(1)
def power_to_y(v):
    return (math.log10(v) - min_y) / (max_y - min_y) * GRAPH_H

# how many periods to sample
PERIODS = 100
def compute_power(freq):
    samples = int(SAMPLERATE * PERIODS / freq)
#    samples = SAMPLERATE * 10
    max = 0
    for i in range(samples):
        input = math.sin(i / SAMPLERATE * 2 * math.pi * freq)
#        output = steering_function(input)
        output = leash_function(input)
        abs_output = abs(output)
# drop 1st period & get maximum of remaneing samples
        if i >= SAMPLERATE / freq and abs_output > max:
            max = abs_output
#    return math.log10(max)
    return max


#compute_power(0)


buffer = []
for x in range(GRAPH_W):
    buffer.append(0)

# compute values
# reset the filters
compute_power(MIN_FREQ)
for x in range(0, GRAPH_W, STEP):
    freq = x_to_freq(x)
    buffer[x] = compute_power(freq)
    print("%d%%: %fHz=%f" % (int(x * 100 / GRAPH_W), freq, buffer[x]))
#    buffer.append(math.sin(x / GRAPH_W * 2 * math.pi) + 1)

# normalize
max = -99999
min = 99999
for x in range(0, GRAPH_W, STEP):
    if buffer[x] > max:
        max = buffer[x]
    if buffer[x] < min:
        min = buffer[x]

print("max=%f min=%f" % (max, min))

win = tk.Tk()
win.title("Leash response")
win.geometry(str(WINDOW_W) + "x" + str(WINDOW_H) + "+0+0")
canvas = tk.Canvas(win, bg='white', highlightthickness=0)
# fill white
canvas.pack(fill=tk.BOTH, expand=1)
# rectangle
canvas.create_rectangle(GRAPH_X, 
    GRAPH_Y, 
    GRAPH_X + GRAPH_W, 
    GRAPH_Y + GRAPH_H, 
    fill='white', 
    width=1)
# frequency marks
for i in range(len(PRINT_FREQS)):
    x = freq_to_x(PRINT_FREQS[i]) + GRAPH_X
    canvas.create_line(x, GRAPH_Y, x, GRAPH_Y + GRAPH_H, fill='grey', width=1)
    if PRINT_FREQS[i] < 1:
        the_text = "%.1f" % PRINT_FREQS[i]
    else:
        the_text = "%.0f" % PRINT_FREQS[i]
    justify_ = 'center'
    if i == 0:
        x += 10
    else:
        if i == len(PRINT_FREQS) - 1:
            x -= 10
    canvas.create_text(x, 
        GRAPH_Y + GRAPH_H + 10, 
        text=the_text,
        font=("Helvetica 8"),
        fill='black')

# power marks
#for i in range(len(PRINT_POWERS)):
#    y = GRAPH_Y + GRAPH_H - power_to_y(PRINT_POWERS[i])
#    canvas.create_line(GRAPH_X, 
#        y, 
#        GRAPH_X + GRAPH_W, 
#        y, 
#        fill='grey', 
#        width=1)
#    the_text = "%.1f" % (PRINT_POWERS[i] * (max - min) + min)
#    canvas.create_text(15, 
#        y, 
#        text=the_text,
#        font=("Helvetica 8"),
#        fill='black')

for i in range(11):
    y = GRAPH_Y + GRAPH_H - i * GRAPH_H / 10
    canvas.create_line(GRAPH_X, 
        y, 
        GRAPH_X + GRAPH_W, 
        y, 
        fill='grey', 
        width=1)
    the_text = "%.0f" % (i * (max - min) / 10 + min)
    canvas.create_text(15, 
        y, 
        text=the_text,
        font=("Helvetica 8"),
        fill='black')

# draw it
x1 = -STEP
y1 = -1
for x in range(0, GRAPH_W, STEP):
    x2 = x1 + STEP
    y2 = GRAPH_Y + GRAPH_H - 1 - (buffer[x] - min) / (max - min) * (GRAPH_H - 2)
#    y2 = WINDOW_H - (current_freq - MIN_FREQ) / (MAX_FREQ - MIN_FREQ) * WINDOW_H
    if x1 >= 0:
        canvas.create_line(GRAPH_X + x1, y1, GRAPH_X + x2, y2, fill='red', width=2)
    x1 = x2
    y1 = y2

def handle_keypress(e):
    if e.char == 'q':
        exit()

def handle_sig(sig, frame):
    exit(0)

signal.signal(signal.SIGINT, handle_sig)

win.bind('<KeyPress>', lambda e: handle_keypress(e))
win.mainloop()




