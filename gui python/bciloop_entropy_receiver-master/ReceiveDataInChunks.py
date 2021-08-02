"""Example program to demonstrate how to read a multi-channel time-series
from LSL in a chunk-by-chunk manner (which is more efficient)."""

from pylsl import StreamInlet, resolve_stream
from bciloop_utilities.RingBuffer import RingBuffer
from bciloop_utilities.TimeFilters import ButterFilter
from bciloop_utilities.Hilbert import Hilbert
from bciloop_utilities.Entropy import ShannonEntropy
from bciloop_utilities.SpatialFilters import CommonSpatialPatterns, car_filter
from bciloop_utilities.Integrators import ExponentialIntegrator
from bciloop_utilities.draw.GUI import SMRGUI
from xml.dom import minidom
import numpy as np
import time
import math
import pickle

# TODO add notch filter


def get_param_from_xml(doc, name):
    item = doc.getElementsByTagName(name)
    return item[0].firstChild.data


# first resolve an EEG stream on the lab network
print("looking for an EEG stream...")
streams = resolve_stream('type', 'EEG')
labelname = ['STAND', 'WALK']

# create a new inlet to read from the stream
inlet = StreamInlet(streams[0])
streaminfo = inlet.info()
srate = streaminfo.nominal_srate()
nchans = streaminfo.channel_count()

# Settings
xml_settings = minidom.parse('bciloop_receiver_settings.xml')
window_length = float(get_param_from_xml(xml_settings, 'window-length'))
window_shift = float(get_param_from_xml(xml_settings, 'window-shift'))
filter_order = int(get_param_from_xml(xml_settings, 'filter-order'))
filter_lowf = float(get_param_from_xml(xml_settings, 'filter-lowf'))
filter_highf = float(get_param_from_xml(xml_settings, 'filter-highf'))
nbins = int(get_param_from_xml(xml_settings, 'entropy-nbins'))
cspdimm = int(get_param_from_xml(xml_settings, 'csp-dimm'))
alpha = float(get_param_from_xml(xml_settings, 'integrator-alpha'))
threshold = float(get_param_from_xml(xml_settings, 'integrator-threshold'))
rejection = float(get_param_from_xml(xml_settings, 'integrator-rejection'))
begin = float(get_param_from_xml(xml_settings, 'integrator-begin'))
screen_height = int(get_param_from_xml(xml_settings, 'screen-height'))
screen_width = int(get_param_from_xml(xml_settings, 'screen-width'))
screen_scale = int(get_param_from_xml(xml_settings, 'screen-scale'))
nclasses = int(get_param_from_xml(xml_settings, 'num-classes'))

idx_chan=[3,4,5,6,7,8,9,10,11,13,14,15,16,17,19,20,21,22,23,24,25,26,27,28,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,54,55,59,60,61,62]

# Set up draw engine
gui = SMRGUI(screen_height, screen_width, screen_scale)
gui.init_bars(nclasses, labelname)
gui.draw()
# Set up ring buffer
bufferlen = math.floor(window_length*srate)
buffershift = math.floor(window_shift*srate)
ringbuffer = RingBuffer(bufferlen)
# Set up butter filter
btfilt = ButterFilter(filter_order,
                      low_f=filter_lowf, high_f=filter_highf,
                      filter_type='bandpass', fs=srate)
# Set up Hilbert and Entropy
hilb = Hilbert()
entropy = ShannonEntropy(nbins)
# Common Spatial Patterns settings
csp_coeff = np.load('csp_coeff.npy')
csp = CommonSpatialPatterns(cspdimm, csp_coeff)
# Linear Discriminant Analysis
clf = pickle.load(open('lda_model.sav', 'rb'))
# Set up integrator
integrator = ExponentialIntegrator(alpha, threshold, rejection, begin)

time.sleep(1)
while True:
    t = time.time()
    chunk, timestamps = inlet.pull_chunk()
    chunk2 = [[i[j] for j in idx_chan] for i in chunk]
    if timestamps:
        ringbuffer.append(chunk2)
        if ringbuffer.isFull:
            # Reshape buffer
            data = np.array(ringbuffer.data)
            np.clip(data, -400, 400, out=data)
            # CAR filter
            dcar = car_filter(data, axis=1)
            # Bandpass filter
            dfilt = btfilt.apply_filt(dcar)
            # Hilbert envelope
            hilb.apply(dfilt)
            denv = hilb.get_envelope()
            # Shannon Entropy
            dentropy = entropy.apply(denv)
            # Common Spatial Patterns
            dcsp = csp.apply(dentropy.reshape((1, len(dentropy))))
            dproba = clf.predict_proba(dcsp)
            pred, ipp = integrator.apply(np.squeeze(dproba))
            for c in range(nclasses):
                gui.set_value_bars(gui.normalize_probabilities(ipp[c], 1.0, 1/nclasses), c)
                gui.set_alpha_bars(0.8, c) if c == pred else gui.set_alpha_bars(0.5, c)
            print(str(ipp) + '  ->  ' + labelname[int(pred)])
        else:
            print('Filling the buffer...')

    # ensure a constant classification rate
    elapsed = time.time()-t
    if elapsed > window_shift:
        print('Warning!!! The loop had a delay of ' + str(elapsed-window_shift) + ' second!')
    else:
        time.sleep(window_shift-elapsed)
