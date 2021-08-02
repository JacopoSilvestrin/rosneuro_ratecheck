from scipy.io import savemat, loadmat
import numpy as np
import math
from tqdm import tqdm
import sys
import pickle
from xml.dom import minidom
from bciloop_utilities.TimeFilters import ButterFilter
from bciloop_utilities.Hilbert import Hilbert
from bciloop_utilities.Entropy import ShannonEntropy
from bciloop_utilities.SpatialFilters import CommonSpatialPatterns, car_filter
from bciloop_utilities.Integrators import ExponentialIntegrator
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn.metrics import confusion_matrix, accuracy_score


def get_param_from_xml(doc, name):
    item = doc.getElementsByTagName(name)
    return item[0].firstChild.data


class ProcEEGentropy:
    def __init__(self, WinLength, WinStep, NumBins, fcutoff, forder, srate):
        self.WinLength = math.floor(WinLength*srate)
        self.WinStep = math.floor(WinStep*srate)
        self.btfilt = ButterFilter(forder, low_f=fcutoff[0], high_f=fcutoff[1], filter_type='bandpass', fs=srate)
        self.hilb = Hilbert()
        self.entropy = ShannonEntropy(NumBins)

    def apply(self, signal):
        NumSamples = np.shape(signal)[0]
        NumChans = np.shape(signal)[1]
        WinStart = np.arange(0, NumSamples-self.WinLength+1, self.WinStep)
        WinStop = WinStart + self.WinLength - 1
        NumWins = len(WinStart)
        signal_entropy = np.empty([NumWins, NumChans])

        for wid, wstart in enumerate(tqdm(WinStart, file=sys.stdout)):
            wsignal = signal[wstart:WinStop[wid], :]
            np.clip(wsignal, -400, 400, out=wsignal)
            # CAR filter
            wcar = car_filter(wsignal, axis=1)
            # Bandpass filter
            wfilt = self.btfilt.apply_filt(wcar)
            # Hilbert envelope
            self.hilb.apply(wfilt)
            wenv = self.hilb.get_envelope()
            # Shannon Entropy
            signal_entropy[wid, :] = self.entropy.apply(wenv)
        return signal_entropy


# Settings
xml_settings = minidom.parse('bciloop_receiver_settings.xml')
window_length = float(get_param_from_xml(xml_settings, 'window-length'))
window_shift = float(get_param_from_xml(xml_settings, 'window-shift'))
srate = int(get_param_from_xml(xml_settings, 'sampling-rate'))
filter_order = int(get_param_from_xml(xml_settings, 'filter-order'))
filter_lowf = float(get_param_from_xml(xml_settings, 'filter-lowf'))
filter_highf = float(get_param_from_xml(xml_settings, 'filter-highf'))
nbins = int(get_param_from_xml(xml_settings, 'entropy-nbins'))
cspdimm = int(get_param_from_xml(xml_settings, 'csp-dimm'))
alpha = float(get_param_from_xml(xml_settings, 'integrator-alpha'))
threshold = float(get_param_from_xml(xml_settings, 'integrator-threshold'))
rejection = float(get_param_from_xml(xml_settings, 'integrator-rejection'))
begin = float(get_param_from_xml(xml_settings, 'integrator-begin'))
load_data = bool(int(get_param_from_xml(xml_settings, 'load-data')))
save_classifier = bool(int(get_param_from_xml(xml_settings, 'save-classifier')))
cross_validation = bool(int(get_param_from_xml(xml_settings, 'crossval-apply')))
nfold = int(get_param_from_xml(xml_settings, 'crossval-nfold'))

print('[TrainEntropyClassifier] Initialize processing')
proc_entropy = ProcEEGentropy(window_length,
                              window_shift,
                              nbins,
                              [filter_lowf, filter_highf],
                              filter_order,
                              srate)
csp = CommonSpatialPatterns(cspdimm)
clf = LinearDiscriminantAnalysis(solver='eigen', priors=[0.5, 0.5], shrinkage='auto')
integrator = ExponentialIntegrator(alpha, threshold, rejection, begin)

# COMPUTE OR LOAD ENTROPY
if load_data is True:
    entropy_walking = np.load('entropy_walking.npy')
    entropy_standing = np.load('entropy_standing.npy')
else:
    # Load subject's dataset
    print('[TrainEntropyClassifier] Loading subject dataset')
    data_dict = loadmat('redEEG.mat')
    data_struct = data_dict['redEEG'][0, 0]
    data = np.transpose(np.array(data_struct['data']))
    labels = np.squeeze(np.array(data_struct['labels']))
    # downsampling to 512 Hz
    #data = data[::2, :]
    #labels = labels[::2]
    data_walking = data[labels == 1, :]
    data_standing = data[labels == 0, :]

    # Compute entropy
    print('[TrainEntropyClassifier] Processing walking data...')
    entropy_walking = proc_entropy.apply(data_walking)
    np.save('entropy_walking.npy', entropy_walking)
    print('[TrainEntropyClassifier] Processing standing data...')
    entropy_standing = proc_entropy.apply(data_standing)
    np.save('entropy_standing.npy', entropy_standing)
    savemat('entropy.mat', {'standing': entropy_standing,
                            'walking': entropy_walking})

# CROSS-VALIDATION
if cross_validation is True:
    print('\n[TrainEntropyClassifier] Preparing cross-validation...')
    entropy_walking = entropy_walking[:np.shape(entropy_walking)[0] - (np.shape(entropy_walking)[0] % nfold), :]
    entropy_standing = entropy_standing[:np.shape(entropy_standing)[0] - (np.shape(entropy_standing)[0] % nfold), :]
    nfold_samples_walk = np.shape(entropy_walking)[0] / nfold
    nfold_samples_stand = np.shape(entropy_standing)[0] / nfold

    cnf_mat = np.empty((2, 2, nfold))
    acc = np.empty(nfold)
    for n in np.arange(0, nfold):
        fold_start = int(n * nfold_samples_walk)
        fold_stop = int(fold_start + nfold_samples_walk - 1)
        train_walk = np.delete(entropy_walking, np.s_[fold_start:fold_stop], axis=0)
        test_walk = entropy_walking[fold_start:fold_stop, :]

        fold_start = int(n * nfold_samples_stand)
        fold_stop = int(fold_start + nfold_samples_stand - 1)
        train_stand = np.delete(entropy_standing, np.s_[fold_start:fold_stop], axis=0)
        test_stand = entropy_standing[fold_start:fold_stop, :]

        ndiff = np.shape(train_walk)[0] - np.shape(train_stand)[0]
        if ndiff > 0:
            train_stand = np.concatenate((train_stand,
                                          train_stand[np.random.choice(np.shape(train_stand)[0], size=ndiff), :]))
        else:
            train_walk = np.concatenate((train_walk,
                                         train_walk[np.random.choice(np.shape(train_walk)[0], size=-ndiff), :]))
        train_data = np.concatenate((train_stand, train_walk), axis=0)
        train_labels = np.concatenate((np.zeros(np.shape(train_stand)[0]), np.ones(np.shape(train_walk)[0])), axis=0)
        test_data = np.concatenate((test_stand, test_walk), axis=0)
        test_labels = np.concatenate((np.zeros(np.shape(test_stand)[0]), np.ones(np.shape(test_walk)[0])), axis=0)

        print('\n   Training and validate iteration ' + str(n+1) + '/' + str(nfold))
        # Compute CSP filter
        csp.compute_filters(train_data, train_labels)
        csp_train_data = csp.apply(train_data)
        csp_test_data = csp.apply(test_data)

        # Train LDA classifier
        clf.fit(csp_train_data, train_labels)
        dproba = clf.predict_proba(csp_test_data)
        pred, ipp = integrator.apply_array(dproba)
        cnf_mat[:, :, n] = confusion_matrix(test_labels, pred, normalize='true')
        acc[n] = accuracy_score(test_labels, pred)
        print('     Confusion matrix: ')
        print(cnf_mat[:, :, n])
        print('     Accuracy: ' + str(acc[n]))

        # Reset csp and integrator before next iteration
        csp.reset_coeff()
        integrator.reset()
    print('\n[TrainEntropyClassifier] Average performance: ')
    print(np.mean(cnf_mat, axis=2))
    print('Accuracy: ' + str(np.mean(acc)))

# TRAIN CLASSIFIER WITH ALL THE DATASET
if save_classifier is True:
    print('[TrainEntropyClassifier] Training and saving classifier')
    ndiff = np.shape(entropy_walking)[0] - np.shape(entropy_standing)[0]
    if ndiff > 0:
        train_walk = entropy_walking
        train_stand = np.concatenate((entropy_standing,
                                      entropy_standing[np.random.choice(np.shape(entropy_standing)[0], size=ndiff), :]))
    else:
        train_stand = entropy_standing
        train_walk = np.concatenate((entropy_walking,
                                     entropy_walking[np.random.choice(np.shape(entropy_walking)[0], size=-ndiff), :]))
    train_data = np.concatenate((train_stand, train_walk), axis=0)
    train_labels = np.concatenate((np.zeros(np.shape(train_stand)[0]), np.ones(np.shape(train_walk)[0])), axis=0)
    # CSP filter
    csp.compute_filters(train_data, train_labels)
    csp_train_data = csp.apply(train_data)
    # LDA training
    clf.fit(csp_train_data, train_labels)
    # save
    np.save('csp_coeff.npy', csp.coeff)
    pickle.dump(clf, open('lda_model.sav', 'wb'))
print('[TrainEntropyClassifier] Successfully exiting the program!')



