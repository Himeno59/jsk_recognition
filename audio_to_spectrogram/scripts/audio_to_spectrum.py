#!/usr/bin/env python

import numpy as np
import rospy
from distutils.version import LooseVersion
import pkg_resources

from audio_common_msgs.msg import AudioData
from jsk_recognition_msgs.msg import Spectrum

from audio_to_spectrogram import AudioBuffer

# This node execute FFT to audio (audio_common_msgs/AudioData)
# The format of audio topic is assumed to be wave.

class AudioToSpectrum(object):

    def __init__(self):
        super(AudioToSpectrum, self).__init__()

        self.audio_buffer = AudioBuffer.from_rosparam(auto_start=True)
        fft_sampling_period = rospy.get_param('~fft_sampling_period', 0.3)
        self.audio_buffer.window_size = fft_sampling_period
        mic_sampling_rate = self.audio_buffer.input_sample_rate

        # fft config
        window_function = np.arange(
            0.0, 1.0, 1.0 / self.audio_buffer.audio_buffer_len)
        self.window_function = 0.54 - 0.46 * np.cos(
            2 * np.pi * window_function)
        high_cut_freq = rospy.get_param('~high_cut_freq', 800)
        if high_cut_freq > mic_sampling_rate / 2:
            rospy.logerr('Set high_cut_freq lower than {} Hz'.format(
                mic_sampling_rate / 2))
        low_cut_freq = rospy.get_param('~low_cut_freq', 1)  # remove 0 Hz
        self.freq = np.fft.fftfreq(
            self.audio_buffer.audio_buffer_len, d=1./mic_sampling_rate)
        self.cutoff_mask = np.where(
            (low_cut_freq <= self.freq) & (self.freq <= high_cut_freq),
            True, False)
        # How many times fft is executed in one second
        # fft_exec_rate equals to output spectrogram hz
        self.fft_exec_rate = rospy.get_param('~fft_exec_rate', 50)

        # Publisher and Subscriber
        self.pub_spectrum = rospy.Publisher(
            '~spectrum', Spectrum, queue_size=1)
        self.pub_spectrum_filtered = rospy.Publisher(
            '~spectrum_filtered', Spectrum, queue_size=1)
        self.pub_log_spectrum = rospy.Publisher(
            '~log_spectrum', Spectrum, queue_size=1)
        self.pub_log_spectrum_filtered = rospy.Publisher(
            '~log_spectrum_filtered', Spectrum, queue_size=1)

        timer_kwargs = dict(
            period=rospy.Duration(1.0 / self.fft_exec_rate),
            callback=self.timer_cb,
            oneshot=False,
        )
        if (LooseVersion(pkg_resources.get_distribution('rospy').version) >=
                LooseVersion('1.12.0')) and rospy.get_param('/use_sim_time', None):
            # on >=kinetic, it raises ROSTimeMovedBackwardsException
            # when we use rosbag play --loop.
            timer_kwargs['reset'] = True
        self.timer = rospy.Timer(**timer_kwargs)

    def publish_spectrum(self, pub, pub_filtered, amplitude):
        spectrum_msg = Spectrum()
        spectrum_msg.header.stamp = rospy.Time.now()
        spectrum_msg.amplitude = amplitude
        spectrum_msg.frequency = self.freq
        pub.publish(spectrum_msg)
        spectrum_msg.amplitude = amplitude[self.cutoff_mask]
        spectrum_msg.frequency = self.freq[self.cutoff_mask]
        pub_filtered.publish(spectrum_msg)

    def timer_cb(self, timer):
        if len(self.audio_buffer) != self.audio_buffer.audio_buffer_len:
            return
        audio_data = self.audio_buffer.read()
        # Calc spectrum by fft
        fft = np.fft.fft(audio_data * self.window_function)
        self.publish_spectrum(
            self.pub_spectrum,
            self.pub_spectrum_filtered,
            np.abs(fft / (self.audio_buffer.audio_buffer_len / 2)),
            # Usual "amplitude spectrum" is np.abs(fft),
            # but we use the above equation to get "amplitude"
            # consistent with the amplitude of the original signal.
            # https://github.com/jsk-ros-pkg/jsk_recognition/issues/2761#issue-1550715400
        )
        self.publish_spectrum(
            self.pub_log_spectrum,
            self.pub_log_spectrum_filtered,
            np.log(np.abs(fft)),
            # Usually, log is applied to "power spectrum" (np.abs(fft)**2):
            # np.log(np.abs(fft)**2), 20*np.log10(np.abs(fft)), ...
            # But we use the above equation for simplicity.
            # https://github.com/jsk-ros-pkg/jsk_recognition/issues/2761#issuecomment-1445810380
            # http://makotomurakami.com/blog/2020/05/23/5266/
        )


if __name__ == '__main__':
    rospy.init_node('audio_to_spectrum')
    AudioToSpectrum()
    rospy.spin()
