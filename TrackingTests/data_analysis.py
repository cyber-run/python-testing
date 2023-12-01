from scipy.signal import hanning, savgol_filter, hilbert
from matplotlib.ticker import LogLocator
import matplotlib.pyplot as plt
import numpy as np
import os

def analyze_data(directory_path: str, known_period: float):
    """
    Analyze collected servo tracking data.

    Parameters:
    - directory_path (str): The path to the directory containing the data files.

    Returns:
    - Tuple: A tuple containing tracker_yaw, target_yaw, amplitude, and phase information.
    """
    # Load data from saved NumPy arrays
    temp_dir = 'temp'
    directory_path = os.path.join(temp_dir, directory_path)

    time_stamp = np.load(os.path.join(directory_path, 'time_stamp.npy'))
    yaw = np.load(os.path.join(directory_path, 'yaw.npy'))
    angle_error = np.load(os.path.join(directory_path, 'angle_error.npy'))

    # Combine tracker_yaw and target_yaw
    tracker_yaw = yaw
    target_yaw = tracker_yaw + angle_error

    # # Apply Savitzky-Golay filter
    # tracker_yaw = savgol_filter(tracker_yaw, window_length=5, polyorder=3)
    # target_yaw = savgol_filter(target_yaw, window_length=5, polyorder=3)

    # plot_signals(tracker_yaw, tracker_yaw_smooth, 'Tracker Yaw')
    # plot_signals(target_yaw, target_yaw_smooth, 'Target Yaw')

    # Call the function to calculate amplitude and phase
    amplitude, phase = calculate_amplitude_and_phase(time_stamp, tracker_yaw, target_yaw, known_period)

    return tracker_yaw, target_yaw, amplitude, phase


def calculate_amplitude_and_phase(timestamps, tracker_yaw, target_yaw, known_period):
    """
    Calculate amplitude and phase from servo tracking data.

    Parameters:
    - timestamps: Timestamps of the collected data.
    - tracker_yaw: Tracker yaw in the servo tracking.
    - target_yaw: Target yaw in the servo tracking.
    - known_period: Known period of the signals.

    Returns:
    - Tuple: A tuple containing amplitude and phase information.
    """
    # Calculate the number of samples per period
    avg_samples_per_period = int(known_period / np.mean(np.diff(timestamps)))

    # Extract one period of data
    one_T_tracker_yaw = tracker_yaw[:avg_samples_per_period]
    one_T_target_yaw = target_yaw[:avg_samples_per_period]

    # Calculate amplitude and phase
    amplitude_tracker = np.max(one_T_tracker_yaw)
    amplitude_target = np.max(one_T_target_yaw)
    amplitude = amplitude_tracker / amplitude_target

    phase = cross_correlate(one_T_tracker_yaw, one_T_target_yaw)

    return amplitude, phase


def calculate_phase(tracker_yaw, target_yaw):
    """
    Calculate phase from servo tracking data.

    Parameters:
    - tracker_yaw: Yaw angles of the servo.
    - target_yaw: Error angles in the servo tracking.

    Returns:
    - float: Phase information.
    """
    # Represent the signals as column vectors
    x = tracker_yaw.reshape(-1, 1)
    y = target_yaw.reshape(-1, 1)

    # Remove the DC component of the signals
    x = x - np.mean(x)
    y = y - np.mean(y)

    # Signals length calculation
    xlen = len(x)
    ylen = len(y)

    # Windows generation
    xwin = hanning(xlen, 'periodic')
    ywin = hanning(ylen, 'periodic')

    # Perform FFT on the signals
    X = np.fft.fft(x * xwin)
    Y = np.fft.fft(y * ywin)

    # Fundamental frequency detection
    indx = np.argmax(np.abs(X), axis=None)
    indy = np.argmax(np.abs(Y), axis=None)
    
    # Normalize the indices
    indx = int(indx/xlen)
    indy = int(indy/ylen)

    # Phase difference estimation
    phase_diff = np.angle(Y[indy]) - np.angle(X[indx])
    phase_diff = np.unwrap([phase_diff])[0]

    return phase_diff


def hillbert_phase(tracker_yaw, target_yaw):
    """
    Calculate phase using Hilbert transform.

    Parameters:
    - tracker_yaw (numpy.ndarray): Yaw angles of the servo.
    - target_yaw (numpy.ndarray): Error angles in the servo tracking.

    Returns:
    - numpy.ndarray: Phase difference information.
    """

    analytic_x = hilbert(tracker_yaw)
    analytic_y = hilbert(target_yaw)

    phase_x = np.angle(tracker_yaw)
    phase_y = np.angle(target_yaw)

    phase_diff = np.unwrap(tracker_yaw - target_yaw)

    return phase_diff


def cross_correlate(x, y):
    """
    Calculate phase difference using cross-correlation.

    Parameters:
    - x (numpy.ndarray): Input signal x.
    - y (numpy.ndarray): Input signal y.

    Returns:
    - float: Phase difference information.
    """

    cross_corr = np.correlate(x, y, mode='full')

    lag = np.argmax(cross_corr) - len(x) + 1

    phase_diff = 2 * np.pi * lag / len(x)

    return phase_diff


def plot_signals(x, y, title):
    """
    Plot original and filtered signals.

    Parameters:
    - x (numpy.ndarray): Original signal.
    - y (numpy.ndarray): Filtered signal.
    - title (str): Title of the plot.
    """

    plt.plot(x, label='Original Signal')
    plt.plot(y, label='Filtered Signal')
    plt.title(title)
    plt.xlabel('Sample Index')
    plt.ylabel('Amplitude')
    plt.legend()
    plt.show()


def plot_bode(freq, gain_dB, phase, title, save_path='temp'):
    """
    Plot Bode plot and optionally save frequency, gain_dB, and phase data.

    Parameters:
    - freq (numpy.ndarray): Array of frequencies.
    - gain_dB (numpy.ndarray): Array of gain values in dB.
    - phase (numpy.ndarray): Array of phase values in degrees.
    - title (str): Title of the plot.
    - save_path (str, optional): Path to save frequency, gain_dB, and phase data. Default is 'temp'.
    """

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # Plot gain in dB
    ax1.set_ylabel('Gain (dB)')
    ax1.semilogx(freq, gain_dB, 'x-',)
    ax1.tick_params(axis='y')
    ax1.grid(True, which='both', linestyle='--', linewidth=0.5)  # Add grid lines

    # Plot phase
    ax2.set_xlabel('Frequency (Hz)')
    ax2.set_ylabel('Phase Difference (degrees)')
    ax2.semilogx(freq, phase, 'x-')
    ax2.tick_params(axis='y')
    ax2.grid(True, which='both', linestyle='--', linewidth=0.5)  # Add grid lines

    # Set major and minor ticks on x-axis
    major_locator = LogLocator(base=10.0, numticks=12)
    minor_locator = LogLocator(base=10.0, subs=np.arange(2, 10) * 0.1, numticks=12)
    ax2.xaxis.set_major_locator(major_locator)
    ax2.xaxis.set_minor_locator(minor_locator)
    ax2.xaxis.set_minor_formatter(plt.NullFormatter())

    fig.suptitle(title)

    if save_path:
        os.makedirs(save_path, exist_ok=True)
        np.save(os.path.join(save_path, 'frequency.npy'), freq)
        np.save(os.path.join(save_path, 'gain_dB.npy'), gain_dB)
        np.save(os.path.join(save_path, 'phase.npy'), phase)

    plt.show()