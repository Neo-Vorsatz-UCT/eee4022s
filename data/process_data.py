"""
    Program to extract and display data
    Author: Neo Vorsatz
    Last updated: 27 October 2026
"""

# IMPORTS ================================

# External
import csv
import re
from itertools import islice
from typing import Iterator
import math
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
import statistics
import numpy as np
from scipy.interpolate import make_smoothing_spline

# TYPES ================================

Number = int | float

# CONSTANTS ================================

# File paths
_FILE_CSV = "laptop_data.csv"
_FILE_PING = "network_data.txt"
_FILE_LOG = "esp32_data.log"
_FILE_ACT = "actuation_angles.txt"

# Real column names
_COL_FETCH_FRAME = "fetch frame (ms)"
_COL_LANDMARK = "landmark (ms)"
_COL_CALC_TARGET = "calc target (ms)"
_COL_TRANSMISSION = "transmission (ms)"
_COL_IK_SOLVER = "ik solver (ms)"
_COL_ITERATIONS = "iterations (#)"
_COL_SQR_ERR = "sqr err (mm^2)"
_COL_PING = "ping (ms)"
_COL_RECEPTION = "reception (ms)"
_COL_ACTUATION = "actuation (degrees)"

# Fake column names
_COL_IDX = "idx" # calls the `range()` funciton
_COL_UART = "uart" # calculated value of UART latency (ms)
_COL_SERVO = "servo" # effects of exponential smoothing

# Data related constants
_MAX = 10**4 # maximum number of rows in any data set

# GENERAL HELPER FUNCTIONS ================================

def _to_number(text: str) -> Number:
    """
    Convert text to int if possible, otherwise float.
    Empty/blank text (a missing value) becomes `float("nan")`.
    """
    text = text.strip()
    if not text:
        return float("nan")
    try:
        return int(text)
    except ValueError:
        return float(text)

# CSV FILE ================================

def _csv_columns() -> dict[str,int]:
    """
    Generates a dictionary of column names mapping to their indices, in the CSV file

    Returns:
        dict[str,int]: The dictionary of column names mapping to their indices
    """
    # Extract the header
    file = open(_FILE_CSV, newline="")
    header = next(csv.reader(file))
    file.close()

    # Create dictionary mapping column names to their index
    columns: dict[str,int] = {}
    for i, name in enumerate(header):
        if not name.strip():  # if heading is empty string
            continue
        columns[name.strip()] = i
    return columns

def _iter_csv(col:int) -> Iterator[Number]:
    """
    Generator function, yielding every value of a column in the CSV file.

    Args:
        col (int): The index of the column
    Returns:
        Iterator[Number]: The values of the column
    """
    file = open(_FILE_CSV, newline="")
    reader = csv.reader(file)
    next(reader)  # skip header
    for row in reader:
        if col<len(row):
            yield _to_number(row[col])
        else:
            yield float("nan")
    file.close()

def get_csv_data(key:str, end:int, start:int=0) -> list[Number]:
    """
    Read a slice of a column from the CSV file

    Args:
        key (str): column name
        end (int): last row, exclusive
        start (int): first row, inclusive
    Returns:
        list[Number]: The data in the column. Empty cells are `float("nan")`
    """
    return list(islice(_iter_csv(_csv_columns()[key]), start, end))

# PING FILE ================================

def _iter_ping() -> Iterator[Number]:
    """
    Generator function, yielding all the round trip times in the Ping file

    Returns:
        Iterator[Number]: The values of the column
    """
    PING_RE = re.compile(r"icmp_seq=(\d+) ttl=(\d+) time=([\d.]+) ms")
    file = open(_FILE_PING)
    for line in file:
        result = PING_RE.search(line)
        if result:
            yield float(result.group(3))/2 # divide by two for one-way time
    file.close()

def get_ping_data(end:int, start:int=0) -> list[Number]:
    """
    Read a slice of latency data from the Ping file

    Args:
        end (int): last row, exclusive
        start (int): first row, inclusive
    Returns:
        list[Number]: The data in the column
    """
    return list(islice(_iter_ping(), start, end))

# LOG FILE ================================

def _iter_log() -> Iterator[Number]:
    """
    Generator function, yielding all the reception latencies in the Ping file

    Returns:
        Iterator[Number]: The values of the column
    """
    LOG_RE = re.compile(r"^I \((\d+)\) dtls_rx: latency: ([\d.]+)")
    file = open(_FILE_LOG)
    for line in file:
        result = LOG_RE.match(line)
        if result:
            yield float(result.group(2))
    file.close()

def get_log_data(end: int, start: int = 0) -> list[Number]:
    """
    Read a slice of latency data from the Log file

    Args:
        end (int): last row, exclusive
        start (int): first row, inclusive
    Returns:
        list[Number]: The data in the column
    """
    return list(islice(_iter_log(), start, end))

# ACTUATION FILE ================================

def _iter_act() -> Iterator[Number]:
    """
    Generator function, yielding all the actuation angles in the Act file

    Returns:
        Iterator[Number]: The values of the column
    """
    file = open(_FILE_ACT)
    for line in file:
        yield float(line)
    file.close()

def get_act_data(end: int, start: int = 0) -> list[Number]:
    """
    Read a slice of actuation angles from the Act file

    Args:
        end (int): last row, exclusive
        start (int): first row, inclusive
    Returns:
        list[Number]: The data in the column
    """
    return list(islice(_iter_act(), start, end))

# COMBINE FILES ================================

def get_data(key:str, end:int, start:int=0) -> list[Number]:
    """
    Read a slice of a column from any of the data files

    Args:
        key (str): column name
        end (int): last row, exclusive
        start (int): first row, inclusive
    Returns:
        list[Number]: The data in the column. Empty cells are `float("nan")`
    """
    # Validate range
    if start<0 or end<start:
        raise ValueError(f"need 0 <= start <= end, got start={start}, end={end}")

    # Check for raw index column
    if key==_COL_IDX:
        return range(start, end)

    # Check for Ping file
    if key==_COL_PING:
        return get_ping_data(end, start)

    # Check for Log file
    if key==_COL_RECEPTION:
        return get_log_data(end, start)

    # Check for Act file
    if key==_COL_ACTUATION:
        return get_act_data(end, start)

    # Assume CSV file
    return get_csv_data(key, end, start)

# PLOTTING DATA ================================

def plot_latencies():
    # All columns related to latency
    LATENCIES = [
        (_COL_FETCH_FRAME, "Fetching video frames"),
        (_COL_LANDMARK, "MediaPipe landmarking"),
        (_COL_CALC_TARGET, "Calculating target"),
        (_COL_IK_SOLVER, "IK solver"),
        (_COL_TRANSMISSION, "DTLS transmission"),
        (_COL_PING, "Packet travel time"),
        (_COL_RECEPTION, "DTLS reception"),
        (_COL_UART, "UART transmission"),
        (_COL_SERVO, "Exponential smoothing")
    ]

    # Extract data
    data, labels = [], []
    for key, label in LATENCIES:
        if (key==_COL_UART):
            uart_bytes = 2 + 6*4 + 2 # header + setpoints + crc16
            uart_bits = uart_bytes * (8+2) # start bit + byte + stop bit
            uart_time = uart_bits / 640000 * 1000 # 640,000 baud, time measured in milliseconds
            values = [uart_time]
        elif (key==_COL_SERVO):
            step_frames = 31
            step_time = step_frames / 30 * 1000 # 30fps camera, time measured in milliseconds
            values = [step_time]
        else:
            values = [v for v in get_data(key, _MAX) if not math.isnan(v)] # ignore empty cells
        data.append(values)
        labels.append(f"{label}\n(n={len(values)})")

    # Calculate statistics
    mean = []
    stdev = []
    num_samples = []
    total_mean = 0
    total_stdev = 0
    for i in range(len(LATENCIES)):
        mean.append(statistics.mean(data[i]))
        total_mean += mean[i]
        if len(data[i]) > 1:
            stdev.append(statistics.stdev(data[i]))
            num_samples.append(len(data[i]))
            total_stdev += stdev[i]**2
        else:
            stdev.append(None)
            num_samples.append(None)
    total_stdev = math.sqrt(total_stdev)

    # Print statistics
    for i in range(len(LATENCIES)):
        print(f"{LATENCIES[i][1]} & {mean[i]:.3g} & ", end="")
        if stdev[i] is None:
            print(f"N/A & N/A & ", end="")
        else:
            print(f"{stdev[i]:.3g} & {num_samples[i]} & ", end="")
        percentage = 100 * mean[i]/total_mean
        print(f"{percentage:.3g}% \\\\")
        print("\\hline")
    print("\\hline")
    print(f"Total & {total_mean:.3g} & {total_stdev:.3g} & N/A & 100% \\\\")
    print("\\hline")

    # Box and whisker plot
    fig, ax = plt.subplots(figsize=(11, 6))
    ax.boxplot(
        # Data
        data,
        tick_labels=labels,

        # Rendering
        showfliers=True,
        patch_artist=True,

        # Colors
        boxprops=dict(facecolor="0.8", edgecolor="black"),
        medianprops=dict(color="black", linewidth=2),
        whiskerprops=dict(color="black"),
        capprops=dict(color="black"),
        flierprops=dict(marker="o", markerfacecolor="0.5", markeredgecolor="black", markersize=4)
    )
    plt.setp(ax.get_xticklabels(), rotation=-45, ha="left")
    ax.set_yscale("log")     # the latencies span ~0.1 ms to ~150 ms; remove for linear
    ax.set_ylabel("Latency (ms)")
    # ax.set_title("Latency contributed by each stage")
    ax.grid(axis="y", which="both", alpha=0.3, linewidth=2)

    # Save and show
    fig.tight_layout()
    fig.savefig("plots/latency_boxplot.png", dpi=150)
    plt.show()

def plot_error():
    # FULL TIME SERIES ================

    # Extract data
    sqr_err = get_data(_COL_SQR_ERR, _MAX)

    # Plot line graph
    fig, ax = plt.subplots(figsize=(10, 3))
    ax.plot(range(len(sqr_err)), sqr_err, color="black", linewidth=2)
    ax.set_yscale("symlog", linthresh=1e-2)
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    ax.set_xlabel("Frame")
    ax.set_ylabel("Squared error (mm²)")
    # ax.set_title("Squared error throughout the teleoperation task")
    ax.grid(axis="y", which="both", alpha=0.3, linewidth=2)

    # Save and show
    fig.tight_layout()
    fig.savefig("plots/sqr_err.png", dpi=150)
    plt.show()

    # PARTIAL TIME SERIES ================

    START = 207
    END = 245

    # Extract data
    sub_sqr_err = get_data(_COL_SQR_ERR, END, START)

    # Plot line graph
    fig, ax = plt.subplots(figsize=(2, 3))
    ax.plot(range(START, END), sub_sqr_err, color="black", linewidth=2)
    # ax.set_yscale("symlog", linthresh=1e-2)
    ax.set_xlim(left=START)
    ax.set_ylim(bottom=0)
    ax.set_xlabel("Frame")
    ax.set_ylabel("Squared error (mm² linear)")
    # ax.set_title("Zoomed in portion of squared error")
    ax.grid(axis="y", which="both", alpha=0.3, linewidth=2)
    
    # Save and show
    fig.tight_layout()
    fig.savefig("plots/sub_sqr_err.png", dpi=150)
    plt.show()

def plot_angles():
    # Extract data
    angles = get_data(_COL_ACTUATION, _MAX)
    x = list(range(len(angles)))
    spline = make_smoothing_spline(x, angles, lam=2.0)
    x_series = np.linspace(0, x[len(x)-1], 500)

    # Plot line graph
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(range(len(angles)), angles, color="black", linewidth=3)
    ax.plot(x_series, spline(x_series), "--", color="black", linewidth=2, label="Smoothing spline")
    ax.set_ylim(bottom=0)
    ax.yaxis.set_major_locator(MultipleLocator(10))
    ax.set_xlabel("Frame (1/30 second)")
    ax.set_ylabel("Angle (degrees)")
    # ax.set_title("Braccio arm step response")
    ax.grid(axis="both", which="both", alpha=0.3, linewidth=2)

    # Save and show
    fig.tight_layout()
    fig.savefig("plots/actuation_line.png", dpi=150)
    plt.show()

def plot_latency_per_iter():
    # Extract data
    iterations = get_data(_COL_ITERATIONS, _MAX)
    latency = get_data(_COL_IK_SOLVER, _MAX)
    GROUPS = [0, 1, 4, 10000]
    group_data = [[], [], [], []]
    for ite, lat in zip(iterations, latency):
        if not math.isnan(ite):
            index = GROUPS.index(ite)
            group_data[index].append(lat)
    labels = [
        f"0 iterations\n(n={len(group_data[0])})",
        f"1 iterations\n(n={len(group_data[1])})",
        f"4 iterations\n(n={len(group_data[2])})",
        f"10,000 iterations\n(n={len(group_data[3])})"
    ]

    # Box and whisker plot
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.boxplot(
        # Data
        group_data,
        tick_labels=labels,

        # Rendering
        showfliers=True,
        patch_artist=True,

        # Colors
        boxprops=dict(facecolor="0.8", edgecolor="black"),
        medianprops=dict(color="black", linewidth=2),
        whiskerprops=dict(color="black"),
        capprops=dict(color="black"),
        flierprops=dict(marker="o", markerfacecolor="0.5", markeredgecolor="black", markersize=4)
    )
    # plt.setp(ax.get_xticklabels(), rotation=-45, ha="left")
    ax.set_yscale("log")     # the latencies span ~0.1 ms to ~150 ms; remove for linear
    ax.set_ylabel("Latency (ms)")
    # ax.set_title("Latency different number of iterations")
    ax.grid(axis="y", which="both", alpha=0.3, linewidth=2)

    # Save and show
    fig.tight_layout()
    fig.savefig("plots/latency_per_iter.png", dpi=150)
    plt.show()

def plot_error_per_iter():
    # Extract data
    iterations = get_data(_COL_ITERATIONS, _MAX)
    error = get_data(_COL_SQR_ERR, _MAX)
    GROUPS = [0, 1, 4, 10000]
    group_data = [[], [], [], []]
    for ite, err in zip(iterations, error):
        if not math.isnan(ite):
            index = GROUPS.index(ite)
            group_data[index].append(err)
    labels = [
        f"0 iterations\n(n={len(group_data[0])})",
        f"1 iterations\n(n={len(group_data[1])})",
        f"4 iterations\n(n={len(group_data[2])})",
        f"10,000 iterations\n(n={len(group_data[3])})"
    ]

    # Box and whisker plot
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.boxplot(
        # Data
        group_data,
        tick_labels=labels,

        # Rendering
        showfliers=True,
        patch_artist=True,

        # Colors
        boxprops=dict(facecolor="0.8", edgecolor="black"),
        medianprops=dict(color="black", linewidth=2),
        whiskerprops=dict(color="black"),
        capprops=dict(color="black"),
        flierprops=dict(marker="o", markerfacecolor="0.5", markeredgecolor="black", markersize=4)
    )
    # plt.setp(ax.get_xticklabels(), rotation=-45, ha="left")
    ax.set_yscale("log")     # the latencies span ~0.1 ms to ~150 ms; remove for linear
    ax.set_ylabel("Error (mm²)")
    # ax.set_title("Error per different number of iterations")
    ax.grid(axis="y", which="both", alpha=0.3, linewidth=2)

    # Save and show
    fig.tight_layout()
    fig.savefig("plots/error_per_iter.png", dpi=150)
    plt.show()

# MAIN ================================

if __name__ == "__main__":
    # plot_latencies();
    plot_error()
    # plot_angles()
    # plot_latency_per_iter()
    # plot_error_per_iter()
