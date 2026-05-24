#!/usr/bin/env python3
# Copyright 2026 px4_ros2_ctrl contributors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Fit hover-thrust voltage compensation parameters from calibration CSV data."""

import argparse
import csv
import json
import math
import statistics
from pathlib import Path


G = 9.80665


def parse_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def numeric_values(row):
    values = []
    for item in row:
        value = parse_float(item)
        if value is not None and math.isfinite(value):
            values.append(value)
    return values


def looks_like_command(value):
    return 0.0 <= value <= 2.0


def looks_like_voltage(value):
    return 2.0 <= value <= 100.0


def extract_mass(row):
    for index, item in enumerate(row):
        if item.strip().lower().startswith("mass"):
            for candidate in row[index + 1:]:
                value = parse_float(candidate)
                if value is not None and value > 0.0:
                    return value
    return None


def load_records(csv_path):
    rows = []
    mass_kg = None

    with open(csv_path, newline="") as file:
        reader = csv.reader(file)
        for row in reader:
            row = [item.strip() for item in row if item.strip()]
            if not row:
                continue
            rows.append(row)
            row_mass = extract_mass(row)
            if row_mass is not None:
                mass_kg = row_mass

    records = []
    numeric_rows = []

    for row in rows:
        if extract_mass(row) is not None:
            continue

        values = numeric_values(row)
        if not values:
            continue

        numeric_rows.append(values)

        if len(values) == 2:
            command, voltage = values
            if looks_like_command(command) and looks_like_voltage(voltage):
                records.append((command, voltage))

    if records:
        return records, mass_kg

    # Legacy thrust_calibrate.py wrote all commands on one row and all voltages
    # on the next row. Handle that layout if pair rows were not found.
    for first, second in zip(numeric_rows, numeric_rows[1:]):
        if len(first) != len(second):
            continue
        if not first:
            continue
        if all(looks_like_command(value) for value in first) and all(
            looks_like_voltage(value) for value in second
        ):
            return list(zip(first, second)), mass_kg

    return [], mass_kg


def linear_regression(xs, ys):
    if len(xs) != len(ys) or len(xs) < 2:
        raise ValueError("need at least two samples for regression")

    x_mean = statistics.fmean(xs)
    y_mean = statistics.fmean(ys)
    sxx = sum((x - x_mean) ** 2 for x in xs)
    if sxx <= 0.0:
        raise ValueError("input samples do not span enough voltage range")
    sxy = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys))

    slope = sxy / sxx
    intercept = y_mean - slope * x_mean
    predicted = [slope * x + intercept for x in xs]
    return slope, intercept, metrics(ys, predicted)


def metrics(actual, predicted):
    residuals = [y - y_hat for y, y_hat in zip(actual, predicted)]
    mse = statistics.fmean(value * value for value in residuals)
    rmse = math.sqrt(mse)
    y_mean = statistics.fmean(actual)
    sst = sum((value - y_mean) ** 2 for value in actual)
    sse = sum(value * value for value in residuals)
    r2 = 1.0 - sse / sst if sst > 0.0 else 1.0
    return {"rmse": rmse, "r2": r2}


def fit_power_model(records, nominal_voltage):
    commands = []
    voltages = []
    for command, voltage in records:
        if command > 0.0 and voltage > 0.0:
            commands.append(command)
            voltages.append(voltage)

    xs = [math.log(nominal_voltage / voltage) for voltage in voltages]
    ys = [math.log(command) for command in commands]
    exponent, log_hover_command = linear_regression(xs, ys)[:2]
    hover_command = math.exp(log_hover_command)
    predicted = [
        hover_command * (nominal_voltage / voltage) ** exponent
        for voltage in voltages
    ]
    return {
        "model": "command = hover_command * (nominal_voltage / voltage) ^ exponent",
        "nominal_voltage": nominal_voltage,
        "hover_command": hover_command,
        "voltage_exponent": exponent,
        **metrics(commands, predicted),
    }


def fit_linear_model(records, nominal_voltage):
    commands = [command for command, _ in records]
    voltages = [voltage for _, voltage in records]
    slope, intercept, fit_metrics = linear_regression(voltages, commands)
    return {
        "model": "command = slope * voltage + intercept",
        "nominal_voltage": nominal_voltage,
        "hover_command": slope * nominal_voltage + intercept,
        "slope": slope,
        "intercept": intercept,
        **fit_metrics,
    }


def add_force_estimates(result, mass_kg):
    if mass_kg is None or result["hover_command"] <= 0.0:
        return result

    weight_n = mass_kg * G
    result["mass_kg"] = mass_kg
    result["weight_n"] = weight_n
    result["newtons_per_normalized_command_at_nominal_voltage"] = (
        weight_n / result["hover_command"]
    )
    return result


def write_outputs(result, output_path):
    text = json.dumps(result, indent=2, sort_keys=True)
    print(text)
    if output_path:
        Path(output_path).write_text(text + "\n")


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Fit thrust-command voltage compensation parameters from a CSV "
            "recorded by thrust_calibration_node."
        )
    )
    parser.add_argument("csv", help="Calibration CSV file")
    parser.add_argument(
        "--model",
        choices=["power", "linear"],
        default="power",
        help="Fit model. The power model is usually better for voltage compensation.",
    )
    parser.add_argument(
        "--nominal-voltage",
        type=float,
        default=None,
        help="Nominal voltage used for hover_command. Defaults to max recorded voltage.",
    )
    parser.add_argument(
        "--mass-kg",
        type=float,
        default=None,
        help="Override mass in kg. If omitted, the script reads mass from CSV header.",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Optional JSON output path.",
    )
    args = parser.parse_args()

    records, csv_mass_kg = load_records(args.csv)
    if len(records) < 2:
        raise SystemExit("Need at least two command-voltage records to fit parameters")

    nominal_voltage = args.nominal_voltage
    if nominal_voltage is None:
        nominal_voltage = max(voltage for _, voltage in records)

    if args.model == "power":
        result = fit_power_model(records, nominal_voltage)
    else:
        result = fit_linear_model(records, nominal_voltage)

    mass_kg = args.mass_kg if args.mass_kg is not None else csv_mass_kg
    result = add_force_estimates(result, mass_kg)
    result["sample_count"] = len(records)
    result["source_csv"] = str(args.csv)
    result["assumption"] = (
        "Samples are treated as hover or near-hover points, so average required "
        "thrust is approximately mass * g. Add acceleration data for a full "
        "dynamic thrust model."
    )

    write_outputs(result, args.output)


if __name__ == "__main__":
    main()
